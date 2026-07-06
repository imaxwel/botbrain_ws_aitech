import { NextRequest, NextResponse } from 'next/server';

const DEFAULT_SUPERVISOR_URL =
  process.env.MISSION_SUPERVISOR_URL ??
  process.env.NEXT_PUBLIC_MISSION_SUPERVISOR_URL ??
  'http://127.0.0.1:8787';

const PROXY_TIMEOUT_MS = Number(
  process.env.MISSION_SUPERVISOR_PROXY_TIMEOUT_MS ?? '8000'
);

const ALLOWED_GET_PATHS = new Set([
  'healthz',
  'snapshot',
  'events',
  'stream',
  'visualization/model',
  'exceptions/catalog',
  'preflight',
  'voice-context',
  'voice/sessions',
  'voice/transcript',
  'auth/demo-code',
  'pending-decisions',
  'adapter-status',
  'testing/playback/scenarios',
]);
const ALLOWED_POST_PATHS = new Set([
  'instruction',
  'intent',
  'operator-decision',
  'control',
  'tick',
  'testing/playback/sessions',
]);

type RouteContext = {
  params: Promise<{ path?: string[] }>;
};

function isPrivateIPv4(hostname: string): boolean {
  const parts = hostname.split('.').map((part) => Number(part));

  if (
    parts.length !== 4 ||
    parts.some((part) => !Number.isInteger(part) || part < 0 || part > 255)
  ) {
    return false;
  }

  const [first, second] = parts;

  return (
    first === 10 ||
    first === 127 ||
    (first === 172 && second >= 16 && second <= 31) ||
    (first === 192 && second === 168) ||
    (first === 169 && second === 254)
  );
}

function validateClientTarget(baseUrl: string): URL {
  const url = new URL(baseUrl);
  const hostname = url.hostname.toLowerCase();

  if (url.protocol !== 'http:' && url.protocol !== 'https:') {
    throw new Error('Mission Supervisor target must use http or https');
  }

  const isLocalhost =
    hostname === 'localhost' ||
    hostname === '::1' ||
    hostname === '[::1]' ||
    hostname === '0.0.0.0';

  if (!isLocalhost && !isPrivateIPv4(hostname)) {
    throw new Error('Mission Supervisor target must be localhost or a private IPv4 address');
  }

  return url;
}

function trustedEnvTarget(): URL {
  const url = new URL(DEFAULT_SUPERVISOR_URL);

  if (url.protocol !== 'http:' && url.protocol !== 'https:') {
    throw new Error('MISSION_SUPERVISOR_URL must use http or https');
  }

  return url;
}

async function getForwardPath(context: RouteContext): Promise<string> {
  const params = await context.params;
  return (params.path ?? []).join('/');
}

function resolveTarget(req: NextRequest): URL {
  const clientTarget = req.nextUrl.searchParams.get('target');
  return clientTarget ? validateClientTarget(clientTarget) : trustedEnvTarget();
}

function buildUpstreamUrl(req: NextRequest, target: URL, path: string): URL {
  const upstream = new URL(path, `${target.toString().replace(/\/+$/, '')}/`);

  req.nextUrl.searchParams.forEach((value, key) => {
    if (key !== 'target') {
      upstream.searchParams.append(key, value);
    }
  });

  return upstream;
}

function isAllowedPath(path: string, method: 'GET' | 'POST'): boolean {
  const staticAllowed =
    method === 'GET'
      ? ALLOWED_GET_PATHS.has(path)
      : ALLOWED_POST_PATHS.has(path);

  if (staticAllowed) {
    return true;
  }

  if (method === 'GET') {
    return (
      /^voice\/sessions\/[^/]+\/transcript$/.test(path) ||
      /^testing\/playback\/sessions\/[^/]+$/.test(path) ||
      /^testing\/playback\/sessions\/[^/]+\/transcript$/.test(path) ||
      /^testing\/playback\/sessions\/[^/]+\/report$/.test(path)
    );
  }

  return /^testing\/playback\/sessions\/[^/]+\/control$/.test(path);
}

async function forwardRequest(
  req: NextRequest,
  context: RouteContext,
  method: 'GET' | 'POST'
) {
  try {
    const path = await getForwardPath(context);

    if (!isAllowedPath(path, method)) {
      return NextResponse.json(
        { error: `Mission Supervisor path is not allowed: ${path}` },
        { status: 404 }
      );
    }

    const target = resolveTarget(req);
    const upstream = buildUpstreamUrl(req, target, path);

    if (method === 'GET' && path === 'stream') {
      return await forwardStreamRequest(upstream);
    }

    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), PROXY_TIMEOUT_MS);

    const body = method === 'POST' ? await req.text() : undefined;

    try {
      const response = await fetch(upstream, {
        method,
        headers:
          body === undefined
            ? {
                Accept: 'application/json',
              }
            : {
                Accept: 'application/json',
                'Content-Type':
                  req.headers.get('content-type') ?? 'application/json',
              },
        body,
        cache: 'no-store',
        signal: controller.signal,
      });

      const responseBody = await response.text();
      const contentType =
        response.headers.get('content-type') ?? 'application/json';

      return new NextResponse(responseBody, {
        status: response.status,
        headers: {
          'Content-Type': contentType,
          'Cache-Control': 'no-store',
        },
      });
    } finally {
      clearTimeout(timeout);
    }
  } catch (error) {
    const message =
      error instanceof Error ? error.message : 'Mission Supervisor proxy error';

    return NextResponse.json({ error: message }, { status: 502 });
  }
}

async function forwardStreamRequest(upstream: URL) {
  const response = await fetch(upstream, {
    method: 'GET',
    headers: {
      Accept: 'text/event-stream',
    },
    cache: 'no-store',
  });

  return new NextResponse(response.body, {
    status: response.status,
    headers: {
      'Content-Type': response.headers.get('content-type') ?? 'text/event-stream',
      'Cache-Control': 'no-store, no-transform',
      'X-Accel-Buffering': 'no',
    },
  });
}

export async function GET(req: NextRequest, context: RouteContext) {
  return forwardRequest(req, context, 'GET');
}

export async function POST(req: NextRequest, context: RouteContext) {
  return forwardRequest(req, context, 'POST');
}
