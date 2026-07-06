import type {
  AdapterStatusSnapshot,
  BTStateSnapshot,
  DemoAuthCodeState,
  ExceptionCatalog,
  MissionEvent,
  MissionControlCommand,
  MissionControlResult,
  MissionSupervisorHealth,
  MissionVoiceContext,
  OperatorDecisionCommand,
  PlaybackControlCommand,
  PlaybackCreateCommand,
  PlaybackScenariosResponse,
  PlaybackSessionSnapshot,
  PendingOperatorDecision,
  PreflightSnapshot,
  UserInstructionCommand,
  VoiceSessionsResponse,
  VoiceTranscriptSnapshot,
  VisualizationModel,
} from '@/types/mission-control';

const PROXY_ROOT = '/api/mission-supervisor';

export const MISSION_SUPERVISOR_BASE_URL_STORAGE_KEY =
  'botbrain_mission_supervisor_base_url';

const DEFAULT_BASE_URL =
  process.env.NEXT_PUBLIC_MISSION_SUPERVISOR_URL ?? 'http://127.0.0.1:8787';

interface RequestOptions {
  method?: 'GET' | 'POST';
  body?: unknown;
  baseUrlOverride?: string;
}

function normalizeBaseUrl(baseUrl: string): string {
  return baseUrl.trim().replace(/\/+$/, '');
}

function buildProxyUrl(path: string, baseUrlOverride?: string): string {
  const cleanPath = path.replace(/^\/+/, '');
  const url = new URL(`${PROXY_ROOT}/${cleanPath}`, window.location.origin);

  if (baseUrlOverride?.trim()) {
    url.searchParams.set('target', normalizeBaseUrl(baseUrlOverride));
  }

  return url.toString();
}

async function requestJson<T>(
  path: string,
  options: RequestOptions = {}
): Promise<T> {
  const response = await fetch(buildProxyUrl(path, options.baseUrlOverride), {
    method: options.method ?? 'GET',
    headers:
      options.body === undefined
        ? undefined
        : {
            'Content-Type': 'application/json',
          },
    body: options.body === undefined ? undefined : JSON.stringify(options.body),
    cache: 'no-store',
  });

  if (!response.ok) {
    let message = `Mission Supervisor request failed with ${response.status}`;
    const text = await response.text();

    try {
      const payload = JSON.parse(text);
      if (typeof payload?.detail === 'string') {
        message = payload.detail;
      } else if (typeof payload?.error === 'string') {
        message = payload.error;
      }
    } catch {
      if (text) {
        message = text;
      }
    }

    throw new Error(message);
  }

  return response.json() as Promise<T>;
}

export const missionSupervisorService = {
  defaultBaseUrl: DEFAULT_BASE_URL,

  getHealth(baseUrlOverride?: string): Promise<MissionSupervisorHealth> {
    return requestJson<MissionSupervisorHealth>('healthz', { baseUrlOverride });
  },

  getSnapshot(baseUrlOverride?: string): Promise<BTStateSnapshot> {
    return requestJson<BTStateSnapshot>('snapshot', { baseUrlOverride });
  },

  getPreflight(baseUrlOverride?: string): Promise<PreflightSnapshot> {
    return requestJson<PreflightSnapshot>('preflight', { baseUrlOverride });
  },

  getVoiceContext(baseUrlOverride?: string): Promise<MissionVoiceContext> {
    return requestJson<MissionVoiceContext>('voice-context', { baseUrlOverride });
  },

  getVoiceSessions(baseUrlOverride?: string): Promise<VoiceSessionsResponse> {
    return requestJson<VoiceSessionsResponse>('voice/sessions', {
      baseUrlOverride,
    });
  },

  getLatestVoiceTranscript(
    baseUrlOverride?: string
  ): Promise<VoiceTranscriptSnapshot> {
    return requestJson<VoiceTranscriptSnapshot>('voice/transcript', {
      baseUrlOverride,
    });
  },

  getVoiceTranscript(
    conversationId: string,
    baseUrlOverride?: string
  ): Promise<VoiceTranscriptSnapshot> {
    return requestJson<VoiceTranscriptSnapshot>(
      `voice/sessions/${encodeURIComponent(conversationId)}/transcript`,
      { baseUrlOverride }
    );
  },

  getDemoAuthCode(baseUrlOverride?: string): Promise<DemoAuthCodeState> {
    return requestJson<DemoAuthCodeState>('auth/demo-code', { baseUrlOverride });
  },

  getAdapterStatus(baseUrlOverride?: string): Promise<AdapterStatusSnapshot> {
    return requestJson<AdapterStatusSnapshot>('adapter-status', { baseUrlOverride });
  },

  getPendingDecisions(
    baseUrlOverride?: string
  ): Promise<PendingOperatorDecision[]> {
    return requestJson<PendingOperatorDecision[]>('pending-decisions', {
      baseUrlOverride,
    });
  },

  getEvents(baseUrlOverride?: string): Promise<MissionEvent[]> {
    return requestJson<MissionEvent[]>('events', { baseUrlOverride });
  },

  getStreamUrl(baseUrlOverride?: string): string {
    return buildProxyUrl('stream', baseUrlOverride);
  },

  getVisualizationModel(baseUrlOverride?: string): Promise<VisualizationModel> {
    return requestJson<VisualizationModel>('visualization/model', {
      baseUrlOverride,
    });
  },

  getExceptionCatalog(baseUrlOverride?: string): Promise<ExceptionCatalog> {
    return requestJson<ExceptionCatalog>('exceptions/catalog', {
      baseUrlOverride,
    });
  },

  getPlaybackScenarios(baseUrlOverride?: string): Promise<PlaybackScenariosResponse> {
    return requestJson<PlaybackScenariosResponse>('testing/playback/scenarios', {
      baseUrlOverride,
    });
  },

  createPlaybackSession(
    command: PlaybackCreateCommand,
    baseUrlOverride?: string
  ): Promise<PlaybackSessionSnapshot> {
    return requestJson<PlaybackSessionSnapshot>('testing/playback/sessions', {
      method: 'POST',
      body: {
        schema_version: 'g1.testing.playback.create.v1',
        pace: 'manual',
        step_delay_seconds: 2,
        reset_supervisor: true,
        ...command,
      },
      baseUrlOverride,
    });
  },

  getPlaybackSession(
    sessionId: string,
    baseUrlOverride?: string
  ): Promise<PlaybackSessionSnapshot> {
    return requestJson<PlaybackSessionSnapshot>(
      `testing/playback/sessions/${encodeURIComponent(sessionId)}`,
      { baseUrlOverride }
    );
  },

  controlPlaybackSession(
    sessionId: string,
    command: PlaybackControlCommand,
    baseUrlOverride?: string
  ): Promise<PlaybackSessionSnapshot> {
    return requestJson<PlaybackSessionSnapshot>(
      `testing/playback/sessions/${encodeURIComponent(sessionId)}/control`,
      {
        method: 'POST',
        body: {
          schema_version: 'g1.testing.playback.control.v1',
          ...command,
        },
        baseUrlOverride,
      }
    );
  },

  getPlaybackReport(
    sessionId: string,
    baseUrlOverride?: string
  ): Promise<Record<string, unknown>> {
    return requestJson<Record<string, unknown>>(
      `testing/playback/sessions/${encodeURIComponent(sessionId)}/report`,
      { baseUrlOverride }
    );
  },

  submitInstruction(
    command: UserInstructionCommand,
    baseUrlOverride?: string
  ): Promise<BTStateSnapshot> {
    return requestJson<BTStateSnapshot>('instruction', {
      method: 'POST',
      body: {
        schema_version: 'g1.user_instruction.v1',
        source: 'botbrain_3000',
        modality: 'text',
        language: 'zh',
        ...command,
      },
      baseUrlOverride,
    });
  },

  tick(baseUrlOverride?: string): Promise<BTStateSnapshot> {
    return requestJson<BTStateSnapshot>('tick', {
      method: 'POST',
      body: {},
      baseUrlOverride,
    });
  },

  submitOperatorDecision(
    command: OperatorDecisionCommand,
    baseUrlOverride?: string
  ): Promise<BTStateSnapshot> {
    return requestJson<BTStateSnapshot>('operator-decision', {
      method: 'POST',
      body: {
        schema_version: 'g1.operator_command.v1',
        ...command,
      },
      baseUrlOverride,
    });
  },

  submitControl(
    command: MissionControlCommand,
    baseUrlOverride?: string
  ): Promise<MissionControlResult> {
    return requestJson<MissionControlResult>('control', {
      method: 'POST',
      body: {
        schema_version: 'g1.mission.control_command.v1',
        ...command,
      },
      baseUrlOverride,
    });
  },
};
