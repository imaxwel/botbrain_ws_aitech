'use client';

import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
} from 'react';
import { useNotifications } from '@/contexts/NotificationsContext';
import {
  MISSION_SUPERVISOR_BASE_URL_STORAGE_KEY,
  missionSupervisorService,
} from '@/services/mission-supervisor';
import type {
  AdapterStatusSnapshot,
  BTStateSnapshot,
  DemoAuthCodeState,
  ExceptionCatalog,
  MissionEvent,
  MissionControlAction,
  MissionControlResult,
  MissionSupervisorConnectionStatus,
  MissionSupervisorHealth,
  MissionVoiceContext,
  OperatorDecisionAction,
  PendingOperatorDecision,
  PreflightSnapshot,
  VisualizationModel,
} from '@/types/mission-control';

const POLL_INTERVAL_MS = 1000;
const STALE_AFTER_MS = 2500;
const MAX_EVENTS = 200;

interface MissionSupervisorContextValue {
  baseUrlOverride: string;
  defaultBaseUrl: string;
  effectiveBaseUrl: string;
  connectionStatus: MissionSupervisorConnectionStatus;
  health: MissionSupervisorHealth | null;
  preflight: PreflightSnapshot | null;
  voiceContext: MissionVoiceContext | null;
  adapterStatus: AdapterStatusSnapshot | null;
  snapshot: BTStateSnapshot | null;
  visualizationModel: VisualizationModel | null;
  exceptionCatalog: ExceptionCatalog | null;
  events: MissionEvent[];
  pendingDecisions: PendingOperatorDecision[];
  demoAuthCode: DemoAuthCodeState | null;
  lastError: string | null;
  lastSnapshotAt: number | null;
  isRefreshing: boolean;
  isStale: boolean;
  setBaseUrlOverride: (baseUrl: string) => void;
  resetBaseUrlOverride: () => void;
  refresh: () => Promise<void>;
  submitInstruction: (text: string, language?: string) => Promise<void>;
  tick: () => Promise<void>;
  submitDecision: (
    decision: PendingOperatorDecision,
    action: OperatorDecisionAction,
    operatorId: string,
    comment?: string
  ) => Promise<void>;
  submitControl: (
    command: MissionControlAction,
    operatorId: string,
    reason?: string
  ) => Promise<MissionControlResult>;
}

const MissionSupervisorContext =
  createContext<MissionSupervisorContextValue | null>(null);

export function MissionSupervisorProvider({
  children,
}: {
  children: React.ReactNode;
}) {
  const { dispatch: notificationDispatch } = useNotifications();
  const [baseUrlOverride, setBaseUrlOverrideState] = useState('');
  const [connectionStatus, setConnectionStatus] =
    useState<MissionSupervisorConnectionStatus>('idle');
  const [health, setHealth] = useState<MissionSupervisorHealth | null>(null);
  const [preflight, setPreflight] = useState<PreflightSnapshot | null>(null);
  const [voiceContext, setVoiceContext] = useState<MissionVoiceContext | null>(
    null
  );
  const [adapterStatus, setAdapterStatus] =
    useState<AdapterStatusSnapshot | null>(null);
  const [snapshot, setSnapshot] = useState<BTStateSnapshot | null>(null);
  const [visualizationModel, setVisualizationModel] =
    useState<VisualizationModel | null>(null);
  const [exceptionCatalog, setExceptionCatalog] =
    useState<ExceptionCatalog | null>(null);
  const [events, setEvents] = useState<MissionEvent[]>([]);
  const [pendingDecisions, setPendingDecisions] = useState<
    PendingOperatorDecision[]
  >([]);
  const [demoAuthCode, setDemoAuthCode] = useState<DemoAuthCodeState | null>(
    null
  );
  const [lastError, setLastError] = useState<string | null>(null);
  const [lastSnapshotAt, setLastSnapshotAt] = useState<number | null>(null);
  const [isRefreshing, setIsRefreshing] = useState(false);
  const [now, setNow] = useState(Date.now());
  const refreshInFlightRef = useRef(false);
  const staticDataInFlightRef = useRef(false);
  const staticDataRequestIdRef = useRef(0);
  const notifiedDecisionIds = useRef<Set<string>>(new Set());
  const notifiedExceptionIds = useRef<Set<string>>(new Set());

  const effectiveBaseUrl = baseUrlOverride || missionSupervisorService.defaultBaseUrl;
  const requestBaseUrl = baseUrlOverride || undefined;

  useEffect(() => {
    const stored = window.localStorage.getItem(
      MISSION_SUPERVISOR_BASE_URL_STORAGE_KEY
    );

    if (stored) {
      setBaseUrlOverrideState(stored);
    }
  }, []);

  useEffect(() => {
    const timer = window.setInterval(() => setNow(Date.now()), 500);
    return () => window.clearInterval(timer);
  }, []);

  const loadStaticSupervisorData = useCallback(async () => {
    if (staticDataInFlightRef.current) {
      return;
    }

    staticDataInFlightRef.current = true;
    const requestId = staticDataRequestIdRef.current + 1;
    staticDataRequestIdRef.current = requestId;

    try {
      const [modelResult, catalogResult] = await Promise.allSettled([
        missionSupervisorService.getVisualizationModel(requestBaseUrl),
        missionSupervisorService.getExceptionCatalog(requestBaseUrl),
      ]);

      if (requestId !== staticDataRequestIdRef.current) {
        return;
      }

      if (modelResult.status === 'fulfilled') {
        setVisualizationModel(modelResult.value);
      }

      if (catalogResult.status === 'fulfilled') {
        setExceptionCatalog(catalogResult.value);
      }
    } finally {
      if (requestId === staticDataRequestIdRef.current) {
        staticDataInFlightRef.current = false;
      }
    }
  }, [requestBaseUrl]);

  useEffect(() => {
    let cancelled = false;

    staticDataRequestIdRef.current += 1;
    staticDataInFlightRef.current = false;
    setVisualizationModel(null);
    setExceptionCatalog(null);

    const run = async () => {
      if (!cancelled) {
        await loadStaticSupervisorData();
      }
    };

    run();

    return () => {
      cancelled = true;
      staticDataRequestIdRef.current += 1;
      staticDataInFlightRef.current = false;
    };
  }, [loadStaticSupervisorData]);

  const refresh = useCallback(async () => {
    if (refreshInFlightRef.current) {
      return;
    }

    refreshInFlightRef.current = true;
    setIsRefreshing(true);
    setConnectionStatus((current) =>
      current === 'idle' ? 'connecting' : current
    );

    try {
      const [
        nextHealth,
        nextPreflight,
        nextVoiceContext,
        nextDemoAuthCode,
        nextAdapterStatus,
        nextSnapshot,
        nextPendingDecisions,
        nextEvents,
      ] = await Promise.all([
        missionSupervisorService.getHealth(requestBaseUrl),
        missionSupervisorService.getPreflight(requestBaseUrl),
        missionSupervisorService.getVoiceContext(requestBaseUrl),
        missionSupervisorService.getDemoAuthCode(requestBaseUrl),
        missionSupervisorService.getAdapterStatus(requestBaseUrl),
        missionSupervisorService.getSnapshot(requestBaseUrl),
        missionSupervisorService.getPendingDecisions(requestBaseUrl),
        missionSupervisorService.getEvents(requestBaseUrl),
      ]);

      setHealth(nextHealth);
      setPreflight(nextPreflight);
      setVoiceContext(nextVoiceContext);
      setDemoAuthCode(nextDemoAuthCode);
      setAdapterStatus(nextAdapterStatus);
      setSnapshot(nextSnapshot);
      setPendingDecisions(nextPendingDecisions);
      setEvents(nextEvents.slice(-MAX_EVENTS));
      setLastSnapshotAt(Date.now());
      setLastError(null);
      setConnectionStatus('connected');

      if (!visualizationModel || !exceptionCatalog) {
        void loadStaticSupervisorData();
      }
    } catch (error) {
      setConnectionStatus('error');
      setLastError(error instanceof Error ? error.message : 'Connection failed');
    } finally {
      refreshInFlightRef.current = false;
      setIsRefreshing(false);
    }
  }, [
    exceptionCatalog,
    loadStaticSupervisorData,
    requestBaseUrl,
    visualizationModel,
  ]);

  useEffect(() => {
    let cancelled = false;

    const run = async () => {
      if (!cancelled) {
        await refresh();
      }
    };

    run();
    const interval = window.setInterval(run, POLL_INTERVAL_MS);

    return () => {
      cancelled = true;
      window.clearInterval(interval);
    };
  }, [refresh]);

  useEffect(() => {
    if (typeof window.EventSource === 'undefined') {
      return undefined;
    }

    let closed = false;
    const source = new window.EventSource(
      missionSupervisorService.getStreamUrl(requestBaseUrl)
    );

    const handleOpen = () => {
      if (closed) return;
      setConnectionStatus('connected');
      setLastError(null);
    };

    const handleSnapshot = (event: Event) => {
      if (closed) return;
      const nextSnapshot = parseStreamData<BTStateSnapshot>(event);
      if (!nextSnapshot) return;

      setSnapshot(nextSnapshot);
      setPendingDecisions(
        nextSnapshot.pending_decisions.filter(
          (decision) => decision.status === 'PENDING'
        )
      );
      setLastSnapshotAt(Date.now());
      setConnectionStatus('connected');
      setLastError(null);
    };

    const handleMissionEvent = (event: Event) => {
      if (closed) return;
      const nextEvent = parseStreamData<MissionEvent>(event);
      if (!nextEvent) return;

      setEvents((current) => appendMissionEvent(current, nextEvent));
      setConnectionStatus('connected');
      setLastError(null);
    };

    const handleHeartbeat = () => {
      if (closed) return;
      setConnectionStatus('connected');
      setLastError(null);
    };

    const handleError = () => {
      if (closed) return;
    };

    source.addEventListener('open', handleOpen);
    source.addEventListener('snapshot', handleSnapshot);
    source.addEventListener('mission_event', handleMissionEvent);
    source.addEventListener('heartbeat', handleHeartbeat);
    source.addEventListener('error', handleError);

    return () => {
      closed = true;
      source.removeEventListener('open', handleOpen);
      source.removeEventListener('snapshot', handleSnapshot);
      source.removeEventListener('mission_event', handleMissionEvent);
      source.removeEventListener('heartbeat', handleHeartbeat);
      source.removeEventListener('error', handleError);
      source.close();
    };
  }, [requestBaseUrl]);

  useEffect(() => {
    for (const decision of pendingDecisions) {
      if (!notifiedDecisionIds.current.has(decision.decision_id)) {
        notifiedDecisionIds.current.add(decision.decision_id);
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: decision.risk === 'LOW' ? 'info' : 'warning',
            title: `${decision.risk} decision pending`,
            message: decision.kind,
          },
        });
      }
    }
  }, [notificationDispatch, pendingDecisions]);

  useEffect(() => {
    const activeException = snapshot?.blackboard_summary.active_exception;
    if (!activeException) {
      return;
    }

    if (notifiedExceptionIds.current.has(activeException.exception_instance_id)) {
      return;
    }

    notifiedExceptionIds.current.add(activeException.exception_instance_id);
    notificationDispatch({
      type: 'ADD_NOTIFICATION',
      payload: {
        type:
          activeException.severity === 'P0' || activeException.severity === 'P1'
            ? 'warning'
            : 'info',
        title: `${activeException.severity} exception`,
        message: activeException.title || activeException.exception_id,
      },
    });
  }, [notificationDispatch, snapshot?.blackboard_summary.active_exception]);

  const setBaseUrlOverride = useCallback((baseUrl: string) => {
    const normalized = baseUrl.trim().replace(/\/+$/, '');
    setBaseUrlOverrideState(normalized);

    if (normalized) {
      window.localStorage.setItem(
        MISSION_SUPERVISOR_BASE_URL_STORAGE_KEY,
        normalized
      );
    } else {
      window.localStorage.removeItem(MISSION_SUPERVISOR_BASE_URL_STORAGE_KEY);
    }
  }, []);

  const resetBaseUrlOverride = useCallback(() => {
    setBaseUrlOverride('');
  }, [setBaseUrlOverride]);

  const submitDecision = useCallback(
    async (
      decision: PendingOperatorDecision,
      action: OperatorDecisionAction,
      operatorId: string,
      comment?: string
    ) => {
      const nextSnapshot = await missionSupervisorService.submitOperatorDecision(
        {
          command_id: newClientId('cmd'),
          idempotency_key: newClientId('idem'),
          decision_id: decision.decision_id,
          mission_run_id: decision.mission_run_id,
          action,
          expected_state_version: snapshot?.state_version ?? decision.state_version,
          operator_id: operatorId.trim() || 'botbrain-operator',
          comment: comment?.trim() || undefined,
        },
        requestBaseUrl
      );

      setSnapshot(nextSnapshot);
      setLastSnapshotAt(Date.now());
      await refresh();
    },
    [refresh, requestBaseUrl, snapshot?.state_version]
  );

  const submitInstruction = useCallback(
    async (text: string, language = 'zh') => {
      const nextSnapshot = await missionSupervisorService.submitInstruction(
        {
          event_id: newClientId('text'),
          idempotency_key: newClientId('idem'),
          source: 'botbrain_3000',
          modality: 'text',
          robot_id: snapshot?.robot_id,
          mission_run_id: snapshot?.mission_run_id ?? null,
          language,
          text,
          state_version: snapshot?.state_version ?? 0,
        },
        requestBaseUrl
      );

      setSnapshot(nextSnapshot);
      setPendingDecisions(
        nextSnapshot.pending_decisions.filter(
          (decision) => decision.status === 'PENDING'
        )
      );
      setLastSnapshotAt(Date.now());
      await refresh();
    },
    [
      refresh,
      requestBaseUrl,
      snapshot?.mission_run_id,
      snapshot?.robot_id,
      snapshot?.state_version,
    ]
  );

  const tick = useCallback(async () => {
    const nextSnapshot = await missionSupervisorService.tick(requestBaseUrl);

    setSnapshot(nextSnapshot);
    setPendingDecisions(
      nextSnapshot.pending_decisions.filter(
        (decision) => decision.status === 'PENDING'
      )
    );
    setLastSnapshotAt(Date.now());
    await refresh();
  }, [refresh, requestBaseUrl]);

  const submitControl = useCallback(
    async (
      command: MissionControlAction,
      operatorId: string,
      reason?: string
    ) => {
      const result = await missionSupervisorService.submitControl(
        {
          command_id: newClientId('cmd'),
          idempotency_key: newClientId('idem'),
          mission_run_id: snapshot?.mission_run_id ?? null,
          command,
          expected_state_version: snapshot?.state_version,
          operator_id: operatorId.trim() || 'botbrain-operator',
          reason: reason?.trim() || undefined,
        },
        requestBaseUrl
      );

      setSnapshot(result.snapshot);
      setPendingDecisions(
        result.snapshot.pending_decisions.filter(
          (decision) => decision.status === 'PENDING'
        )
      );
      setLastSnapshotAt(Date.now());
      await refresh();
      return result;
    },
    [refresh, requestBaseUrl, snapshot?.mission_run_id, snapshot?.state_version]
  );

  const visiblePendingDecisions = useMemo(
    () => pendingDecisions.filter((decision) => decision.status === 'PENDING'),
    [pendingDecisions]
  );

  const isStale =
    lastSnapshotAt === null ? true : now - lastSnapshotAt > STALE_AFTER_MS;

  const value = useMemo<MissionSupervisorContextValue>(
    () => ({
      baseUrlOverride,
      defaultBaseUrl: missionSupervisorService.defaultBaseUrl,
      effectiveBaseUrl,
      connectionStatus,
      health,
      preflight,
      voiceContext,
      adapterStatus,
      snapshot,
      visualizationModel,
      exceptionCatalog,
      events,
      pendingDecisions: visiblePendingDecisions,
      demoAuthCode,
      lastError,
      lastSnapshotAt,
      isRefreshing,
      isStale,
      setBaseUrlOverride,
      resetBaseUrlOverride,
      refresh,
      submitInstruction,
      tick,
      submitDecision,
      submitControl,
    }),
    [
      adapterStatus,
      baseUrlOverride,
      connectionStatus,
      demoAuthCode,
      effectiveBaseUrl,
      exceptionCatalog,
      events,
      health,
      isRefreshing,
      isStale,
      lastError,
      lastSnapshotAt,
      preflight,
      refresh,
      resetBaseUrlOverride,
      setBaseUrlOverride,
      snapshot,
      submitDecision,
      submitInstruction,
      submitControl,
      tick,
      visualizationModel,
      visiblePendingDecisions,
      voiceContext,
    ]
  );

  return (
    <MissionSupervisorContext.Provider value={value}>
      {children}
    </MissionSupervisorContext.Provider>
  );
}

function newClientId(prefix: string): string {
  if (typeof crypto !== 'undefined' && 'randomUUID' in crypto) {
    return `${prefix}-${crypto.randomUUID()}`;
  }

  return `${prefix}-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

function parseStreamData<T>(event: Event): T | null {
  const data = (event as MessageEvent<string>).data;

  if (!data) {
    return null;
  }

  try {
    return JSON.parse(data) as T;
  } catch {
    return null;
  }
}

function appendMissionEvent(
  currentEvents: MissionEvent[],
  nextEvent: MissionEvent
): MissionEvent[] {
  if (
    currentEvents.some(
      (event) =>
        event.event_id === nextEvent.event_id ||
        event.sequence === nextEvent.sequence
    )
  ) {
    return currentEvents;
  }

  return [...currentEvents, nextEvent]
    .sort((left, right) => left.sequence - right.sequence)
    .slice(-MAX_EVENTS);
}

export function useMissionSupervisor() {
  const context = useContext(MissionSupervisorContext);

  if (!context) {
    throw new Error(
      'useMissionSupervisor must be used within MissionSupervisorProvider'
    );
  }

  return context;
}
