'use client';

import type React from 'react';
import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  Bot,
  CheckCircle2,
  Download,
  Loader2,
  Mic2,
  Pause,
  Play,
  Radio,
  RotateCcw,
  SkipForward,
  Square,
  User,
  XCircle,
} from 'lucide-react';
import { missionSupervisorService } from '@/services/mission-supervisor';
import type {
  ConversationTurn,
  MissionSupervisorConnectionStatus,
  PlaybackControlAction,
  PlaybackScenarioSummary,
  PlaybackSessionSnapshot,
  PlaybackStep,
  VoiceSessionSummary,
  VoiceTranscriptSnapshot,
} from '@/types/mission-control';

interface PlaybackControlPanelProps {
  baseUrlOverride?: string;
  connectionStatus: MissionSupervisorConnectionStatus;
  profile?: string | null;
  onSupervisorChanged?: () => void;
}

const DEFAULT_SCENARIO = 'tc36_full_fake_happy_path';

const STATUS_CLASSES: Record<string, string> = {
  pending:
    'border-gray-200 bg-gray-50 text-gray-500 dark:border-gray-800 dark:bg-gray-900 dark:text-gray-400',
  running:
    'border-blue-300 bg-blue-50 text-blue-800 dark:border-blue-800 dark:bg-blue-950/40 dark:text-blue-200',
  passed:
    'border-emerald-300 bg-emerald-50 text-emerald-800 dark:border-emerald-800 dark:bg-emerald-950/40 dark:text-emerald-200',
  failed:
    'border-red-300 bg-red-50 text-red-800 dark:border-red-800 dark:bg-red-950/40 dark:text-red-200',
  skipped:
    'border-amber-300 bg-amber-50 text-amber-800 dark:border-amber-800 dark:bg-amber-950/40 dark:text-amber-200',
};

const LANGUAGE_LABELS: Record<string, string> = {
  zh: '🇨🇳 普通话',
  yue: '🇭🇰 粤语',
  en: '🇺🇸 English',
};

export function PlaybackControlPanel({
  baseUrlOverride,
  connectionStatus,
  profile,
  onSupervisorChanged,
}: PlaybackControlPanelProps) {
  const [scenarios, setScenarios] = useState<PlaybackScenarioSummary[]>([]);
  const [scenario, setScenario] = useState(DEFAULT_SCENARIO);
  const [session, setSession] = useState<PlaybackSessionSnapshot | null>(null);
  const [voiceSessions, setVoiceSessions] = useState<VoiceSessionSummary[]>([]);
  const [voiceTranscript, setVoiceTranscript] =
    useState<VoiceTranscriptSnapshot | null>(null);
  const [stepDelay, setStepDelay] = useState(1.2);
  const [busyAction, setBusyAction] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const stepsScrollRef = useRef<HTMLDivElement | null>(null);
  const transcriptScrollRef = useRef<HTMLDivElement | null>(null);

  const connected = connectionStatus === 'connected';
  const safeProfile = !profile || profile === 'fake' || profile === 'test';
  const activeStep = useMemo(() => getActiveStep(session), [session]);
  const latestTurnId =
    session?.transcript[session.transcript.length - 1]?.turn_id ?? null;
  const latestLiveTurnId =
    voiceTranscript?.transcript[voiceTranscript.transcript.length - 1]?.turn_id ??
    null;

  const refreshSession = useCallback(async () => {
    if (!session) {
      return;
    }

    try {
      const next = await missionSupervisorService.getPlaybackSession(
        session.session_id,
        baseUrlOverride
      );
      setSession(next);
    } catch (refreshError) {
      setError(
        refreshError instanceof Error
          ? refreshError.message
          : 'Failed to refresh playback session'
      );
    }
  }, [baseUrlOverride, session]);

  const refreshLiveObserve = useCallback(async () => {
    if (!connected) {
      return;
    }

    try {
      const [sessionsResult, transcriptResult] = await Promise.all([
        missionSupervisorService.getVoiceSessions(baseUrlOverride),
        missionSupervisorService.getLatestVoiceTranscript(baseUrlOverride),
      ]);
      setVoiceSessions(sessionsResult.sessions);
      setVoiceTranscript(transcriptResult);
    } catch {
      // Live Observe is best-effort and should not break script playback controls.
    }
  }, [baseUrlOverride, connected]);

  useEffect(() => {
    let cancelled = false;

    missionSupervisorService
      .getPlaybackScenarios(baseUrlOverride)
      .then((result) => {
        if (cancelled) {
          return;
        }
        setScenarios(result.scenarios);
        if (
          result.scenarios.length &&
          !result.scenarios.some((item) => item.name === scenario)
        ) {
          setScenario(result.scenarios[0].name);
        }
      })
      .catch((loadError) => {
        if (!cancelled) {
          setError(
            loadError instanceof Error
              ? loadError.message
              : 'Failed to load playback scenarios'
          );
        }
      });

    return () => {
      cancelled = true;
    };
  }, [baseUrlOverride, scenario]);

  useEffect(() => {
    if (!session || !['playing', 'step_running'].includes(session.status)) {
      return undefined;
    }

    const timer = window.setInterval(refreshSession, 800);
    return () => window.clearInterval(timer);
  }, [refreshSession, session]);

  useEffect(() => {
    if (!connected) {
      setVoiceSessions([]);
      setVoiceTranscript(null);
      return undefined;
    }

    refreshLiveObserve();
    const timer = window.setInterval(refreshLiveObserve, 1500);
    return () => window.clearInterval(timer);
  }, [connected, refreshLiveObserve]);

  useEffect(() => {
    const container = stepsScrollRef.current;
    if (!container || !activeStep) {
      return;
    }

    const target = container.querySelector<HTMLElement>(
      `[data-step-index="${activeStep.index}"]`
    );
    window.requestAnimationFrame(() => {
      target?.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
    });
  }, [activeStep, session?.current_step_index, session?.session_id, session?.status]);

  useEffect(() => {
    const container = transcriptScrollRef.current;
    if (!container) {
      return;
    }

    window.requestAnimationFrame(() => {
      container.scrollTo({
        top: container.scrollHeight,
        behavior: 'smooth',
      });
    });
  }, [
    latestTurnId,
    latestLiveTurnId,
    session?.session_id,
    session?.transcript.length,
    voiceTranscript?.conversation_id,
    voiceTranscript?.transcript.length,
  ]);

  const loadScenario = async () => {
    if (!connected) {
      setError('Mission Supervisor is not connected');
      return;
    }

    setBusyAction('LOAD');
    setError(null);

    try {
      const next = await missionSupervisorService.createPlaybackSession(
        {
          scenario,
          pace: 'manual',
          step_delay_seconds: stepDelay,
          reset_supervisor: true,
        },
        baseUrlOverride
      );
      setSession(next);
      onSupervisorChanged?.();
    } catch (loadError) {
      setError(
        loadError instanceof Error
          ? loadError.message
          : 'Failed to create playback session'
      );
    } finally {
      setBusyAction(null);
    }
  };

  const control = async (action: PlaybackControlAction) => {
    if (!session) {
      return;
    }

    if (
      action === 'RESET' &&
      !window.confirm('Reset playback and Mission Supervisor state?')
    ) {
      return;
    }

    setBusyAction(action);
    setError(null);

    try {
      const next = await missionSupervisorService.controlPlaybackSession(
        session.session_id,
        {
          action,
          pace: action === 'PLAY' ? 'auto' : undefined,
          step_delay_seconds: action === 'PLAY' ? stepDelay : undefined,
        },
        baseUrlOverride
      );
      setSession(next);
      onSupervisorChanged?.();
    } catch (controlError) {
      setError(
        controlError instanceof Error
          ? controlError.message
          : `Failed to run playback action ${action}`
      );
    } finally {
      setBusyAction(null);
    }
  };

  const exportReport = async () => {
    if (!session) {
      return;
    }

    setBusyAction('EXPORT');
    setError(null);

    try {
      const report = await missionSupervisorService.getPlaybackReport(
        session.session_id,
        baseUrlOverride
      );
      const blob = new Blob([JSON.stringify(report, null, 2)], {
        type: 'application/json',
      });
      const url = URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;
      link.download = `${session.run_id}-playback-report.json`;
      link.click();
      URL.revokeObjectURL(url);
    } catch (exportError) {
      setError(
        exportError instanceof Error
          ? exportError.message
          : 'Failed to export playback report'
      );
    } finally {
      setBusyAction(null);
    }
  };

  return (
    <div className="grid min-h-0 gap-4 p-4 xl:h-[44rem] xl:grid-cols-[minmax(0,1.05fr)_minmax(320px,0.95fr)] xl:items-stretch">
      <div className="flex min-w-0 flex-col gap-4 xl:min-h-0">
        {!safeProfile ? (
          <div className="rounded-md border border-amber-300 bg-amber-50 px-3 py-2 text-xs font-medium text-amber-900 dark:border-amber-900 dark:bg-amber-950/40 dark:text-amber-100">
            Web playback is intended for fake/test profiles. Current profile:
            {' '}
            {profile}
          </div>
        ) : null}

        {error ? (
          <div className="rounded-md border border-red-300 bg-red-50 px-3 py-2 text-xs font-medium text-red-800 dark:border-red-900 dark:bg-red-950/40 dark:text-red-100">
            {error}
          </div>
        ) : null}

        <div className="grid gap-2 sm:grid-cols-[minmax(0,1fr)_110px_auto]">
          <select
            value={scenario}
            onChange={(event) => setScenario(event.target.value)}
            className="h-10 min-w-0 rounded-md border border-gray-300 bg-white px-3 text-sm text-gray-900 outline-none focus:border-primary dark:border-gray-700 dark:bg-gray-900 dark:text-white"
          >
            {scenarios.length ? (
              scenarios.map((item) => (
                <option key={item.name} value={item.name}>
                  {item.name}
                </option>
              ))
            ) : (
              <option value={scenario}>{scenario}</option>
            )}
          </select>
          <input
            type="number"
            min={0.1}
            step={0.1}
            value={stepDelay}
            onChange={(event) => setStepDelay(Number(event.target.value))}
            className="h-10 min-w-0 rounded-md border border-gray-300 bg-white px-3 text-sm text-gray-900 outline-none focus:border-primary dark:border-gray-700 dark:bg-gray-900 dark:text-white"
            aria-label="Step delay seconds"
          />
          <button
            type="button"
            onClick={loadScenario}
            disabled={!connected || Boolean(busyAction)}
            className="inline-flex h-10 items-center justify-center gap-2 rounded-md border border-gray-300 bg-gray-900 px-3 text-sm font-semibold text-white transition-colors hover:bg-black disabled:cursor-not-allowed disabled:opacity-60 dark:border-gray-700 dark:bg-white dark:text-gray-900 dark:hover:bg-gray-200"
          >
            {busyAction === 'LOAD' ? <Loader2 className="h-4 w-4 animate-spin" /> : <RotateCcw className="h-4 w-4" />}
            Load
          </button>
        </div>

        <div className="flex flex-wrap items-center gap-2">
          <PlaybackButton
            label="Play"
            icon={<Play className="h-4 w-4" />}
            busy={busyAction === 'PLAY'}
            disabled={!session || Boolean(busyAction) || session.status === 'completed'}
            onClick={() => control('PLAY')}
          />
          <PlaybackButton
            label="Pause"
            icon={<Pause className="h-4 w-4" />}
            busy={busyAction === 'PAUSE'}
            disabled={!session || Boolean(busyAction)}
            onClick={() => control('PAUSE')}
          />
          <PlaybackButton
            label="Next"
            icon={<SkipForward className="h-4 w-4" />}
            busy={busyAction === 'NEXT'}
            disabled={!session || Boolean(busyAction) || session.status === 'completed'}
            onClick={() => control('NEXT')}
          />
          <PlaybackButton
            label="Reset"
            icon={<RotateCcw className="h-4 w-4" />}
            busy={busyAction === 'RESET'}
            disabled={!session || Boolean(busyAction)}
            onClick={() => control('RESET')}
          />
          <PlaybackButton
            label="Stop"
            icon={<Square className="h-4 w-4" />}
            busy={busyAction === 'STOP'}
            disabled={!session || Boolean(busyAction)}
            onClick={() => control('STOP')}
          />
          <PlaybackButton
            label="Export"
            icon={<Download className="h-4 w-4" />}
            busy={busyAction === 'EXPORT'}
            disabled={!session || Boolean(busyAction)}
            onClick={exportReport}
          />
        </div>

        <div className="grid gap-2 sm:grid-cols-4">
          <Metric label="Status" value={session?.status ?? 'not loaded'} />
          <Metric
            label="Step"
            value={
              session
                ? `${Math.min(session.current_step_index + 1, session.steps_total)} / ${session.steps_total}`
                : 'n/a'
            }
          />
          <Metric label="Run" value={session?.run_id ?? 'n/a'} />
          <Metric label="Active TC" value={activeStep?.test_case ? `TC${activeStep.test_case}` : 'n/a'} />
        </div>

        <div ref={stepsScrollRef} className="max-h-80 min-h-80 overflow-auto rounded-lg border border-gray-200 dark:border-gray-800 xl:min-h-0 xl:flex-1 xl:max-h-none">
          {session?.steps.length ? (
            <div className="divide-y divide-gray-200 dark:divide-gray-800">
              {session.steps.map((step) => (
                <StepRow key={step.index} step={step} active={activeStep?.index === step.index} />
              ))}
            </div>
          ) : (
            <div className="flex min-h-32 items-center justify-center px-4 py-8 text-sm text-gray-500 dark:text-gray-400">
              Load a scenario to inspect playback steps
            </div>
          )}
        </div>
      </div>

      <div className="flex min-w-0 flex-col gap-4 xl:min-h-0">
        <div className="rounded-lg border border-gray-200 bg-gray-50 p-3 dark:border-gray-800 dark:bg-black/15">
          <div className="text-xs font-semibold uppercase text-gray-500 dark:text-gray-400">
            Current Voice Step
          </div>
          {activeStep ? (
            <div className="mt-2 space-y-2">
              <div className="flex flex-wrap items-center gap-2">
                <span className={`inline-flex h-7 items-center rounded-md border px-2 text-xs font-semibold ${STATUS_CLASSES[activeStep.status]}`}>
                  {activeStep.status}
                </span>
                <span className="text-sm font-semibold text-gray-900 dark:text-white">
                  Step {activeStep.index}
                </span>
                {activeStep.test_case ? (
                  <span className="text-sm text-gray-600 dark:text-gray-300">
                    TC{activeStep.test_case}
                  </span>
                ) : null}
              </div>
              <div className="text-sm text-gray-800 dark:text-gray-100">
                {activeStep.user_utterance ?? activeStep.title ?? activeStep.action}
              </div>
              {activeStep.title ? (
                <div className="text-xs text-gray-500 dark:text-gray-400">
                  {activeStep.title}
                </div>
              ) : null}
              {activeStep.error ? (
                <div className="rounded-md border border-red-300 bg-red-50 px-2 py-1 text-xs text-red-800 dark:border-red-900 dark:bg-red-950/40 dark:text-red-100">
                  {activeStep.error}
                </div>
              ) : null}
            </div>
          ) : (
            <div className="mt-2 text-sm text-gray-500 dark:text-gray-400">
              No active playback step
            </div>
          )}
        </div>

        <LiveObservePanel
          sessions={voiceSessions}
          transcript={voiceTranscript}
        />

        <div ref={transcriptScrollRef} className="max-h-[30rem] min-h-80 overflow-auto rounded-lg border border-gray-200 dark:border-gray-800 xl:min-h-0 xl:flex-1 xl:max-h-none">
          <div className="sticky top-0 z-10 border-b border-gray-200 bg-white px-3 py-2 text-xs font-semibold uppercase text-gray-500 dark:border-gray-800 dark:bg-botbot-dark dark:text-gray-400">
            Voice Interaction Context
          </div>
          {session?.transcript.length ? (
            <div className="space-y-3 p-3">
              {session.transcript.map((turn) => (
                <ConversationTurnRow key={turn.turn_id} turn={turn} />
              ))}
            </div>
          ) : (
            <div className="flex min-h-32 items-center justify-center px-4 py-8 text-sm text-gray-500 dark:text-gray-400">
              Conversation context will appear as playback runs
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

function PlaybackButton({
  label,
  icon,
  busy,
  disabled,
  onClick,
}: {
  label: string;
  icon: React.ReactNode;
  busy: boolean;
  disabled: boolean;
  onClick: () => void;
}) {
  return (
    <button
      type="button"
      onClick={onClick}
      disabled={disabled}
      className="inline-flex h-9 items-center justify-center gap-2 rounded-md border border-gray-300 bg-white px-3 text-xs font-semibold text-gray-800 transition-colors hover:bg-gray-100 disabled:cursor-not-allowed disabled:opacity-60 dark:border-gray-700 dark:bg-gray-900 dark:text-gray-100 dark:hover:bg-gray-800"
    >
      {busy ? <Loader2 className="h-4 w-4 animate-spin" /> : icon}
      {label}
    </button>
  );
}

function Metric({ label, value }: { label: string; value: React.ReactNode }) {
  return (
    <div className="min-w-0 rounded-md border border-gray-200 bg-gray-50 px-3 py-2 dark:border-gray-800 dark:bg-black/15">
      <div className="text-xs font-medium text-gray-500 dark:text-gray-400">
        {label}
      </div>
      <div className="mt-1 truncate text-sm font-semibold text-gray-900 dark:text-white">
        {value}
      </div>
    </div>
  );
}

function LiveObservePanel({
  sessions,
  transcript,
}: {
  sessions: VoiceSessionSummary[];
  transcript: VoiceTranscriptSnapshot | null;
}) {
  const activeSession = sessions[0] ?? null;
  const liveTurns = transcript?.transcript.slice(-4) ?? [];

  return (
    <div className="rounded-lg border border-gray-200 bg-gray-50 p-3 dark:border-gray-800 dark:bg-black/15">
      <div className="flex items-center justify-between gap-3">
        <div className="flex items-center gap-2 text-xs font-semibold uppercase text-gray-500 dark:text-gray-400">
          <Radio className="h-4 w-4" />
          Live Observe
        </div>
        <span
          className={`inline-flex h-6 items-center rounded-md border px-2 text-[11px] font-semibold ${
            activeSession
              ? 'border-emerald-300 bg-emerald-50 text-emerald-800 dark:border-emerald-800 dark:bg-emerald-950/40 dark:text-emerald-200'
              : 'border-gray-200 bg-white text-gray-500 dark:border-gray-800 dark:bg-gray-900 dark:text-gray-400'
          }`}
        >
          {activeSession ? 'live' : 'idle'}
        </span>
      </div>

      {activeSession ? (
        <div className="mt-2 grid gap-2 sm:grid-cols-3">
          <Metric label="Language" value={activeSession.language ? formatLanguageLabel(activeSession.language) : 'n/a'} />
          <Metric label="Events" value={activeSession.event_count} />
          <Metric label="Turns" value={activeSession.turn_count} />
        </div>
      ) : null}

      <div className="mt-3 max-h-56 overflow-auto rounded-md border border-gray-200 bg-white dark:border-gray-800 dark:bg-botbot-dark">
        {liveTurns.length ? (
          <div className="space-y-3 p-3">
            {liveTurns.map((turn) => (
              <ConversationTurnRow key={turn.turn_id} turn={turn} />
            ))}
          </div>
        ) : (
          <div className="flex min-h-24 items-center justify-center px-4 py-6 text-sm text-gray-500 dark:text-gray-400">
            Live voice context will appear here
          </div>
        )}
      </div>
    </div>
  );
}

function StepRow({ step, active }: { step: PlaybackStep; active: boolean }) {
  return (
    <div
      data-step-index={step.index}
      className={`grid gap-2 px-3 py-2 sm:grid-cols-[64px_minmax(0,1fr)_80px] ${
        active ? 'bg-blue-50/70 dark:bg-blue-950/20' : 'bg-white dark:bg-botbot-dark'
      }`}
    >
      <div className="flex items-center gap-2">
        {step.status === 'passed' ? (
          <CheckCircle2 className="h-4 w-4 text-emerald-600" />
        ) : step.status === 'failed' ? (
          <XCircle className="h-4 w-4 text-red-600" />
        ) : step.status === 'running' ? (
          <Loader2 className="h-4 w-4 animate-spin text-blue-600" />
        ) : (
          <span className="h-4 w-4 rounded-full border border-gray-300 dark:border-gray-700" />
        )}
        <span className="text-xs font-semibold text-gray-700 dark:text-gray-200">
          {step.index}
        </span>
      </div>
      <div className="min-w-0">
        <div className="truncate text-xs font-semibold text-gray-900 dark:text-white">
          {step.title ?? step.user_utterance ?? step.action}
        </div>
        <div className="truncate text-xs text-gray-500 dark:text-gray-400">
          {step.action}
          {step.current_waypoint ? ` | ${step.current_waypoint}` : ''}
        </div>
      </div>
      <div className="flex items-center justify-start sm:justify-end">
        <span className={`inline-flex h-7 items-center rounded-md border px-2 text-xs font-semibold ${STATUS_CLASSES[step.status]}`}>
          {step.test_case ? `TC${step.test_case}` : step.status}
        </span>
      </div>
    </div>
  );
}

function ConversationTurnRow({ turn }: { turn: ConversationTurn }) {
  const robot = turn.speaker === 'robot';
  const user = turn.speaker === 'staff' || turn.speaker === 'visitor';

  return (
    <div className={`flex gap-2 ${user ? 'flex-row-reverse' : ''}`}>
      <div
        className={`flex h-8 w-8 shrink-0 items-center justify-center rounded-full ${
          robot
            ? 'bg-emerald-100 text-emerald-700 dark:bg-emerald-950/50 dark:text-emerald-200'
            : user
              ? 'bg-blue-100 text-blue-700 dark:bg-blue-950/50 dark:text-blue-200'
              : 'bg-gray-100 text-gray-700 dark:bg-gray-900 dark:text-gray-300'
        }`}
      >
        {robot ? <Bot className="h-4 w-4" /> : user ? <User className="h-4 w-4" /> : <Mic2 className="h-4 w-4" />}
      </div>
      <div
        className={`min-w-0 max-w-[86%] rounded-lg border px-3 py-2 ${
          robot
            ? 'border-emerald-200 bg-emerald-50 dark:border-emerald-900 dark:bg-emerald-950/30'
            : user
              ? 'border-blue-200 bg-blue-50 dark:border-blue-900 dark:bg-blue-950/30'
              : 'border-gray-200 bg-gray-50 dark:border-gray-800 dark:bg-black/15'
        }`}
      >
        <div className="flex flex-wrap items-center gap-2 text-[11px] font-semibold uppercase text-gray-500 dark:text-gray-400">
          <span>{turn.speaker}</span>
          <span>{turn.modality}</span>
          {turn.language ? <span>{formatLanguageLabel(turn.language)}</span> : null}
          {turn.related_test_case ? <span>TC{turn.related_test_case}</span> : null}
          {turn.state_version ? <span>sv {turn.state_version}</span> : null}
        </div>
        <div className="mt-1 whitespace-pre-wrap break-words text-sm text-gray-900 dark:text-white">
          {turn.text}
        </div>
        {turn.intent ? (
          <div className="mt-1 truncate text-xs text-gray-500 dark:text-gray-400">
            {turn.intent}
            {typeof turn.auth_valid === 'boolean' ? ` | auth ${turn.auth_valid ? 'valid' : 'invalid'}` : ''}
          </div>
        ) : null}
      </div>
    </div>
  );
}

function formatLanguageLabel(language: string): string {
  return LANGUAGE_LABELS[language] ?? language.toUpperCase();
}

function getActiveStep(session: PlaybackSessionSnapshot | null): PlaybackStep | null {
  if (!session) {
    return null;
  }
  return (
    session.steps.find((step) => step.status === 'running') ??
    session.steps[session.current_step_index] ??
    session.steps[session.steps.length - 1] ??
    null
  );
}
