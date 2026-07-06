'use client';

import { useEffect, useMemo, useState } from 'react';
import {
  Activity,
  AlertTriangle,
  Check,
  Clock3,
  FileText,
  GitBranch,
  KeyRound,
  Loader2,
  OctagonX,
  Pause,
  Play,
  RefreshCw,
  RotateCcw,
  Send,
  Server,
  Settings,
  ShieldAlert,
  WifiOff,
  X,
} from 'lucide-react';
import { PlaybackControlPanel } from '@/components/mission-control/PlaybackControlPanel';
import { TestCaseMonitorPanel } from '@/components/mission-control/TestCaseMonitorPanel';
import { useMissionSupervisor } from '@/contexts/MissionSupervisorContext';
import type {
  AdapterStatusSnapshot,
  BTStateSnapshot,
  ExceptionCatalog,
  ExceptionSeverity,
  MissionControlAction,
  MissionEvent,
  MissionVoiceContext,
  PreflightSnapshot,
  MissionRisk,
  OperatorDecisionAction,
  PendingOperatorDecision,
} from '@/types/mission-control';

export const dynamic = 'force-dynamic';

function riskClasses(risk: MissionRisk): string {
  switch (risk) {
    case 'LOW':
      return 'border-emerald-300 bg-emerald-50 text-emerald-800 dark:border-emerald-800 dark:bg-emerald-950/40 dark:text-emerald-200';
    case 'MEDIUM':
      return 'border-amber-300 bg-amber-50 text-amber-800 dark:border-amber-800 dark:bg-amber-950/40 dark:text-amber-200';
    case 'HIGH':
      return 'border-orange-300 bg-orange-50 text-orange-800 dark:border-orange-800 dark:bg-orange-950/40 dark:text-orange-200';
    case 'CRITICAL':
      return 'border-red-300 bg-red-50 text-red-800 dark:border-red-800 dark:bg-red-950/40 dark:text-red-200';
    default:
      return 'border-gray-300 bg-gray-50 text-gray-800 dark:border-gray-700 dark:bg-gray-900 dark:text-gray-200';
  }
}

function severityClasses(severity: ExceptionSeverity | undefined): string {
  switch (severity) {
    case 'P0':
      return 'border-red-300 bg-red-50 text-red-800 dark:border-red-800 dark:bg-red-950/40 dark:text-red-200';
    case 'P1':
      return 'border-orange-300 bg-orange-50 text-orange-800 dark:border-orange-800 dark:bg-orange-950/40 dark:text-orange-200';
    case 'P2':
      return 'border-amber-300 bg-amber-50 text-amber-800 dark:border-amber-800 dark:bg-amber-950/40 dark:text-amber-200';
    case 'P3':
    case 'P4':
      return 'border-blue-300 bg-blue-50 text-blue-800 dark:border-blue-800 dark:bg-blue-950/40 dark:text-blue-200';
    default:
      return 'border-gray-300 bg-gray-50 text-gray-800 dark:border-gray-700 dark:bg-gray-900 dark:text-gray-200';
  }
}

function formatTime(value: string | number | null): string {
  if (!value) {
    return 'n/a';
  }

  const date = typeof value === 'number' ? new Date(value) : new Date(value);

  if (Number.isNaN(date.getTime())) {
    return String(value);
  }

  return date.toLocaleTimeString([], {
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
  });
}

function compactJson(value: unknown): string {
  if (value === null || value === undefined) {
    return 'null';
  }

  if (typeof value === 'string') {
    return value;
  }

  return JSON.stringify(value, null, 2);
}

function Panel({
  title,
  description,
  icon,
  children,
  actions,
}: {
  title: string;
  description?: string;
  icon: React.ReactNode;
  children: React.ReactNode;
  actions?: React.ReactNode;
}) {
  return (
    <section className="min-h-0 rounded-lg border border-gray-200 bg-white dark:border-gray-800 dark:bg-botbot-dark">
      <div className="flex min-h-12 items-center justify-between gap-3 border-b border-gray-200 px-4 py-3 dark:border-gray-800">
        <div className="flex min-w-0 items-center gap-2">
          {icon}
          <h2 className="flex min-w-0 flex-wrap items-baseline gap-x-2 gap-y-1 text-sm font-semibold text-gray-900 dark:text-white">
            <span>{title}</span>
            {description ? (
              <span className="text-xs font-normal leading-snug text-gray-500 dark:text-gray-400">
                {description}
              </span>
            ) : null}
          </h2>
        </div>
        {actions}
      </div>
      {children}
    </section>
  );
}

function Badge({ children, className = '' }: { children: React.ReactNode; className?: string }) {
  return (
    <span
      className={`inline-flex h-7 items-center rounded-md border px-2 text-xs font-semibold ${className}`}
    >
      {children}
    </span>
  );
}

function Metric({
  label,
  value,
}: {
  label: string;
  value: React.ReactNode;
}) {
  return (
    <div className="min-w-0 rounded-lg border border-gray-200 bg-gray-50 px-3 py-2 dark:border-gray-800 dark:bg-black/15">
      <div className="text-xs font-medium text-gray-500 dark:text-gray-400">
        {label}
      </div>
      <div className="mt-1 truncate text-sm font-semibold text-gray-900 dark:text-white">
        {value}
      </div>
    </div>
  );
}

function JsonPreview({ value }: { value: unknown }) {
  return (
    <pre className="max-h-48 overflow-auto rounded-md bg-gray-950 p-3 text-xs leading-relaxed text-gray-100">
      {compactJson(value)}
    </pre>
  );
}

function EmptyState({ text }: { text: string }) {
  return (
    <div className="flex min-h-28 items-center justify-center px-4 py-8 text-sm text-gray-500 dark:text-gray-400">
      {text}
    </div>
  );
}

function SystemSafetyBanner({
  snapshot,
  isStale,
}: {
  snapshot: BTStateSnapshot | null;
  isStale: boolean;
}) {
  const activeException = snapshot?.blackboard_summary.active_exception;
  const fault = snapshot?.health.last_fault;
  const isCritical =
    snapshot?.mission_state === 'SAFE_STOP' ||
    snapshot?.mission_state === 'EMERGENCY' ||
    activeException?.severity === 'P0';

  if (!isCritical && !activeException && !fault && !isStale) {
    return null;
  }

  const tone = isCritical
    ? 'border-red-300 bg-red-50 text-red-900 dark:border-red-900 dark:bg-red-950/40 dark:text-red-100'
    : 'border-amber-300 bg-amber-50 text-amber-900 dark:border-amber-900 dark:bg-amber-950/40 dark:text-amber-100';

  return (
    <div className={`flex items-start gap-3 rounded-lg border px-4 py-3 text-sm ${tone}`}>
      <ShieldAlert className="mt-0.5 h-5 w-5 shrink-0" />
      <div className="min-w-0">
        <div className="font-semibold">
          {activeException
            ? `${activeException.severity} ${activeException.title}`
            : isStale
              ? 'Supervisor snapshot is stale'
              : fault?.error_code ?? snapshot?.mission_state ?? 'Mission warning'}
        </div>
        <div className="mt-1 text-xs opacity-90">
          {activeException?.user_message ||
            fault?.message ||
            (isStale ? 'Operator actions are limited until a fresh snapshot is received.' : 'Review Mission Control before continuing.')}
        </div>
      </div>
    </div>
  );
}

function ActiveExceptionPanel({
  snapshot,
  exceptionCatalog,
}: {
  snapshot: BTStateSnapshot | null;
  exceptionCatalog: ExceptionCatalog | null;
}) {
  const activeException = snapshot?.blackboard_summary.active_exception ?? null;
  const descriptor = activeException
    ? exceptionCatalog?.exceptions.find(
        (entry) => entry.exception_id === activeException.exception_id
      )
    : null;

  if (!activeException) {
    const history = snapshot?.blackboard_summary.exception_history ?? [];
    return (
      <div className="space-y-3 p-4">
        <div className="rounded-md border border-emerald-200 bg-emerald-50 p-3 text-sm text-emerald-800 dark:border-emerald-900 dark:bg-emerald-950/30 dark:text-emerald-200">
          No active exception
        </div>
        {history.length ? (
          <div className="space-y-2">
            <div className="text-xs font-semibold uppercase text-gray-500 dark:text-gray-400">
              Recent Resolved Exceptions
            </div>
            {history.slice(-4).reverse().map((item) => (
              <div
                key={item.exception_instance_id}
                className="rounded-md border border-gray-200 bg-gray-50 px-3 py-2 text-xs dark:border-gray-800 dark:bg-black/15"
              >
                <div className="font-semibold text-gray-900 dark:text-white">
                  {item.exception_id} | {item.status}
                </div>
                <div className="mt-1 text-gray-500 dark:text-gray-400">
                  {item.waypoint ?? 'n/a'} | {item.action_id ?? 'n/a'}
                </div>
              </div>
            ))}
          </div>
        ) : null}
      </div>
    );
  }

  return (
    <div className="space-y-3 p-4">
      <div className="flex flex-wrap items-center gap-2">
        <Badge className={severityClasses(activeException.severity)}>
          {activeException.severity}
        </Badge>
        <Badge className={riskClasses(activeException.risk)}>
          {activeException.risk}
        </Badge>
        <Badge className="border-gray-200 bg-gray-50 text-gray-700 dark:border-gray-700 dark:bg-gray-900 dark:text-gray-200">
          {activeException.status}
        </Badge>
      </div>
      <div>
        <div className="text-sm font-semibold text-gray-900 dark:text-white">
          {activeException.title || activeException.exception_id}
        </div>
        <div className="mt-1 text-xs text-gray-500 dark:text-gray-400">
          {activeException.exception_id} | {activeException.exception_instance_id}
        </div>
      </div>
      {activeException.user_message ? (
        <div className="rounded-md border border-amber-200 bg-amber-50 p-3 text-sm text-amber-900 dark:border-amber-900 dark:bg-amber-950/30 dark:text-amber-100">
          {activeException.user_message}
        </div>
      ) : null}
      <div className="grid gap-2 sm:grid-cols-2">
        <Metric label="Waypoint" value={activeException.waypoint ?? 'n/a'} />
        <Metric label="Action" value={activeException.action_id ?? 'n/a'} />
        <Metric label="Error" value={activeException.error_code ?? 'n/a'} />
        <Metric
          label="Attempt"
          value={
            activeException.attempt && activeException.max_attempts
              ? `${activeException.attempt}/${activeException.max_attempts}`
              : 'n/a'
          }
        />
      </div>
      <div className="flex flex-wrap gap-2">
        {(descriptor?.related_test_cases ?? activeException.related_test_cases).map((testCase) => (
          <Badge
            key={testCase}
            className="border-blue-200 bg-blue-50 text-blue-800 dark:border-blue-900 dark:bg-blue-950/30 dark:text-blue-200"
          >
            TC{testCase}
          </Badge>
        ))}
      </div>
    </div>
  );
}

function EvidenceChecklistPanel({
  snapshot,
  exceptionCatalog,
}: {
  snapshot: BTStateSnapshot | null;
  exceptionCatalog: ExceptionCatalog | null;
}) {
  const activeException = snapshot?.blackboard_summary.active_exception ?? null;
  const descriptor = activeException
    ? exceptionCatalog?.exceptions.find(
        (entry) => entry.exception_id === activeException.exception_id
      )
    : null;
  const requiredEvidence =
    descriptor?.evidence ??
    ((activeException?.evidence?.required_evidence as string[] | undefined) || []);

  if (!activeException) {
    return <EmptyState text="No active exception evidence required" />;
  }

  const evidence = activeException.evidence ?? {};
  const fault =
    typeof evidence.fault === 'object' && evidence.fault !== null
      ? (evidence.fault as Record<string, unknown>)
      : {};

  return (
    <div className="space-y-3 p-4">
      {requiredEvidence.length ? (
        <div className="space-y-2">
          {requiredEvidence.map((item) => {
            const present = item in evidence || item in fault;
            return (
              <div
                key={item}
                className="flex items-center justify-between gap-2 rounded-md border border-gray-200 bg-gray-50 px-3 py-2 text-xs dark:border-gray-800 dark:bg-black/15"
              >
                <span className="font-semibold text-gray-900 dark:text-white">
                  {item}
                </span>
                {present ? (
                  <Check className="h-4 w-4 text-emerald-600" />
                ) : (
                  <Clock3 className="h-4 w-4 text-amber-600" />
                )}
              </div>
            );
          })}
        </div>
      ) : (
        <div className="text-sm text-gray-500 dark:text-gray-400">
          This exception has no explicit evidence checklist.
        </div>
      )}
      <JsonPreview value={evidence} />
    </div>
  );
}

function RecoveryTimelinePanel({ snapshot }: { snapshot: BTStateSnapshot | null }) {
  const retry = snapshot?.blackboard_summary.retry_summary ?? null;
  const activeAction = snapshot?.blackboard_summary.active_action ?? null;
  const taskHistory = snapshot?.blackboard_summary.task_history ?? [];

  return (
    <div className="space-y-3 p-4">
      <div className="grid gap-2 sm:grid-cols-2">
        <Metric label="Active Action" value={activeAction?.action_id ?? 'n/a'} />
        <Metric label="Action Status" value={activeAction?.status ?? 'n/a'} />
        <Metric
          label="Retry"
          value={
            retry
              ? `${retry.status ?? 'RUNNING'} ${retry.attempt ?? '-'}/${retry.max_attempts ?? '-'}`
              : 'n/a'
          }
        />
        <Metric label="Recovery" value={retry?.recovery_action ?? 'n/a'} />
      </div>
      {taskHistory.length ? (
        <div className="space-y-2">
          <div className="text-xs font-semibold uppercase text-gray-500 dark:text-gray-400">
            Recent Task Actions
          </div>
          {taskHistory.slice(-8).reverse().map((item, index) => (
            <div
              key={`${item.action_key ?? item.action_id}-${index}`}
              className="rounded-md border border-gray-200 bg-gray-50 px-3 py-2 text-xs dark:border-gray-800 dark:bg-black/15"
            >
              <div className="flex flex-wrap items-center justify-between gap-2">
                <span className="font-semibold text-gray-900 dark:text-white">
                  {item.action_id ?? item.action_key}
                </span>
                <span>{item.status}</span>
              </div>
              <div className="mt-1 text-gray-500 dark:text-gray-400">
                {item.error_code ?? 'ok'} | attempt {item.attempt ?? '-'}
              </div>
            </div>
          ))}
        </div>
      ) : (
        <EmptyState text="No task action history" />
      )}
    </div>
  );
}

function DecisionButton({
  action,
  onClick,
  disabled,
  busy,
  disabledReason,
}: {
  action: OperatorDecisionAction;
  onClick: () => void;
  disabled: boolean;
  busy: boolean;
  disabledReason?: string | null;
}) {
  const styles: Record<OperatorDecisionAction, string> = {
    APPROVE:
      'border-emerald-300 bg-emerald-600 text-white hover:bg-emerald-700 disabled:bg-emerald-300 dark:border-emerald-700',
    DENY:
      'border-gray-300 bg-white text-gray-800 hover:bg-gray-100 disabled:bg-gray-100 dark:border-gray-700 dark:bg-gray-900 dark:text-gray-100 dark:hover:bg-gray-800',
    RETRY:
      'border-blue-300 bg-blue-600 text-white hover:bg-blue-700 disabled:bg-blue-300 dark:border-blue-700',
    SKIP:
      'border-gray-300 bg-white text-gray-800 hover:bg-gray-100 disabled:bg-gray-100 dark:border-gray-700 dark:bg-gray-900 dark:text-gray-100 dark:hover:bg-gray-800',
    PAUSE:
      'border-amber-300 bg-amber-600 text-white hover:bg-amber-700 disabled:bg-amber-300 dark:border-amber-700',
    TAKEOVER:
      'border-orange-300 bg-orange-600 text-white hover:bg-orange-700 disabled:bg-orange-300 dark:border-orange-700',
    ABORT:
      'border-red-300 bg-red-600 text-white hover:bg-red-700 disabled:bg-red-300 dark:border-red-700',
  };

  const icons: Record<OperatorDecisionAction, React.ReactNode> = {
    APPROVE: <Check className="h-4 w-4" />,
    DENY: <X className="h-4 w-4" />,
    RETRY: <RefreshCw className="h-4 w-4" />,
    SKIP: <X className="h-4 w-4" />,
    PAUSE: <Clock3 className="h-4 w-4" />,
    TAKEOVER: <ShieldAlert className="h-4 w-4" />,
    ABORT: <OctagonX className="h-4 w-4" />,
  };

  return (
    <button
      type="button"
      onClick={onClick}
      disabled={disabled || busy}
      title={disabledReason || action}
      className={`inline-flex h-9 items-center justify-center gap-2 rounded-md border px-3 text-xs font-semibold transition-colors disabled:cursor-not-allowed disabled:opacity-70 ${styles[action]}`}
    >
      {busy ? <Loader2 className="h-4 w-4 animate-spin" /> : icons[action]}
      {action}
    </button>
  );
}

function DecisionQueue({
  decisions,
  disabled,
  busyKey,
  onDecision,
  getDisabledReason,
}: {
  decisions: PendingOperatorDecision[];
  disabled: boolean;
  busyKey: string | null;
  onDecision: (
    decision: PendingOperatorDecision,
    action: OperatorDecisionAction
  ) => void;
  getDisabledReason?: (
    decision: PendingOperatorDecision,
    action: OperatorDecisionAction
  ) => string | null;
}) {
  if (decisions.length === 0) {
    return <EmptyState text="No pending decisions" />;
  }

  return (
    <div className="divide-y divide-gray-200 dark:divide-gray-800">
      {decisions.map((decision) => (
        <article key={decision.decision_id} className="px-4 py-4">
          <div className="flex flex-col gap-3 lg:flex-row lg:items-start lg:justify-between">
            <div className="min-w-0">
              <div className="mb-2 flex flex-wrap items-center gap-2">
                <Badge className={riskClasses(decision.risk)}>{decision.risk}</Badge>
                <Badge className="border-gray-200 bg-gray-50 text-gray-700 dark:border-gray-700 dark:bg-gray-900 dark:text-gray-200">
                  {decision.status}
                </Badge>
                <span className="text-xs text-gray-500 dark:text-gray-400">
                  sv {decision.state_version} | {formatTime(decision.created_at)}
                </span>
              </div>
              <h3 className="truncate text-sm font-semibold text-gray-900 dark:text-white">
                {decision.kind}
              </h3>
              <p className="mt-1 truncate text-xs text-gray-500 dark:text-gray-400">
                {decision.node_path}
              </p>
            </div>
            <div className="flex flex-wrap gap-2">
              {decision.allowed_actions
                .map((action) => {
                  const disabledReason = getDisabledReason?.(decision, action);

                  return (
                    <DecisionButton
                      key={action}
                      action={action}
                      disabled={disabled || Boolean(disabledReason)}
                      busy={busyKey === `${decision.decision_id}:${action}`}
                      disabledReason={disabledReason}
                      onClick={() => onDecision(decision, action)}
                    />
                  );
                })}
            </div>
          </div>
          <div className="mt-3 grid gap-3 xl:grid-cols-2">
            <div>
              <div className="mb-1 text-xs font-semibold text-gray-500 dark:text-gray-400">
                Context
              </div>
              <JsonPreview value={decision.context} />
            </div>
            <div>
              <div className="mb-1 text-xs font-semibold text-gray-500 dark:text-gray-400">
                Required Evidence
              </div>
              <JsonPreview value={decision.required_evidence} />
            </div>
          </div>
        </article>
      ))}
    </div>
  );
}

function EventLog({ events }: { events: MissionEvent[] }) {
  const latestEvents = useMemo(() => [...events].slice(-50).reverse(), [events]);

  if (latestEvents.length === 0) {
    return <EmptyState text="No mission events" />;
  }

  return (
    <div className="max-h-[420px] divide-y divide-gray-200 overflow-auto dark:divide-gray-800">
      {latestEvents.map((event) => (
        <details key={event.event_id} className="group px-4 py-3">
          <summary className="flex cursor-pointer list-none items-start justify-between gap-3">
            <div className="min-w-0">
              <div className="truncate text-sm font-semibold text-gray-900 dark:text-white">
                {event.event_type}
              </div>
              <div className="mt-1 text-xs text-gray-500 dark:text-gray-400">
                seq {event.sequence} | sv {event.state_version} |{' '}
                {formatTime(event.timestamp)}
              </div>
            </div>
            <FileText className="mt-0.5 h-4 w-4 shrink-0 text-gray-400 transition-transform group-open:rotate-90" />
          </summary>
          <div className="mt-3">
            <JsonPreview value={event.payload} />
          </div>
        </details>
      ))}
    </div>
  );
}

const CONTROL_ACTIONS: MissionControlAction[] = [
  'PAUSE',
  'RESUME',
  'ABORT',
  'TAKEOVER',
  'RESET_IDLE',
  'RESYNC',
];

const CONTROL_ICONS: Record<MissionControlAction, React.ReactNode> = {
  PAUSE: <Pause className="h-4 w-4" />,
  RESUME: <Play className="h-4 w-4" />,
  ABORT: <OctagonX className="h-4 w-4" />,
  TAKEOVER: <ShieldAlert className="h-4 w-4" />,
  RESET_IDLE: <RotateCcw className="h-4 w-4" />,
  RESYNC: <RefreshCw className="h-4 w-4" />,
};

function PreflightGatePanel({
  preflight,
  onCopy,
}: {
  preflight: PreflightSnapshot | null;
  onCopy: () => void;
}) {
  if (!preflight) {
    return <EmptyState text="Preflight has not been loaded" />;
  }

  return (
    <div className="space-y-3 p-4">
      <div className="flex flex-wrap items-center justify-between gap-2">
        <Badge
          className={
            preflight.go
              ? 'border-emerald-300 bg-emerald-50 text-emerald-800 dark:border-emerald-800 dark:bg-emerald-950/40 dark:text-emerald-200'
              : 'border-red-300 bg-red-50 text-red-800 dark:border-red-800 dark:bg-red-950/40 dark:text-red-200'
          }
        >
          {preflight.go ? 'GO' : 'NO-GO'}
        </Badge>
        <button
          type="button"
          onClick={onCopy}
          className="inline-flex h-8 items-center justify-center gap-2 rounded-md border border-gray-300 bg-white px-2 text-xs font-semibold text-gray-800 transition-colors hover:bg-gray-100 dark:border-gray-700 dark:bg-gray-900 dark:text-gray-100 dark:hover:bg-gray-800"
        >
          <FileText className="h-4 w-4" />
          Copy
        </button>
      </div>
      {preflight.blockers.length ? (
        <div className="rounded-md border border-red-200 bg-red-50 p-3 text-xs text-red-800 dark:border-red-900 dark:bg-red-950/30 dark:text-red-200">
          {preflight.blockers.join(' | ')}
        </div>
      ) : null}
      <div className="max-h-60 divide-y divide-gray-200 overflow-auto rounded-md border border-gray-200 dark:divide-gray-800 dark:border-gray-800">
        {preflight.checks.map((check) => (
          <div key={check.name} className="flex items-start gap-2 px-3 py-2">
            {check.ok ? (
              <Check className="mt-0.5 h-4 w-4 shrink-0 text-emerald-600" />
            ) : (
              <X className="mt-0.5 h-4 w-4 shrink-0 text-red-600" />
            )}
            <div className="min-w-0">
              <div className="truncate text-xs font-semibold text-gray-900 dark:text-white">
                {check.name}
              </div>
              <div className="mt-0.5 text-xs text-gray-500 dark:text-gray-400">
                {check.message || 'n/a'}
              </div>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}

function AdapterStatusPanel({
  adapterStatus,
}: {
  adapterStatus: AdapterStatusSnapshot | null;
}) {
  if (!adapterStatus) {
    return <EmptyState text="Adapter status has not been loaded" />;
  }

  return (
    <div className="space-y-3 p-4">
      <div className="grid gap-2 sm:grid-cols-2">
        <Metric label="Adapter" value={adapterStatus.adapter} />
        <Metric
          label="Available"
          value={adapterStatus.available ? 'true' : 'false'}
        />
        <Metric label="Profile" value={adapterStatus.profile} />
        <Metric
          label="Running"
          value={adapterStatus.running_commands.length}
        />
      </div>
      {adapterStatus.last_error ? (
        <JsonPreview value={adapterStatus.last_error} />
      ) : null}
    </div>
  );
}

function VoiceContextPanel({
  voiceContext,
}: {
  voiceContext: MissionVoiceContext | null;
}) {
  if (!voiceContext) {
    return <EmptyState text="Voice context has not been loaded" />;
  }

  return (
    <div className="space-y-3 p-4">
      <div className="grid gap-2 sm:grid-cols-2">
        <Metric label="Node" value={voiceContext.node_id ?? 'n/a'} />
        <Metric label="State Version" value={voiceContext.state_version} />
      </div>
      <div>
        <div className="mb-2 text-xs font-semibold text-gray-500 dark:text-gray-400">
          Allowed Intents
        </div>
        <div className="flex flex-wrap gap-2">
          {voiceContext.allowed_intents.map((intent) => (
            <Badge
              key={intent}
              className="border-emerald-200 bg-emerald-50 text-emerald-800 dark:border-emerald-900 dark:bg-emerald-950/30 dark:text-emerald-200"
            >
              {intent}
            </Badge>
          ))}
        </div>
      </div>
      {voiceContext.forbidden_intents.length ? (
        <div>
          <div className="mb-2 text-xs font-semibold text-gray-500 dark:text-gray-400">
            Forbidden Intents
          </div>
          <div className="flex flex-wrap gap-2">
            {voiceContext.forbidden_intents.map((intent) => (
              <Badge
                key={intent}
                className="border-red-200 bg-red-50 text-red-800 dark:border-red-900 dark:bg-red-950/30 dark:text-red-200"
              >
                {intent}
              </Badge>
            ))}
          </div>
        </div>
      ) : null}
      <JsonPreview value={voiceContext.risk_by_intent} />
    </div>
  );
}

function DemoAuthInstructionPanel({
  code,
  instructionText,
  busy,
  disabled,
  onInstructionTextChange,
  onUseCode,
  onSubmitInstruction,
  onTick,
}: {
  code: ReturnType<typeof useMissionSupervisor>['demoAuthCode'];
  instructionText: string;
  busy: boolean;
  disabled: boolean;
  onInstructionTextChange: (value: string) => void;
  onUseCode: () => void;
  onSubmitInstruction: () => void;
  onTick: () => void;
}) {
  const visibleCode = code?.enabled && code.demo_visible && code.code;

  return (
    <div className="grid gap-3 p-4 lg:grid-cols-[minmax(220px,0.7fr)_minmax(0,1.5fr)]">
      <div className="grid gap-2 sm:grid-cols-3 lg:grid-cols-1">
        <div className="rounded-lg border border-gray-200 bg-gray-50 px-3 py-2 dark:border-gray-800 dark:bg-black/15">
          <div className="text-xs font-medium text-gray-500 dark:text-gray-400">
            Demo Auth Code
          </div>
          <button
            type="button"
            onClick={onUseCode}
            disabled={!visibleCode}
            className="mt-1 inline-flex h-10 min-w-32 items-center justify-center rounded-md border border-gray-300 bg-white px-3 font-mono text-lg font-bold text-gray-950 transition-colors hover:bg-gray-100 disabled:cursor-not-allowed disabled:bg-gray-100 disabled:text-gray-500 dark:border-gray-700 dark:bg-gray-900 dark:text-white dark:hover:bg-gray-800"
          >
            {visibleCode ? code.code : code?.enabled ? 'hidden' : 'disabled'}
          </button>
        </div>
        <Metric
          label="TTL"
          value={
            code?.seconds_remaining === null ||
            code?.seconds_remaining === undefined
              ? 'n/a'
              : `${code.seconds_remaining}s`
          }
        />
        <Metric
          label="Source"
          value={
            code
              ? `${code.source} / ${code.period_seconds}s`
              : 'n/a'
          }
        />
      </div>

      <div className="space-y-3">
        <div className="grid gap-2 sm:grid-cols-[minmax(0,1fr)_auto_auto]">
          <input
            value={instructionText}
            onChange={(event) => onInstructionTextChange(event.target.value)}
            onKeyDown={(event) => {
              if (event.key === 'Enter') {
                event.preventDefault();
                onSubmitInstruction();
              }
            }}
            placeholder="当前验证码 开始导览"
            className="h-10 min-w-0 rounded-md border border-gray-300 bg-white px-3 text-sm text-gray-900 outline-none transition-colors focus:border-primary dark:border-gray-700 dark:bg-gray-900 dark:text-white"
          />
          <button
            type="button"
            onClick={onSubmitInstruction}
            disabled={disabled || busy || !instructionText.trim()}
            className="inline-flex h-10 items-center justify-center gap-2 rounded-md border border-gray-300 bg-gray-900 px-3 text-sm font-semibold text-white transition-colors hover:bg-black disabled:cursor-not-allowed disabled:opacity-60 dark:border-gray-700 dark:bg-white dark:text-gray-900 dark:hover:bg-gray-200"
          >
            {busy ? <Loader2 className="h-4 w-4 animate-spin" /> : <Send className="h-4 w-4" />}
            Send
          </button>
          <button
            type="button"
            onClick={onTick}
            disabled={disabled || busy}
            className="inline-flex h-10 items-center justify-center gap-2 rounded-md border border-gray-300 bg-white px-3 text-sm font-semibold text-gray-800 transition-colors hover:bg-gray-100 disabled:cursor-not-allowed disabled:opacity-60 dark:border-gray-700 dark:bg-gray-900 dark:text-gray-100 dark:hover:bg-gray-800"
          >
            <RefreshCw className={`h-4 w-4 ${busy ? 'animate-spin' : ''}`} />
            Tick
          </button>
        </div>
      </div>
    </div>
  );
}

function MissionControlActions({
  disabledReasonFor,
  busyKey,
  onControl,
}: {
  disabledReasonFor: (action: MissionControlAction) => string | null;
  busyKey: string | null;
  onControl: (action: MissionControlAction) => void;
}) {
  return (
    <div className="grid grid-cols-2 gap-2 p-4 sm:grid-cols-3">
      {CONTROL_ACTIONS.map((action) => {
        const disabledReason = disabledReasonFor(action);
        const busy = busyKey === `control:${action}`;
        const critical = action === 'ABORT' || action === 'TAKEOVER';

        return (
          <button
            key={action}
            type="button"
            onClick={() => onControl(action)}
            disabled={Boolean(disabledReason) || busy}
            title={disabledReason || action}
            className={`inline-flex h-10 items-center justify-center gap-2 rounded-md border px-3 text-xs font-semibold transition-colors disabled:cursor-not-allowed disabled:opacity-70 ${
              critical
                ? 'border-red-300 bg-red-600 text-white hover:bg-red-700 dark:border-red-800'
                : 'border-gray-300 bg-white text-gray-800 hover:bg-gray-100 dark:border-gray-700 dark:bg-gray-900 dark:text-gray-100 dark:hover:bg-gray-800'
            }`}
          >
            {busy ? <Loader2 className="h-4 w-4 animate-spin" /> : CONTROL_ICONS[action]}
            {action}
          </button>
        );
      })}
    </div>
  );
}

export default function MissionControlPage() {
  const {
    baseUrlOverride,
    defaultBaseUrl,
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
    pendingDecisions,
    demoAuthCode,
    lastError,
    lastSnapshotAt,
    isRefreshing,
    isStale,
    setBaseUrlOverride,
    resetBaseUrlOverride,
    refresh,
    submitInstruction,
    tick: tickSupervisor,
    submitDecision,
    submitControl,
  } = useMissionSupervisor();
  const [baseUrlDraft, setBaseUrlDraft] = useState(baseUrlOverride);
  const [operatorId, setOperatorId] = useState('botbrain-operator');
  const [comment, setComment] = useState('');
  const [instructionText, setInstructionText] = useState('');
  const [busyKey, setBusyKey] = useState<string | null>(null);
  const [decisionError, setDecisionError] = useState<string | null>(null);

  useEffect(() => {
    document.title = 'Mission Control - BotBrain';
  }, []);

  useEffect(() => {
    setBaseUrlDraft(baseUrlOverride);
  }, [baseUrlOverride]);

  const connectionBadge = useMemo(() => {
    if (connectionStatus === 'connected' && !isStale) {
      return (
        <Badge className="border-emerald-300 bg-emerald-50 text-emerald-800 dark:border-emerald-800 dark:bg-emerald-950/40 dark:text-emerald-200">
          Connected
        </Badge>
      );
    }

    if (connectionStatus === 'connecting') {
      return (
        <Badge className="border-blue-300 bg-blue-50 text-blue-800 dark:border-blue-800 dark:bg-blue-950/40 dark:text-blue-200">
          Connecting
        </Badge>
      );
    }

    if (connectionStatus === 'error') {
      return (
        <Badge className="border-red-300 bg-red-50 text-red-800 dark:border-red-800 dark:bg-red-950/40 dark:text-red-200">
          Error
        </Badge>
      );
    }

    return (
      <Badge className="border-amber-300 bg-amber-50 text-amber-800 dark:border-amber-800 dark:bg-amber-950/40 dark:text-amber-200">
        Stale
      </Badge>
    );
  }, [connectionStatus, isStale]);

  const handleDecision = async (
    decision: PendingOperatorDecision,
    action: OperatorDecisionAction
  ) => {
    const disabledReason = getDecisionDisabledReason(decision, action);
    if (disabledReason) {
      setDecisionError(disabledReason);
      return;
    }

    if (
      (decision.risk === 'HIGH' || decision.risk === 'CRITICAL') &&
      action === 'APPROVE' &&
      !window.confirm(`${action} ${decision.risk} decision ${decision.kind}?`)
    ) {
      return;
    }

    if (
      ['ABORT', 'TAKEOVER'].includes(action) &&
      !window.confirm(`${action} mission from decision ${decision.kind}?`)
    ) {
      return;
    }

    const key = `${decision.decision_id}:${action}`;
    setBusyKey(key);
    setDecisionError(null);

    try {
      await submitDecision(decision, action, operatorId, comment);
      setComment('');
    } catch (error) {
      const message =
        error instanceof Error ? error.message : 'Decision submission failed';
      setDecisionError(message);
      if (message.includes('state_version')) {
        await refresh();
      }
    } finally {
      setBusyKey(null);
    }
  };

  const handleControl = async (action: MissionControlAction) => {
    const disabledReason = getControlDisabledReason(action);
    if (disabledReason) {
      setDecisionError(disabledReason);
      return;
    }

    if (
      ['ABORT', 'TAKEOVER', 'RESET_IDLE'].includes(action) &&
      !window.confirm(`${action} mission as ${operatorId || 'botbrain-operator'}?`)
    ) {
      return;
    }

    const key = `control:${action}`;
    setBusyKey(key);
    setDecisionError(null);

    try {
      await submitControl(action, operatorId, comment);
      if (action !== 'RESYNC') {
        setComment('');
      }
    } catch (error) {
      const message =
        error instanceof Error ? error.message : 'Mission control command failed';
      setDecisionError(message);
      if (message.includes('state_version')) {
        await refresh();
      }
    } finally {
      setBusyKey(null);
    }
  };

  const fillInstructionWithAuthCode = () => {
    if (!demoAuthCode?.demo_visible || !demoAuthCode.code) {
      return;
    }

    const command =
      instructionText.replace(/^\d{8}\s*/, '').trim() || '开始导览';
    setInstructionText(`${demoAuthCode.code} ${command}`);
  };

  const handleSubmitInstruction = async () => {
    const text = instructionText.trim();
    if (!text) {
      return;
    }

    setBusyKey('instruction:submit');
    setDecisionError(null);

    try {
      await submitInstruction(text);
      setInstructionText('');
    } catch (error) {
      setDecisionError(
        error instanceof Error ? error.message : 'Instruction submission failed'
      );
    } finally {
      setBusyKey(null);
    }
  };

  const handleTick = async () => {
    setBusyKey('instruction:tick');
    setDecisionError(null);

    try {
      await tickSupervisor();
    } catch (error) {
      setDecisionError(error instanceof Error ? error.message : 'Tick failed');
    } finally {
      setBusyKey(null);
    }
  };

  const getDecisionDisabledReason = (
    decision: PendingOperatorDecision,
    action: OperatorDecisionAction
  ): string | null => {
    if (connectionStatus !== 'connected') {
      return 'Supervisor is not connected';
    }
    if (isStale) {
      return 'Snapshot is stale; use RESYNC before decision actions';
    }
    if (
      action === 'APPROVE' &&
      decision.kind === 'START_MISSION' &&
      preflight &&
      !preflight.go
    ) {
      return 'Preflight is NO-GO; START_MISSION cannot be approved';
    }
    if (
      decision.risk === 'CRITICAL' &&
      ['APPROVE', 'ABORT', 'TAKEOVER'].includes(action) &&
      !comment.trim()
    ) {
      return 'CRITICAL actions require an operator comment';
    }
    return null;
  };

  const getControlDisabledReason = (action: MissionControlAction): string | null => {
    if (connectionStatus !== 'connected') {
      return 'Supervisor is not connected';
    }
    if (isStale && !['ABORT', 'TAKEOVER', 'RESYNC'].includes(action)) {
      return 'Snapshot is stale; only ABORT, TAKEOVER, and RESYNC are available';
    }

    const state = snapshot?.mission_state;
    if (action === 'PAUSE' && !['ACTIVE', 'WAITING_OPERATOR', 'RETURNING'].includes(state ?? '')) {
      return 'PAUSE is only available while a mission is active';
    }
    if (action === 'RESUME' && state !== 'PAUSED') {
      return 'RESUME is only available from PAUSED';
    }
    if (action === 'RESET_IDLE' && !['SAFE_STOP', 'COMPLETED', 'IDLE'].includes(state ?? '')) {
      return 'RESET_IDLE is only available from SAFE_STOP, COMPLETED, or IDLE';
    }
    return null;
  };

  const copyPreflight = async () => {
    if (!preflight) {
      return;
    }
    await navigator.clipboard.writeText(JSON.stringify(preflight, null, 2));
  };

  return (
    <div className="h-full overflow-auto px-3 py-3 lg:px-5 lg:py-4">
      <div className="mx-auto flex max-w-[1800px] flex-col gap-4">
        <div className="flex flex-col gap-3 rounded-lg border border-gray-200 bg-white px-4 py-4 dark:border-gray-800 dark:bg-botbot-dark lg:flex-row lg:items-center lg:justify-between">
          <div className="min-w-0">
            <div className="flex flex-wrap items-center gap-2">
              <h1 className="text-xl font-bold text-gray-950 dark:text-white">
                Mission Control
              </h1>
              {connectionBadge}
              {pendingDecisions.length ? (
                <Badge className="border-amber-300 bg-amber-50 text-amber-800 dark:border-amber-800 dark:bg-amber-950/40 dark:text-amber-200">
                  {pendingDecisions.length} pending
                </Badge>
              ) : null}
              {preflight ? (
                <Badge
                  className={
                    preflight.go
                      ? 'border-emerald-300 bg-emerald-50 text-emerald-800 dark:border-emerald-800 dark:bg-emerald-950/40 dark:text-emerald-200'
                      : 'border-red-300 bg-red-50 text-red-800 dark:border-red-800 dark:bg-red-950/40 dark:text-red-200'
                  }
                >
                  Preflight {preflight.go ? 'GO' : 'NO-GO'}
                </Badge>
              ) : null}
            </div>
            <div className="mt-1 truncate text-sm text-gray-500 dark:text-gray-400">
              {effectiveBaseUrl}
            </div>
          </div>
          <button
            type="button"
            onClick={refresh}
            disabled={isRefreshing}
            className="inline-flex h-10 items-center justify-center gap-2 rounded-md border border-gray-300 bg-white px-3 text-sm font-semibold text-gray-800 transition-colors hover:bg-gray-100 disabled:cursor-wait disabled:opacity-70 dark:border-gray-700 dark:bg-gray-900 dark:text-gray-100 dark:hover:bg-gray-800"
          >
            <RefreshCw
              className={`h-4 w-4 ${isRefreshing ? 'animate-spin' : ''}`}
            />
            Refresh
          </button>
        </div>

        {lastError ? (
          <div className="flex items-start gap-2 rounded-lg border border-red-300 bg-red-50 px-4 py-3 text-sm text-red-800 dark:border-red-800 dark:bg-red-950/40 dark:text-red-200">
            <WifiOff className="mt-0.5 h-4 w-4 shrink-0" />
            <span>{lastError}</span>
          </div>
        ) : null}

        {decisionError ? (
          <div className="flex items-start gap-2 rounded-lg border border-red-300 bg-red-50 px-4 py-3 text-sm text-red-800 dark:border-red-800 dark:bg-red-950/40 dark:text-red-200">
            <AlertTriangle className="mt-0.5 h-4 w-4 shrink-0" />
            <span>{decisionError}</span>
          </div>
        ) : null}

        <SystemSafetyBanner snapshot={snapshot} isStale={isStale} />

        <div className="grid gap-4 xl:grid-cols-[minmax(0,2fr)_minmax(360px,1fr)]">
          <div className="flex min-w-0 flex-col gap-4">
            <Panel
              title="Demo Auth + Instruction"
              description="动态验证码与语音指令输入，用于模拟工作人员下发任务。"
              icon={<KeyRound className="h-4 w-4 text-blue-600" />}
            >
              <DemoAuthInstructionPanel
                code={demoAuthCode}
                instructionText={instructionText}
                busy={busyKey === 'instruction:submit' || busyKey === 'instruction:tick'}
                disabled={connectionStatus !== 'connected'}
                onInstructionTextChange={setInstructionText}
                onUseCode={fillInstructionWithAuthCode}
                onSubmitInstruction={handleSubmitInstruction}
                onTick={handleTick}
              />
            </Panel>

            <Panel
              title="Supervisor Snapshot"
              description="当前任务状态总览，用于快速查看阶段、位置和版本。"
              icon={<Activity className="h-4 w-4 text-emerald-600" />}
            >
              <div className="grid gap-3 p-4 sm:grid-cols-2 lg:grid-cols-4">
                <Metric
                  label="Mission State"
                  value={snapshot?.mission_state ?? 'n/a'}
                />
                <Metric label="Phase" value={snapshot?.phase ?? 'n/a'} />
                <Metric
                  label="Waypoint"
                  value={snapshot?.current_waypoint ?? 'n/a'}
                />
                <Metric
                  label="Floor"
                  value={
                    snapshot
                      ? `${snapshot.current_floor}${
                          snapshot.target_floor
                            ? ` -> ${snapshot.target_floor}`
                            : ''
                        }`
                      : 'n/a'
                  }
                />
                <Metric
                  label="State Version"
                  value={snapshot?.state_version ?? 'n/a'}
                />
                <Metric label="Sequence" value={snapshot?.sequence ?? 'n/a'} />
                <Metric
                  label="Robot"
                  value={health?.robot_id ?? snapshot?.robot_id ?? 'n/a'}
                />
                <Metric
                  label="Last Snapshot"
                  value={formatTime(lastSnapshotAt)}
                />
              </div>
            </Panel>

            <Panel
              title="Active Exception"
              description="当前异常与处置状态，用于定位风险和恢复动作。"
              icon={<ShieldAlert className="h-4 w-4 text-red-600" />}
            >
              <ActiveExceptionPanel
                snapshot={snapshot}
                exceptionCatalog={exceptionCatalog}
              />
            </Panel>

            <TestCaseMonitorPanel
              visualizationModel={visualizationModel}
              snapshot={snapshot}
              events={events}
              pendingDecisions={pendingDecisions}
            />

            <Panel
              title="Web Playback"
              description="网页测试回放，用于播放、暂停和查看 36 项用例上下文。"
              icon={<Play className="h-4 w-4 text-blue-600" />}
            >
              <PlaybackControlPanel
                baseUrlOverride={baseUrlOverride || undefined}
                connectionStatus={connectionStatus}
                profile={health?.profile}
                onSupervisorChanged={refresh}
              />
            </Panel>

            <Panel
              title="Preflight Gate"
              description="放行前检查，用于确认任务启动条件是否满足。"
              icon={<Check className="h-4 w-4 text-emerald-600" />}
            >
              <PreflightGatePanel preflight={preflight} onCopy={copyPreflight} />
            </Panel>

            <Panel
              title="Pending Decisions"
              description="待处理人工决策，用于批准、拒绝或接管高风险动作。"
              icon={<ShieldAlert className="h-4 w-4 text-orange-600" />}
            >
              {isStale ? (
                <div className="border-b border-amber-200 bg-amber-50 px-4 py-3 text-sm text-amber-800 dark:border-amber-900 dark:bg-amber-950/30 dark:text-amber-200">
                  Snapshot is stale; operator actions are disabled.
                </div>
              ) : null}
              <DecisionQueue
                decisions={pendingDecisions}
                disabled={isStale || connectionStatus !== 'connected'}
                busyKey={busyKey}
                onDecision={handleDecision}
                getDisabledReason={getDecisionDisabledReason}
              />
            </Panel>

            <Panel
              title="Recovery Timeline"
              description="恢复过程时间线，用于跟踪异常重试和处理进展。"
              icon={<RefreshCw className="h-4 w-4 text-blue-600" />}
            >
              <RecoveryTimelinePanel snapshot={snapshot} />
            </Panel>

            <Panel
              title="Active Path"
              description="行为树活动路径，用于查看当前执行节点链路。"
              icon={<GitBranch className="h-4 w-4 text-blue-600" />}
            >
              {snapshot?.active_path.length ? (
                <div className="flex flex-wrap gap-2 p-4">
                  {snapshot.active_path.map((node) => (
                    <span
                      key={node}
                      className="rounded-md border border-blue-200 bg-blue-50 px-2 py-1 text-xs font-medium text-blue-800 dark:border-blue-900 dark:bg-blue-950/30 dark:text-blue-200"
                    >
                      {node}
                    </span>
                  ))}
                </div>
              ) : (
                <EmptyState text="No active path" />
              )}
            </Panel>
          </div>

          <div className="flex min-w-0 flex-col gap-4">
            <Panel
              title="Connection"
              description="后端连接配置，用于切换 Mission Supervisor 地址和配置状态。"
              icon={<Settings className="h-4 w-4 text-gray-600 dark:text-gray-300" />}
            >
              <div className="space-y-3 p-4">
                <label className="block text-xs font-semibold text-gray-500 dark:text-gray-400">
                  Supervisor URL
                </label>
                <input
                  value={baseUrlDraft}
                  onChange={(event) => setBaseUrlDraft(event.target.value)}
                  placeholder={defaultBaseUrl}
                  className="h-10 w-full rounded-md border border-gray-300 bg-white px-3 text-sm text-gray-900 outline-none transition-colors focus:border-primary dark:border-gray-700 dark:bg-gray-900 dark:text-white"
                />
                <div className="grid grid-cols-2 gap-2">
                  <button
                    type="button"
                    onClick={() => setBaseUrlOverride(baseUrlDraft)}
                    className="inline-flex h-9 items-center justify-center rounded-md border border-gray-300 bg-gray-900 px-3 text-xs font-semibold text-white hover:bg-black dark:border-gray-700 dark:bg-white dark:text-gray-900 dark:hover:bg-gray-200"
                  >
                    Save
                  </button>
                  <button
                    type="button"
                    onClick={resetBaseUrlOverride}
                    className="inline-flex h-9 items-center justify-center rounded-md border border-gray-300 bg-white px-3 text-xs font-semibold text-gray-800 hover:bg-gray-100 dark:border-gray-700 dark:bg-gray-900 dark:text-gray-100 dark:hover:bg-gray-800"
                  >
                    Reset
                  </button>
                </div>
                <div className="grid gap-2 pt-2">
                  <Metric label="Profile" value={health?.profile ?? 'n/a'} />
                  <Metric
                    label="Auto Approve"
                    value={String(snapshot?.health?.auto_approve ?? 'n/a')}
                  />
                </div>
              </div>
            </Panel>

            <Panel
              title="Operator"
              description="操作员信息，用于提交人工决策时附带身份和备注。"
              icon={<Server className="h-4 w-4 text-gray-600 dark:text-gray-300" />}
            >
              <div className="space-y-3 p-4">
                <label className="block text-xs font-semibold text-gray-500 dark:text-gray-400">
                  Operator ID
                </label>
                <input
                  value={operatorId}
                  onChange={(event) => setOperatorId(event.target.value)}
                  className="h-10 w-full rounded-md border border-gray-300 bg-white px-3 text-sm text-gray-900 outline-none transition-colors focus:border-primary dark:border-gray-700 dark:bg-gray-900 dark:text-white"
                />
                <label className="block text-xs font-semibold text-gray-500 dark:text-gray-400">
                  Comment
                </label>
                <textarea
                  value={comment}
                  onChange={(event) => setComment(event.target.value)}
                  rows={3}
                  className="w-full resize-none rounded-md border border-gray-300 bg-white px-3 py-2 text-sm text-gray-900 outline-none transition-colors focus:border-primary dark:border-gray-700 dark:bg-gray-900 dark:text-white"
                />
              </div>
            </Panel>

            <Panel
              title="Mission Control"
              description="任务控制按钮，用于暂停、恢复、接管、复位或中止任务。"
              icon={<ShieldAlert className="h-4 w-4 text-red-600" />}
            >
              {isStale ? (
                <div className="border-b border-amber-200 bg-amber-50 px-4 py-3 text-sm text-amber-800 dark:border-amber-900 dark:bg-amber-950/30 dark:text-amber-200">
                  Stale snapshot: only ABORT, TAKEOVER, and RESYNC remain enabled.
                </div>
              ) : null}
              <MissionControlActions
                disabledReasonFor={getControlDisabledReason}
                busyKey={busyKey}
                onControl={handleControl}
              />
            </Panel>

            <Panel
              title="Adapter Status"
              description="适配器状态，用于查看导航、任务和语音通道健康。"
              icon={<Server className="h-4 w-4 text-gray-600 dark:text-gray-300" />}
            >
              <AdapterStatusPanel adapterStatus={adapterStatus} />
            </Panel>

            <Panel
              title="Voice Context"
              description="语音上下文约束，用于查看允许意图、鉴权和风险。"
              icon={<Activity className="h-4 w-4 text-blue-600" />}
            >
              <VoiceContextPanel voiceContext={voiceContext} />
            </Panel>

            <Panel
              title="Evidence Checklist"
              description="证据清单，用于核对异常处置所需信息。"
              icon={<FileText className="h-4 w-4 text-amber-600" />}
            >
              <EvidenceChecklistPanel
                snapshot={snapshot}
                exceptionCatalog={exceptionCatalog}
              />
            </Panel>

            <Panel
              title="Blackboard"
              description="黑板数据，用于调试任务运行时共享状态。"
              icon={<FileText className="h-4 w-4 text-gray-600 dark:text-gray-300" />}
            >
              <div className="p-4">
                <JsonPreview value={snapshot?.blackboard_summary ?? {}} />
              </div>
            </Panel>

            <Panel
              title="Health"
              description="健康详情，用于查看 Supervisor 和运行适配器健康数据。"
              icon={<Server className="h-4 w-4 text-gray-600 dark:text-gray-300" />}
            >
              <div className="p-4">
                <JsonPreview value={snapshot?.health ?? health ?? {}} />
              </div>
            </Panel>
          </div>
        </div>

        <Panel
          title="Mission Events"
          description="事件日志，用于回放状态变化、指令和审计记录。"
          icon={<Clock3 className="h-4 w-4 text-gray-600 dark:text-gray-300" />}
        >
          <EventLog events={events} />
        </Panel>
      </div>
    </div>
  );
}
