'use client';

import { Activity, AlertTriangle, CheckCircle2, Circle, Clock3, Loader2, RefreshCw } from 'lucide-react';
import type {
  BTStateSnapshot,
  MissionEvent,
  PendingOperatorDecision,
  TestCaseCatalogEntry,
  VisualizationModel,
} from '@/types/mission-control';

type CaseStatus = 'COMPLETED' | 'WAITING' | 'FAULTED' | 'RETRYING' | 'EXHAUSTED' | 'RUNNING' | 'PENDING';

interface CaseViewModel {
  entry: TestCaseCatalogEntry;
  status: CaseStatus;
}

interface TestCaseMonitorPanelProps {
  visualizationModel: VisualizationModel | null;
  snapshot: BTStateSnapshot | null;
  events: MissionEvent[];
  pendingDecisions: PendingOperatorDecision[];
}

const STATUS_STYLES: Record<CaseStatus, string> = {
  COMPLETED:
    'border-emerald-300 bg-emerald-50 text-emerald-800 dark:border-emerald-800 dark:bg-emerald-950/40 dark:text-emerald-200',
  WAITING:
    'border-amber-300 bg-amber-50 text-amber-800 dark:border-amber-800 dark:bg-amber-950/40 dark:text-amber-200',
  FAULTED:
    'border-red-300 bg-red-50 text-red-800 dark:border-red-800 dark:bg-red-950/40 dark:text-red-200',
  RETRYING:
    'border-blue-300 bg-blue-50 text-blue-800 ring-2 ring-blue-200 dark:border-blue-800 dark:bg-blue-950/40 dark:text-blue-200 dark:ring-blue-900/60',
  EXHAUSTED:
    'border-orange-300 bg-orange-50 text-orange-900 ring-2 ring-orange-200 dark:border-orange-800 dark:bg-orange-950/40 dark:text-orange-200 dark:ring-orange-900/60',
  RUNNING:
    'border-blue-300 bg-blue-50 text-blue-800 ring-2 ring-blue-200 dark:border-blue-800 dark:bg-blue-950/40 dark:text-blue-200 dark:ring-blue-900/60',
  PENDING:
    'border-gray-200 bg-gray-50 text-gray-500 dark:border-gray-800 dark:bg-gray-900/70 dark:text-gray-400',
};

const STATUS_ICONS: Record<CaseStatus, React.ReactNode> = {
  COMPLETED: <CheckCircle2 className="h-4 w-4" />,
  WAITING: <Clock3 className="h-4 w-4" />,
  FAULTED: <AlertTriangle className="h-4 w-4" />,
  RETRYING: <RefreshCw className="h-4 w-4 animate-spin" />,
  EXHAUSTED: <AlertTriangle className="h-4 w-4" />,
  RUNNING: <Loader2 className="h-4 w-4 animate-spin" />,
  PENDING: <Circle className="h-4 w-4" />,
};

export function TestCaseMonitorPanel({
  visualizationModel,
  snapshot,
  events,
  pendingDecisions,
}: TestCaseMonitorPanelProps) {
  const catalog = visualizationModel?.test_case_catalog ?? [];
  const completed = new Set(snapshot?.blackboard_summary.completed_test_cases ?? []);
  const waypointToCase = new Map(
    catalog
      .filter((entry) => entry.waypoint)
      .map((entry) => [entry.waypoint as string, entry.test_case])
  );
  const waitingCases = new Set(
    pendingDecisions
      .map((decision) => decisionToCase(decision, snapshot, waypointToCase))
      .filter((testCase): testCase is number => typeof testCase === 'number')
  );
  const faultCase =
    snapshot?.health.last_fault?.target_waypoint && waypointToCase.has(snapshot.health.last_fault.target_waypoint)
      ? waypointToCase.get(snapshot.health.last_fault.target_waypoint)
      : null;
  const retrySummary = snapshot?.blackboard_summary.retry_summary ?? null;
  const retryCase =
    retrySummary?.waypoint && waypointToCase.has(retrySummary.waypoint)
      ? waypointToCase.get(retrySummary.waypoint)
      : null;
  const retryStatus =
    retrySummary?.status === 'EXHAUSTED'
      ? 'EXHAUSTED'
      : retrySummary?.status
        ? 'RETRYING'
        : null;
  const activeException = snapshot?.blackboard_summary.active_exception ?? null;
  const exceptionCases = new Set(activeException?.related_test_cases ?? []);
  const exceptionStatus: CaseStatus | null = activeException
    ? activeException.status === 'RECOVERING'
      ? 'RETRYING'
      : activeException.status === 'WAITING_OPERATOR'
        ? 'WAITING'
        : activeException.status === 'SAFE_STOP' || activeException.severity === 'P0'
          ? 'FAULTED'
          : 'FAULTED'
    : null;
  const currentCase = currentSnapshotCase(snapshot, catalog, waypointToCase);

  const cases: CaseViewModel[] = catalog.map((entry) => ({
    entry,
    status: statusForCase(entry.test_case, {
      completed,
      waitingCases,
      faultCase,
      retryCase,
      retryStatus,
      exceptionCases,
      exceptionStatus,
      currentCase,
    }),
  }));

  const completedCount = completed.size;
  const activeCase =
    cases.find((item) => item.status === 'EXHAUSTED') ??
    cases.find((item) => item.status === 'RETRYING') ??
    cases.find((item) => item.status === 'FAULTED') ??
    cases.find((item) => item.status === 'WAITING') ??
    cases.find((item) => item.status === 'RUNNING') ??
    cases.find((item) => item.entry.test_case === Math.min(completedCount + 1, catalog.length));
  const relatedEvents = latestRelatedEvents(events, activeCase?.entry.test_case);

  return (
    <section className="min-h-0 rounded-lg border border-gray-200 bg-white dark:border-gray-800 dark:bg-botbot-dark">
      <div className="flex min-h-12 flex-wrap items-center justify-between gap-3 border-b border-gray-200 px-4 py-3 dark:border-gray-800">
        <div className="flex items-center gap-2">
          <Activity className="h-4 w-4 text-blue-600" />
          <h2 className="text-sm font-semibold text-gray-900 dark:text-white">
            36 Test Cases
          </h2>
        </div>
        <div className="flex flex-wrap items-center gap-2">
          <span className="inline-flex h-7 items-center rounded-md border border-emerald-300 bg-emerald-50 px-2 text-xs font-semibold text-emerald-800 dark:border-emerald-800 dark:bg-emerald-950/40 dark:text-emerald-200">
            {completedCount}/{catalog.length || 36} completed
          </span>
          {snapshot ? (
            <span className="inline-flex h-7 items-center rounded-md border border-gray-200 bg-gray-50 px-2 text-xs font-semibold text-gray-700 dark:border-gray-700 dark:bg-gray-900 dark:text-gray-200">
              {snapshot.mission_state} | {snapshot.phase}
            </span>
          ) : null}
        </div>
      </div>

      {!catalog.length ? (
        <div className="flex min-h-28 items-center justify-center px-4 py-8 text-sm text-gray-500 dark:text-gray-400">
          Test case catalog has not been loaded
        </div>
      ) : (
        <div className="space-y-4 p-4">
          <div className="grid grid-cols-3 gap-2 sm:grid-cols-6 xl:grid-cols-9 2xl:grid-cols-12">
            {cases.map(({ entry, status }) => (
              <div
                key={entry.test_case}
                title={`${entry.module ?? 'Case'} | ${entry.title ?? entry.node}`}
                className={`flex aspect-[1.45] min-h-16 flex-col justify-between rounded-md border p-2 transition-colors ${STATUS_STYLES[status]}`}
              >
                <div className="flex items-center justify-between gap-2">
                  <span className="text-xs font-bold">TC{entry.test_case}</span>
                  {STATUS_ICONS[status]}
                </div>
                <div className="min-w-0">
                  <div className="truncate text-[11px] font-semibold">
                    {entry.module ?? status}
                  </div>
                  <div className="truncate text-[11px] opacity-80">
                    {entry.waypoint ?? entry.trigger ?? entry.node}
                  </div>
                </div>
              </div>
            ))}
          </div>

          <div className="grid gap-3 lg:grid-cols-[minmax(0,1.2fr)_minmax(0,0.8fr)]">
            <div className="rounded-lg border border-gray-200 bg-gray-50 p-3 dark:border-gray-800 dark:bg-black/15">
              <div className="text-xs font-semibold uppercase text-gray-500 dark:text-gray-400">
                Current Case
              </div>
              {activeCase ? (
                <div className="mt-2 min-w-0">
                  <div className="flex flex-wrap items-center gap-2">
                    <span className={`inline-flex h-7 items-center gap-1 rounded-md border px-2 text-xs font-semibold ${STATUS_STYLES[activeCase.status]}`}>
                      {STATUS_ICONS[activeCase.status]}
                      {activeCase.status}
                    </span>
                    <span className="text-sm font-semibold text-gray-900 dark:text-white">
                      TC{activeCase.entry.test_case}
                    </span>
                    <span className="text-sm text-gray-700 dark:text-gray-200">
                      {activeCase.entry.module ?? activeCase.entry.node}
                    </span>
                  </div>
                  <div className="mt-2 text-sm text-gray-700 dark:text-gray-300">
                    {activeCase.entry.title ?? activeCase.entry.node}
                  </div>
                  <div className="mt-2 truncate text-xs text-gray-500 dark:text-gray-400">
                    {activeCase.entry.node}
                    {activeCase.entry.waypoint ? ` | ${activeCase.entry.waypoint}` : ''}
                  </div>
                </div>
              ) : (
                <div className="mt-2 text-sm text-gray-500 dark:text-gray-400">
                  No active case
                </div>
              )}
            </div>

            <div className="rounded-lg border border-gray-200 bg-gray-50 p-3 dark:border-gray-800 dark:bg-black/15">
              <div className="text-xs font-semibold uppercase text-gray-500 dark:text-gray-400">
                Recent Case Events
              </div>
              {relatedEvents.length ? (
                <div className="mt-2 space-y-2">
                  {relatedEvents.map((event) => (
                    <div key={event.event_id} className="min-w-0">
                      <div className="truncate text-xs font-semibold text-gray-900 dark:text-white">
                        {event.event_type}
                      </div>
                      <div className="truncate text-xs text-gray-500 dark:text-gray-400">
                        seq {event.sequence} | sv {event.state_version}
                      </div>
                    </div>
                  ))}
                </div>
              ) : (
                <div className="mt-2 text-sm text-gray-500 dark:text-gray-400">
                  No related events yet
                </div>
              )}
            </div>
          </div>

          {snapshot?.health.last_fault ? (
            <div className="rounded-md border border-red-200 bg-red-50 p-3 text-xs text-red-800 dark:border-red-900 dark:bg-red-950/30 dark:text-red-200">
              <div className="font-semibold">
                {snapshot.health.last_fault.error_code ?? snapshot.health.last_fault.status}
              </div>
              <div className="mt-1">
                {snapshot.health.last_fault.message}
              </div>
              {snapshot.health.last_fault.action_id ? (
                <div className="mt-1 text-red-700/80 dark:text-red-200/80">
                  {snapshot.health.last_fault.target_waypoint} | {snapshot.health.last_fault.action_id}
                </div>
              ) : null}
            </div>
          ) : null}

          {retrySummary ? (
            <div className="rounded-md border border-blue-200 bg-blue-50 p-3 text-xs text-blue-900 dark:border-blue-900 dark:bg-blue-950/30 dark:text-blue-100">
              <div className="flex flex-wrap items-center gap-2">
                <span className={`inline-flex h-7 items-center gap-1 rounded-md border px-2 text-xs font-semibold ${retryStatus ? STATUS_STYLES[retryStatus] : STATUS_STYLES.RETRYING}`}>
                  {retryStatus ? STATUS_ICONS[retryStatus] : STATUS_ICONS.RETRYING}
                  {retrySummary.status}
                </span>
                <span className="font-semibold">
                  {retrySummary.waypoint ?? 'task'} | {retrySummary.action_id ?? 'action'}
                </span>
              </div>
              <div className="mt-2 grid gap-2 sm:grid-cols-2 lg:grid-cols-4">
                <RetryDetail label="Attempt" value={`${retrySummary.attempt ?? '-'} / ${retrySummary.max_attempts ?? '-'}`} />
                <RetryDetail label="Failure" value={retrySummary.failure_class ?? '-'} />
                <RetryDetail label="Error" value={retrySummary.error_code ?? '-'} />
                <RetryDetail label="Recovery" value={retrySummary.recovery_action ?? '-'} />
              </div>
              {retrySummary.next_retry_tick !== null && retrySummary.next_retry_tick !== undefined ? (
                <div className="mt-2 text-blue-800/80 dark:text-blue-100/80">
                  next retry tick {retrySummary.next_retry_tick}
                </div>
              ) : null}
            </div>
          ) : null}
        </div>
      )}
    </section>
  );
}

function RetryDetail({ label, value }: { label: string; value: React.ReactNode }) {
  return (
    <div className="min-w-0 rounded-md border border-blue-200 bg-white/70 px-2 py-1.5 dark:border-blue-900 dark:bg-black/15">
      <div className="text-[10px] font-semibold uppercase text-blue-700/70 dark:text-blue-200/70">
        {label}
      </div>
      <div className="mt-0.5 truncate text-xs font-semibold">
        {value}
      </div>
    </div>
  );
}

function statusForCase(
  testCase: number,
  context: {
    completed: Set<number>;
    waitingCases: Set<number>;
    faultCase: number | null | undefined;
    retryCase: number | null | undefined;
    retryStatus: 'RETRYING' | 'EXHAUSTED' | null;
    exceptionCases: Set<number>;
    exceptionStatus: CaseStatus | null;
    currentCase: number | null;
  }
): CaseStatus {
  if (context.exceptionCases.has(testCase) && context.exceptionStatus) {
    return context.exceptionStatus;
  }
  if (context.retryCase === testCase && context.retryStatus) {
    return context.retryStatus;
  }
  if (context.waitingCases.has(testCase)) {
    return 'WAITING';
  }
  if (context.faultCase === testCase) {
    return 'FAULTED';
  }
  if (context.currentCase === testCase) {
    return 'RUNNING';
  }
  if (context.completed.has(testCase)) {
    return 'COMPLETED';
  }
  return 'PENDING';
}

function currentSnapshotCase(
  snapshot: BTStateSnapshot | null,
  catalog: TestCaseCatalogEntry[],
  waypointToCase: Map<string, number>
): number | null {
  if (!snapshot) {
    return null;
  }
  if (snapshot.current_waypoint && waypointToCase.has(snapshot.current_waypoint)) {
    return waypointToCase.get(snapshot.current_waypoint) ?? null;
  }
  const currentNode = snapshot.blackboard_summary.current_node;
  if (currentNode) {
    const match = catalog.find((entry) => entry.node === currentNode);
    if (match) {
      return match.test_case;
    }
  }
  return null;
}

function decisionToCase(
  decision: PendingOperatorDecision,
  snapshot: BTStateSnapshot | null,
  waypointToCase: Map<string, number>
): number | null {
  const targetWaypoint = typeof decision.context?.target_waypoint === 'string' ? decision.context.target_waypoint : null;
  if (targetWaypoint && waypointToCase.has(targetWaypoint)) {
    return waypointToCase.get(targetWaypoint) ?? null;
  }
  if (decision.kind === 'START_MISSION') {
    return 3;
  }
  if (decision.kind === 'RETURN_TO_LOBBY') {
    return 33;
  }
  if (decision.kind === 'ELEVATOR_CLOSE_AND_PRESS_FLOOR') {
    return floorCase(snapshot?.target_floor ?? snapshot?.current_floor, { '11': 10, '14': 18, '15': 26 }) ?? 10;
  }
  if (decision.kind === 'ELEVATOR_EXIT') {
    return floorCase(snapshot?.target_floor ?? snapshot?.current_floor, { '11': 13, '14': 21, '15': 29 }) ?? 13;
  }
  if (decision.kind === 'DOOR_ACCESS') {
    return floorCase(snapshot?.current_floor, { '11': 15, '14': 23, '15': 31 }) ?? 15;
  }
  return null;
}

function floorCase(floor: string | null | undefined, mapping: Record<string, number>): number | null {
  if (!floor) {
    return null;
  }
  return mapping[floor] ?? null;
}

function latestRelatedEvents(events: MissionEvent[], testCase: number | undefined): MissionEvent[] {
  const interesting = new Set([
    'MISSION_TEST_CASE_REACHED',
    'WAYPOINT_REACHED',
    'MISSION_FAULT',
    'TASK_RETRY_SCHEDULED',
    'TASK_RETRY_EXHAUSTED',
    'TASK_OPERATOR_RETRY_APPROVED',
    'EXCEPTION_RAISED',
    'EXCEPTION_RESOLVED',
    'RECOVERY_SCHEDULED',
    'SAFE_STOP_ENTERED',
  ]);
  return [...events]
    .reverse()
    .filter((event) => {
      if (!interesting.has(event.event_type)) {
        return false;
      }
      if (testCase === undefined) {
        return true;
      }
      return Number(event.payload?.test_case) === testCase;
    })
    .slice(0, 4);
}
