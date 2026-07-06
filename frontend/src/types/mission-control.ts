export type MissionRisk = 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
export type ExceptionSeverity = 'P0' | 'P1' | 'P2' | 'P3' | 'P4';

export type MissionState =
  | 'IDLE'
  | 'WAITING_OPERATOR'
  | 'ACTIVE'
  | 'PAUSED'
  | 'RETURNING'
  | 'COMPLETED'
  | 'SAFE_STOP'
  | 'EMERGENCY';

export type MissionControlAction =
  | 'PAUSE'
  | 'RESUME'
  | 'ABORT'
  | 'TAKEOVER'
  | 'RESET_IDLE'
  | 'RESYNC';

export type OperatorDecisionAction =
  | 'APPROVE'
  | 'DENY'
  | 'RETRY'
  | 'SKIP'
  | 'PAUSE'
  | 'TAKEOVER'
  | 'ABORT';

export type PendingDecisionStatus =
  | 'PENDING'
  | 'APPROVED'
  | 'DENIED'
  | 'EXPIRED'
  | 'CANCELLED';

export interface PendingOperatorDecision {
  schema_version: 'g1.operator_decision.v1';
  decision_id: string;
  mission_run_id: string;
  node_id: string;
  node_path: string;
  kind: string;
  risk: MissionRisk;
  status: PendingDecisionStatus;
  allowed_actions: OperatorDecisionAction[];
  required_evidence: string[];
  context: Record<string, unknown>;
  created_at: string;
  expires_at: string | null;
  state_version: number;
}

export interface BlackboardSummary {
  profile?: string;
  robot_prefix?: string;
  route_index?: number;
  route?: string[];
  completed_test_cases?: number[];
  last_completed_test_case?: number | null;
  current_node?: string | null;
  running_nav?: string[];
  running_task?: string | null;
  active_action?: ActiveTaskAction | null;
  retry_summary?: RetrySummary | null;
  task_history?: TaskHistoryEntry[];
  active_exception?: ExceptionInstance | null;
  exception_history?: ExceptionInstance[];
  route_id?: string | null;
  paused_from_state?: string | null;
  paused_from_phase?: string | null;
  paused_target_waypoint?: string | null;
  [key: string]: unknown;
}

export interface ExceptionDescriptor {
  schema_version?: 'g1.exception.descriptor.v1';
  exception_id: string;
  title: string;
  severity: ExceptionSeverity;
  risk: MissionRisk;
  phases: string[];
  related_test_cases: number[];
  action_classes: string[];
  error_codes: string[];
  default_disposition: string;
  allowed_operator_actions: OperatorDecisionAction[];
  forbidden_operator_actions?: OperatorDecisionAction[];
  user_message: string;
  evidence: string[];
}

export interface ExceptionInstance {
  schema_version: 'g1.exception.instance.v1';
  exception_instance_id: string;
  exception_id: string;
  mission_run_id: string | null;
  status:
    | 'ACTIVE'
    | 'RECOVERING'
    | 'WAITING_OPERATOR'
    | 'RESOLVED'
    | 'SAFE_STOP'
    | 'IGNORED';
  severity: ExceptionSeverity;
  risk: MissionRisk;
  phase: string;
  title: string;
  user_message: string;
  related_test_cases: number[];
  waypoint?: string | null;
  action_id?: string | null;
  action_class?: string | null;
  action_key?: string | null;
  error_code?: string | null;
  failure_class?: string | null;
  attempt?: number | null;
  max_attempts?: number | null;
  disposition: string;
  operator_decision_id?: string | null;
  evidence: Record<string, unknown>;
  raised_at: string;
  resolved_at?: string | null;
}

export interface ExceptionCatalog {
  schema_version: 'g1.exception_catalog.v1';
  exceptions: ExceptionDescriptor[];
}

export interface ActiveTaskAction {
  waypoint?: string;
  action_id?: string;
  action_class?: string;
  status?: string;
  attempt?: number;
  evidence?: Record<string, unknown>;
  [key: string]: unknown;
}

export interface RetrySummary {
  status?: 'RUNNING' | 'RECOVERING' | 'EXHAUSTED' | string;
  waypoint?: string;
  action_id?: string;
  attempt?: number;
  max_attempts?: number;
  error_code?: string | null;
  failure_class?: 'retriable' | 'terminal' | 'fatal' | string | null;
  recovery_action?: string | null;
  next_retry_tick?: number | null;
  [key: string]: unknown;
}

export interface TaskHistoryEntry {
  waypoint?: string;
  action_id?: string;
  action_class?: string;
  action_key?: string;
  status?: string;
  attempt?: number;
  error_code?: string | null;
  failure_class?: string | null;
  message?: string;
  [key: string]: unknown;
}

export interface MissionSnapshotHealth {
  adapter?: string;
  auto_approve?: boolean;
  last_fault?: {
    adapter?: string;
    status?: string;
    error_code?: string | null;
    message?: string;
    target_waypoint?: string | null;
    waypoint?: string;
    action_id?: string;
    action_class?: string;
    action_key?: string;
    attempt?: number;
    max_attempts?: number;
    failure_class?: string;
    [key: string]: unknown;
  } | null;
  [key: string]: unknown;
}

export interface BTStateSnapshot {
  schema_version: 'g1.bt.snapshot.v1';
  robot_id: string;
  mission_run_id: string | null;
  sequence: number;
  state_version: number;
  bt_tree_version: string;
  mission_state: MissionState;
  phase: string;
  current_floor: string;
  target_floor: string | null;
  current_waypoint: string | null;
  active_path: string[];
  nodes: Record<string, unknown>[];
  blackboard_summary: BlackboardSummary;
  health: MissionSnapshotHealth;
  pending_decisions: PendingOperatorDecision[];
  timestamp: string;
}

export interface MissionEvent {
  schema_version: 'g1.mission.event.v1';
  event_id: string;
  mission_run_id: string | null;
  sequence: number;
  state_version: number;
  event_type: string;
  source: string;
  node_id: string | null;
  node_path: string | null;
  payload: Record<string, unknown>;
  timestamp: string;
}

export interface MissionSupervisorHealth {
  ok: boolean;
  profile: string;
  robot_id: string;
}

export interface PreflightCheck {
  name: string;
  ok: boolean;
  message: string;
  details: Record<string, unknown>;
}

export interface PreflightSnapshot {
  schema_version: 'g1.preflight.snapshot.v1';
  robot_id: string;
  profile: string;
  mission_state: MissionState;
  state_version: number;
  go: boolean;
  blockers: string[];
  checks: PreflightCheck[];
  timestamp: string;
}

export interface MissionVoiceContext {
  schema_version: 'g1.mission.voice_context.v1';
  robot_id: string;
  mission_run_id: string | null;
  mission_state: MissionState;
  phase: string;
  node_id: string | null;
  current_waypoint: string | null;
  state_version: number;
  allowed_intents: string[];
  forbidden_intents: string[];
  required_auth: string[];
  risk_by_intent: Record<string, MissionRisk>;
  timestamp: string;
}

export type VoiceEventType =
  | 'wake_detected'
  | 'language_switch'
  | 'user_transcript_partial'
  | 'user_transcript_final'
  | 'agent_response_partial'
  | 'agent_response_final'
  | 'tts_started'
  | 'tts_finished'
  | 'supervisor_result'
  | 'error';

export interface VoiceEvent {
  schema_version: 'g1.voice.event.v1';
  event_id: string;
  conversation_id: string;
  turn_id?: string | null;
  source: string;
  event_type: VoiceEventType;
  timestamp: string;
  language?: string | null;
  text?: string | null;
  redacted_text?: string | null;
  wake_event_id?: string | null;
  livekit: {
    room?: string | null;
    participant_identity?: string | null;
    agent_session_id?: string | null;
  };
  mission: {
    mission_run_id?: string | null;
    state_version?: number | null;
    related_test_case?: number | null;
    waypoint?: string | null;
  };
  metadata: Record<string, unknown>;
}

export interface VoiceSessionSummary {
  schema_version: 'g1.voice.session.v1';
  conversation_id: string;
  status: string;
  language?: string | null;
  event_count: number;
  turn_count: number;
  started_at: string;
  updated_at: string;
  last_event_type?: VoiceEventType | null;
  last_text?: string | null;
}

export interface VoiceSessionsResponse {
  schema_version: 'g1.voice.sessions.v1';
  sessions: VoiceSessionSummary[];
}

export interface VoiceTranscriptSnapshot {
  schema_version: 'g1.voice.transcript.v1';
  conversation_id: string | null;
  transcript: ConversationTurn[];
  events: VoiceEvent[];
  updated_at: string;
}

export interface DemoAuthCodeState {
  schema_version: 'g1.auth.demo_code.v1';
  enabled: boolean;
  mode: string;
  profile: string;
  source: string;
  algorithm: string;
  digits: number;
  period_seconds: number;
  valid_window_steps: number;
  demo_visible: boolean;
  code: string | null;
  seconds_remaining: number | null;
  valid_from: string | null;
  expires_at: string | null;
  generated_at: string;
}

export interface AdapterStatusSnapshot {
  schema_version: 'g1.adapter.status.v1';
  robot_id: string;
  profile: string;
  adapter: string;
  available: boolean;
  running_commands: string[];
  last_error: Record<string, unknown> | null;
  last_seen_at: string;
  details: Record<string, unknown>;
}

export interface OperatorDecisionCommand {
  schema_version?: 'g1.operator_command.v1';
  command_id?: string;
  idempotency_key?: string;
  decision_id: string;
  mission_run_id: string;
  action: OperatorDecisionAction;
  expected_state_version?: number;
  operator_id: string;
  comment?: string;
}

export interface MissionControlCommand {
  schema_version?: 'g1.mission.control_command.v1';
  command_id?: string;
  idempotency_key?: string;
  mission_run_id?: string | null;
  command: MissionControlAction;
  expected_state_version?: number;
  operator_id: string;
  reason?: string;
}

export interface UserInstructionCommand {
  schema_version?: 'g1.user_instruction.v1';
  event_id?: string;
  idempotency_key?: string;
  source?: string;
  modality?: 'text' | 'voice';
  robot_id?: string;
  mission_run_id?: string | null;
  language?: string;
  text: string;
  wake_event_id?: string | null;
  state_version?: number;
}

export interface MissionControlResult {
  schema_version: 'g1.mission.control_result.v1';
  command_id: string;
  idempotency_key: string;
  command: MissionControlAction;
  accepted: boolean;
  message: string;
  snapshot: BTStateSnapshot;
}

export interface TestCaseCatalogEntry {
  test_case: number;
  node: string;
  waypoint?: string;
  trigger?: string;
  module?: string;
  title?: string;
  risk?: MissionRisk;
}

export interface VisualizationModel {
  robot_id: string;
  profile: string;
  route: string[];
  waypoints: Record<string, Record<string, unknown>>;
  test_case_catalog: TestCaseCatalogEntry[];
  exception_catalog?: ExceptionCatalog;
  visualization?: Record<string, unknown>;
}

export type MissionSupervisorConnectionStatus =
  | 'idle'
  | 'connecting'
  | 'connected'
  | 'error';

export type PlaybackStatus =
  | 'idle'
  | 'ready'
  | 'playing'
  | 'paused'
  | 'step_running'
  | 'failed'
  | 'completed'
  | 'stopped';

export type PlaybackControlAction =
  | 'PLAY'
  | 'PAUSE'
  | 'NEXT'
  | 'RESET'
  | 'STOP'
  | 'SET_PACE';

export type PlaybackStepStatus =
  | 'pending'
  | 'running'
  | 'passed'
  | 'failed'
  | 'skipped';

export interface PlaybackScenarioSummary {
  name: string;
  path: string;
  title: string;
  steps_total: number;
  test_cases_total?: number | null;
}

export interface PlaybackScenariosResponse {
  schema_version: 'g1.testing.playback.scenarios.v1';
  scenarios: PlaybackScenarioSummary[];
}

export interface PlaybackCreateCommand {
  schema_version?: 'g1.testing.playback.create.v1';
  scenario: string;
  pace?: 'manual' | 'auto';
  step_delay_seconds?: number;
  reset_supervisor?: boolean;
}

export interface PlaybackControlCommand {
  schema_version?: 'g1.testing.playback.control.v1';
  action: PlaybackControlAction;
  pace?: 'manual' | 'auto';
  step_delay_seconds?: number;
}

export interface PlaybackStep {
  index: number;
  action: string;
  status: PlaybackStepStatus;
  test_case?: number | null;
  title?: string | null;
  user_utterance?: string | null;
  robot_response?: string | null;
  duration_ms?: number | null;
  state_version?: number | null;
  mission_state?: string | null;
  current_waypoint?: string | null;
  error?: string | null;
  raw: Record<string, unknown>;
}

export interface ConversationTurn {
  turn_id: string;
  session_id: string;
  step_index: number;
  timestamp: string;
  speaker: 'staff' | 'visitor' | 'robot' | 'supervisor' | 'system';
  modality: 'voice' | 'text' | 'event';
  language?: string | null;
  text: string;
  redacted_text?: string | null;
  intent?: string | null;
  intent_confidence?: number | null;
  auth_valid?: boolean | null;
  related_test_case?: number | null;
  related_waypoint?: string | null;
  state_version?: number | null;
  source_event_id?: string | null;
  status: 'sent' | 'accepted' | 'rejected' | 'responded' | 'failed';
}

export interface PlaybackSessionSnapshot {
  schema_version: 'g1.testing.playback.session.v1';
  session_id: string;
  run_id: string;
  scenario_name: string;
  status: PlaybackStatus;
  current_step_index: number;
  steps_total: number;
  pace: 'manual' | 'auto';
  step_delay_seconds: number;
  steps: PlaybackStep[];
  transcript: ConversationTurn[];
  created_at: string;
  updated_at: string;
  last_error?: string | null;
}
