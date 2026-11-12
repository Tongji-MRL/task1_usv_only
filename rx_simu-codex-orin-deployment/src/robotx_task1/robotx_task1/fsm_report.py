"""Task 1 messages for the team's USV state machine (no ROS imports).

The USV state machine (usv_orin_state_machine, version2) starts Task 1 and is
the only party Task 1 talks to; the vessel never reports to the OCS itself.
Its interface, as used by the Task 3 executor:

* /tongji_mrl/twave/progress, String JSON with vehicle/task/progress/error.
  The state machine forwards every message to /tongji_mrl/mission/status, and
  `progress == "FINISHED"` ends Task 1 (it then stops this launch).
* /tongji_mrl/twave/result, String JSON: one report body in the official
  RxReport field names, in the envelope of the Task 3 executor
  (team_id, vehicle_id, seq, sent_at, <body>). The state machine forwards it
  to /tongji_mrl/mission/task_report.

Task 1 sends one SafePassageReport with every classified buoy when the
mission ends, COMPLETE or ABORT (buoys reported after an abort still score),
and only then progress FINISHED, so the state machine moves on to the next
task either way; after an ABORT the reason is in `error` (null after COMPLETE).
"""
from datetime import datetime, timezone

TERMINAL = ('COMPLETE', 'ABORT')


def progress(state, error=None):
    """Progress message for a mission_core state name; both end states are FINISHED."""
    name = 'FINISHED' if state in TERMINAL else 'TASK1_'+state
    return dict(vehicle='usv', task='TASK1', progress=name, error=error)


def timestamp(unix_seconds):
    return datetime.fromtimestamp(unix_seconds, tz=timezone.utc).isoformat().replace('+00:00', 'Z')


class ReportEnvelope:
    """Sequenced result envelope, the same layout as the Task 3 executor's."""

    def __init__(self, team_id='TONG', vehicle_id='usv', initial_seq=0):
        if not str(team_id).strip() or not str(vehicle_id).strip():
            raise ValueError('team_id and vehicle_id must be non-empty')
        if isinstance(initial_seq, bool) or not isinstance(initial_seq, int) or initial_seq < 0:
            raise ValueError('initial_seq must be a nonnegative integer')
        self.team_id, self.vehicle_id, self.seq = str(team_id).strip(), str(vehicle_id).strip(), initial_seq

    def safe_passage(self, body, unix_seconds):
        self.seq += 1
        return dict(team_id=self.team_id, vehicle_id=self.vehicle_id, seq=self.seq,
                    sent_at=timestamp(unix_seconds), safe_passage=body)


class StateMachineLink:
    """Decides what to send to the state machine for each mission status.

    update() returns a list of ('progress' | 'result', payload) in send order.
    Progress is sent on every state change and refreshed every `refresh`
    seconds; the final progress is repeated every `repeat` seconds until the
    launch is stopped. At the end the report goes first: FINISHED makes the
    state machine stop this launch. Without a georeferenced report (no fresh
    GNSS fix) the final progress waits up to `report_wait` seconds, then goes
    out without one, and `missing_report` is set.
    """

    def __init__(self, envelope, refresh=2.0, repeat=1.0, report_wait=5.0):
        self.envelope, self.refresh, self.repeat, self.report_wait = envelope, refresh, repeat, report_wait
        self.state = self.sent_at = self.ended_at = self.finished_at = None
        self.reported = self.missing_report = False
        self.error = None

    def update(self, now, status, report_body, unix_seconds):
        state = status.get('state', '') if isinstance(status, dict) else ''
        if not state:
            return []
        out = []
        if state in TERMINAL:
            if self.ended_at is None:
                self.ended_at = now
                self.error = (status.get('detail') or 'aborted') if state == 'ABORT' else None
            if not self.reported:
                if report_body is not None:
                    out.append(('result', self.envelope.safe_passage(report_body, unix_seconds)))
                    self.reported = True
                elif now-self.ended_at < self.report_wait:
                    return out
                else:
                    self.reported = self.missing_report = True
            due = self.state != state or self.sent_at is None or now-self.sent_at >= self.repeat
            error = self.error
        else:
            due = self.state != state or self.sent_at is None or now-self.sent_at >= self.refresh
            error = None
        if due:
            out.append(('progress', progress(state, error)))
            self.state, self.sent_at = state, now
            if state in TERMINAL and self.finished_at is None:
                self.finished_at = now
        return out
