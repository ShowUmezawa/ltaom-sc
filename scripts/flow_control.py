"""Offline rosbag2 flow control; runs in the runner, never in optimizer callbacks."""
import json
import time
from pathlib import Path
from rosbag2_interfaces.srv import Pause, Resume, IsPaused


def pressures(statuses):
    """IMU-only accumulation cannot justify pausing the lidar that drains it."""
    fractions = []
    for stage, status in statuses.items():
        for name, queue in status.get('queues', {}).items():
            if stage == 'frontend' and name == 'imu':
                continue
            fractions.append(max(queue.get('count', 0) / max(queue.get('max_count', 1), 1),
                                 (queue.get('bytes', 0) + queue.get('active_bytes', 0)) /
                                 max(queue.get('max_bytes', 1), 1)))
        if stage == 'std' and status.get('credit_window', 0):
            fractions.append(status.get('inflight_submaps', 0) / status['credit_window'])
    return max(fractions, default=0.)


class FlowController:
    def __init__(self, node, out, status_paths, probe_pause_at=None):
        self.node, self.out = node, Path(out)
        self.paths = status_paths
        self.clients = {k: node.create_client(t, '/rosbag2_player/' + k)
                        for k, t in [('pause', Pause), ('resume', Resume), ('is_paused', IsPaused)]}
        self.paused = False
        self.pending = None
        self.action = None
        self.requested_at = 0.
        self.last_change = 0.
        self.started = time.monotonic()
        self.pauses = self.resumes = 0
        self.peak_pressure = 0.
        self.stall_begin = None
        self.probe_pause_at = probe_pause_at
        self.probe_done = False
        self.probe_until = 0.
        self.events = []

    def ready(self):
        return all(c.service_is_ready() for c in self.clients.values())

    def record(self, action, **fields):
        self.events.append(dict(elapsed_s=time.monotonic()-self.started, action=action, **fields))
        data = dict(paused=self.paused, pauses=self.pauses, resumes=self.resumes,
                    peak_pressure=self.peak_pressure, pending=self.action, events=self.events)
        tmp = self.out / 'flow_status.tmp'
        tmp.write_text(json.dumps(data, indent=2) + '\n')
        tmp.replace(self.out / 'flow_status.json')

    def tick(self, playing=True):
        if not playing:
            return
        now = time.monotonic()
        if self.pending:
            if not self.pending.done():
                if now-self.requested_at > 5:
                    raise RuntimeError('rosbag flow-control service timeout: '+self.action)
                return
            result = self.pending.result()  # Surface service failure.
            action = self.action
            self.pending = None
            self.action = None
            if action == 'confirm':
                if result.paused != self.paused:
                    raise RuntimeError('rosbag pause state did not match requested state')
                self.last_change = now
                self.record('confirmed', paused=result.paused)
            else:
                self.paused = action == 'pause'
                self.pauses += self.paused
                self.resumes += not self.paused
                self.action = 'confirm'
                self.pending = self.clients['is_paused'].call_async(IsPaused.Request())
                self.requested_at = now
                return
        if not playing or not self.ready() or now-self.last_change < .1:
            return
        statuses = {}
        for stage, path in self.paths.items():
            if not path.exists():
                continue
            status = json.loads(path.read_text())
            if status.get('fatal'):
                raise RuntimeError(stage+' queue failure: '+status['fatal'])
            statuses[stage] = status
        front = statuses.get('frontend', {})
        # Resume if a paused scan needs future IMU, even while downstream is busy.
        # Hard queue guards cover in-flight overshoot; there is no silent eviction.
        needs_imu = front.get('waiting_for_imu', False)
        pressure = pressures(statuses)
        self.peak_pressure = max(self.peak_pressure, pressure)
        stale = any(time.time()-s.get('updated_unix', time.time()) > 60 for s in statuses.values())
        if self.paused and stale:
            raise RuntimeError('stale queue telemetry while player paused')
        forced = False
        if self.probe_pause_at is not None:
            if not self.probe_done and now-self.started >= self.probe_pause_at and front and not needs_imu:
                self.probe_done = True
                self.probe_until = now+2
            forced = now < self.probe_until
        if self.paused:
            if self.stall_begin is not None and now-self.stall_begin > 90:
                raise RuntimeError('flow-control pause did not drain within 90 seconds')
            desired = not needs_imu and (pressure > .25 or forced)
        else:
            desired = bool(front) and not stale and not needs_imu and (pressure >= .75 or forced)
        if desired == self.paused:
            return
        self.action = 'pause' if desired else 'resume'
        if desired:
            self.stall_begin = now
        else:
            self.stall_begin = None
        self.record(self.action, pressure=pressure, needs_imu=needs_imu, probe=forced)
        self.pending = self.clients[self.action].call_async(Pause.Request() if desired else Resume.Request())
        self.requested_at = now
