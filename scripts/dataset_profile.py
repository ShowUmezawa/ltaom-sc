"""ROS2 IMU relay and replay resource monitoring."""
import hashlib
import json
import time
from pathlib import Path
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu
from sensor_profile import convert_imu


def ros2_parameter_types(value):
    """ROS2 requires homogeneous numeric arrays; retain numeric values."""
    if isinstance(value, dict):
        return {k: ros2_parameter_types(v) for k, v in value.items()}
    if isinstance(value, list):
        if value and all(type(v) in (int, float) for v in value) and any(type(v) is float for v in value):
            return [float(v) for v in value]
        return [ros2_parameter_types(v) for v in value]
    return value


class ImuRelay:
    def __init__(self, node, out, profile):
        self.out = Path(out)
        self.profile = profile
        self.enabled = profile['relay']
        self.count = 0
        self.input_hash = hashlib.sha256()
        self.output_hash = hashlib.sha256()
        if self.enabled:
            self.pub = node.create_publisher(Imu, profile['imu'], 8192)
            self.sub = node.create_subscription(Imu, profile['bag_imu'], self.receive, 8192)

    def receive(self, message):
        self.input_hash.update(serialize_message(message))
        self.pub.publish(convert_imu(message, self.profile))
        self.output_hash.update(serialize_message(message))
        self.count += 1

    def check(self, expected):
        if not self.enabled:
            return
        result = dict(count=self.count, expected=expected, scale=self.profile['acceleration_scale'],
                      angular_velocity_scale=self.profile['angular_velocity_scale'],
                      scale_covariances=self.profile['scale_covariances'],
                      input_topic=self.profile['bag_imu'], output_topic=self.profile['imu'],
                      input_cdr_sha256=self.input_hash.hexdigest(),
                      output_cdr_sha256=self.output_hash.hexdigest(),
                      unchanged='header, axes and orientation',
                      status='passed' if self.count == expected else 'failed')
        (self.out/'imu_relay.json').write_text(json.dumps(result, indent=2)+'\n')
        if self.count != expected:
            raise RuntimeError('IMU relay count mismatch')


class ResourceMonitor:
    def __init__(self, out, children):
        self.out, self.children = Path(out), children
        self.started = time.monotonic()

    def sample(self):
        processes = []
        for proc in self.children:
            try:
                status = Path('/proc/%d/status' % proc.pid).read_text().splitlines()
                rss = {line.split(':')[0]: int(line.split()[1])*1024
                       for line in status if line.startswith(('VmRSS:', 'VmHWM:'))}
                processes.append(dict(pid=proc.pid, command=list(map(str, proc.args)), **rss))
            except FileNotFoundError:
                pass
        queues = {}
        for stage in ['frontend', 'std', 'pipeline', 'backend']:
            path = self.out/stage/'queue_status.json'
            if path.exists():
                queues[stage] = json.loads(path.read_text())
        cgroup = Path('/sys/fs/cgroup/memory.current')
        data = dict(elapsed_s=time.monotonic()-self.started, processes=processes,
                    queues=queues, cgroup_memory_bytes=int(cgroup.read_text()) if cgroup.exists() else None)
        with (self.out/'resource_samples.jsonl').open('a') as f:
            f.write(json.dumps(data)+'\n')
