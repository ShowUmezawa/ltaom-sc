#!/usr/bin/env python3
"""D0 immutable submaps + D1 observation-only candidates, offline and ROS live.

JSON schema 1 is explicit about IDs, coordinates and source scores. Neither this
node nor its workers publish optimizer input topics. World-Z gravity is an
explicit frontend assumption, not an IMU calibration quality assertion.
"""
import argparse
import csv
import hashlib
import io
import json
import math
import re
import select
import subprocess
import threading
import time
from collections import Counter
from pathlib import Path

import numpy as np
from tf.transformations import quaternion_matrix


def dump(path, value):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temp.replace(path)


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def message_hash(msg):
    b = io.BytesIO(); msg.serialize(b)
    return hashlib.sha256(b.getvalue()).hexdigest()


def matrix(pose):
    p = np.asarray(pose, dtype=float)
    if p.shape != (7,) or not np.isfinite(p).all() or abs(np.linalg.norm(p[3:])-1) > 1e-5:
        raise ValueError('invalid raw anchor pose')
    m = quaternion_matrix(p[3:]); m[:3, 3] = p[:3]
    return m


def cloud(msg):
    if msg.is_bigendian:
        raise ValueError('big-endian PointCloud2 is not supported by legacy adapter')
    names = ['x', 'y', 'z', 'intensity']; fields = {f.name: f for f in msg.fields}
    if any(n not in fields or fields[n].datatype != 7 or fields[n].count != 1 for n in names):
        raise ValueError('expected float32 XYZI PointCloud2')
    dtype = np.dtype(dict(names=names, formats=['<f4']*4,
        offsets=[fields[n].offset for n in names], itemsize=msg.point_step))
    a = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                   strides=(msg.row_step, msg.point_step)).reshape(-1)
    p = np.column_stack([a[n] for n in names])
    if not len(p) or not np.isfinite(p).all():
        raise ValueError('empty or nonfinite submap cloud')
    return p


def write_pcd(path, points):
    with path.open('xb') as f:
        f.write(('VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
                 'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' %
                 (len(points), len(points))).encode())
        f.write(np.asarray(points, dtype='<f4').tobytes())


class Archive:
    def __init__(self, root, session, scan_manifest):
        if not re.fullmatch(r'[A-Za-z0-9_-]+', session):
            raise ValueError('invalid session_id')
        self.root, self.session, self.scan_manifest = root, session, scan_manifest
        root.mkdir(exist_ok=False, parents=True)
        (root/'clouds').mkdir()
        self.stream = (root/'submaps.jsonl').open('x')
        self.records = []

    def add(self, odom, msg):
        raw_id = odom.twist.covariance[0]
        if not math.isfinite(raw_id) or int(raw_id) != raw_id or int(raw_id) != len(self.records):
            raise ValueError('submap IDs must be contiguous and start at zero')
        i = int(raw_id)
        if odom.header.stamp != msg.header.stamp:
            raise ValueError('submap cloud/anchor association stamp mismatch')
        if odom.header.frame_id != msg.header.frame_id:
            raise ValueError('submap cloud/anchor frame mismatch')
        pos, q = odom.pose.pose.position, odom.pose.pose.orientation
        pose = [pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w]; x = matrix(pose)
        # Legacy fields are decoded once here; schema 1 never overloads covariance.
        scans = list(csv.DictReader(self.scan_manifest.open()))
        association = np.array([float(s['association_time']) for s in scans])
        def scan_index(t):
            j = int(np.argmin(np.abs(association-t)))
            if abs(association[j]-t) > 2e-6:
                raise ValueError('scan manifest has no exact association stamp')
            return j
        first = scan_index(odom.twist.covariance[1]); last = scan_index(odom.twist.covariance[2])
        if first > last or scan_index(msg.header.stamp.to_sec()) != last:
            raise ValueError('invalid submap scan coverage')
        state_time = float(scans[last]['state_time'])
        if self.records and state_time <= self.records[-1]['state_time']:
            raise ValueError('nonmonotonic submap state time')
        world = cloud(msg)
        local = world.astype(np.float64)
        local[:, :3] = (local[:, :3]-x[:3, 3]) @ x[:3, :3]
        local = local.astype('<f4')
        restored = local[:, :3].astype(float) @ x[:3, :3].T + x[:3, 3]
        error = float(np.linalg.norm(restored-world[:, :3], axis=1).max())
        if error > .001:
            raise ValueError('local/world roundtrip exceeds 1 mm')
        yaw = math.atan2(x[1, 0], x[0, 0]); c, s = math.cos(yaw), math.sin(yaw)
        gravity = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) @ x[:3, :3]
        local_path = 'clouds/%06d.local.pcd' % i; world_path = 'clouds/%06d.world.pcd' % i
        write_pcd(self.root/local_path, local); write_pcd(self.root/world_path, world)
        record = dict(schema=1, session_id=self.session, submap_id=i,
            association_time_ns=msg.header.stamp.to_nsec(), state_time=state_time,
            first_scan_id=int(scans[first]['scan_id']), last_scan_id=int(scans[last]['scan_id']),
            raw_anchor_pose=pose, pose_convention='T_W_B; xyz,qx,qy,qz,qw',
            world_frame=msg.header.frame_id, local_frame='B_%d' % i,
            cloud_path=local_path, cloud_sha256=sha(self.root/local_path),
            world_cloud_path=world_path, world_cloud_sha256=sha(self.root/world_path),
            points=len(world), cloud_message_sha256=message_hash(msg), odom_message_sha256=message_hash(odom),
            cloud_header_seq=msg.header.seq, odom_header_seq=odom.header.seq,
            gravity_alignment_R=gravity.ravel().tolist(), gravity_direction_local=(-x[2, :3]).tolist(),
            gravity_quality='frontend_world_z_assumed_unvalidated',
            local_world_roundtrip_max_m=error, extrinsics='already_applied_by_frontend')
        self.stream.write(json.dumps(record, allow_nan=False)+'\n'); self.stream.flush()
        self.records.append(record)
        return record

    def finish(self, source=None):
        self.stream.close()
        manifest = dict(schema=1, status='complete', session_id=self.session,
            submaps=len(self.records), submaps_sha256=sha(self.root/'submaps.jsonl'),
            scan_manifest_sha256=sha(self.scan_manifest), source=source,
            immutable=True, max_roundtrip_m=max(r['local_world_roundtrip_max_m'] for r in self.records))
        dump(self.root/'manifest.json', manifest)
        return manifest


class Worker:
    def __init__(self, command, log):
        self.log = log.open('x')
        self.process = subprocess.Popen([str(x) for x in command], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=self.log, text=True, bufsize=1, start_new_session=True)

    def query(self, record):
        self.process.stdin.write(json.dumps(record)+'\n'); self.process.stdin.flush()
        if not select.select([self.process.stdout], [], [], 120)[0]:
            raise RuntimeError('candidate worker timed out')
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError('candidate worker exited unexpectedly')
        return json.loads(line)

    def close(self, success=True):
        self.process.stdin.close()
        if not success:
            self.process.terminate()
        try:
            code = self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.kill(); self.process.wait(); raise
        finally:
            self.log.close()
        if success and code:
            raise RuntimeError('candidate worker failed: %d' % code)


class Candidates:
    def __init__(self, root, session, mode, gap, seconds):
        self.root, self.session, self.gap, self.seconds = root, session, gap, seconds
        self.sources = {'std_only': {'STD'}, 'sc_only': {'SC'}, 'dual': {'STD', 'SC'}}[mode]
        self.records, self.batches, self.completed = {}, {}, set()
        self.merged, self.filtered = [], Counter()
        self.raw = (root/'candidate_batches.jsonl').open('x')
        self.events = (root/'candidates.jsonl').open('x')

    def add_record(self, r):
        if r['session_id'] != self.session or r['submap_id'] in self.records:
            raise ValueError('duplicate or cross-session submap record')
        self.records[r['submap_id']] = r

    def add_batch(self, batch):
        i, source = batch['current_id'], batch['source']
        if batch['schema'] != 1 or batch['session_id'] != self.session or source not in self.sources:
            raise ValueError('invalid candidate batch schema/session/source')
        if not isinstance(i, int) or i < 0 or batch.get('stage') != 'retrieved':
            raise ValueError('invalid candidate batch stage/id')
        key = (i, source)
        if key in self.batches:
            if self.batches[key] != batch:
                raise ValueError('conflicting duplicate candidate batch')
            return
        self.batches[key] = batch
        self.raw.write(json.dumps(batch, allow_nan=False)+'\n'); self.raw.flush()

    def process_ready(self):
        emitted = []
        for i in sorted(self.records):
            if i in self.completed:continue
            if not all((i, s) in self.batches for s in self.sources):break
            current = self.records[i]; merged = {}
            for source in sorted(self.sources):
                seen = set()
                for c in self.batches[i, source]['candidates']:
                    h = c['historical_id']
                    if not isinstance(h, int) or h < 0 or h >= i or h not in self.records:
                        raise ValueError('candidate references self, future or missing submap')
                    if h in seen:
                        raise ValueError('duplicate historical ID in source batch')
                    seen.add(h)
                    if not math.isfinite(c['score']):
                        raise ValueError('nonfinite candidate score')
                    if i-h <= self.gap or current['state_time']-self.records[h]['state_time'] < self.seconds:
                        self.filtered['recent_'+source] += 1; continue
                    pair = merged.setdefault(h, dict(schema=1, session_id=self.session,
                        loop_id='%s:loop:%d:%d' % (self.session, h, i), revision=0,
                        historical_id=h, current_id=i, state='retrieved', evidence=[],
                        graph_action='none', historical_cloud_sha256=self.records[h]['cloud_sha256'],
                        current_cloud_sha256=current['cloud_sha256']))
                    pair['evidence'].append(dict(source=source, **c))
            for h, pair in sorted(merged.items()):
                self.events.write(json.dumps(pair, allow_nan=False)+'\n')
                self.merged.append(pair); emitted.append(pair)
            self.events.flush(); self.completed.add(i)
        return emitted

    def finish(self):
        self.process_ready()
        if not self.records or self.completed != set(self.records) or len(self.batches) != len(self.records)*len(self.sources):
            raise ValueError('incomplete candidate batches at EOF')
        self.raw.close(); self.events.close()
        summary = dict(schema=1, status='passed', submaps=len(self.records), batches=len(self.batches),
            unique_pairs=len(self.merged), dual_source_pairs=sum(len(p['evidence']) == 2 for p in self.merged),
            source_pairs=dict(Counter(e['source'] for p in self.merged for e in p['evidence'])),
            filtered=dict(self.filtered), graph_insertions=0,
            candidates_sha256=sha(self.root/'candidates.jsonl'),
            qualification='retrieved candidates only; no geometric verification or precision/recall claim')
        dump(self.root/'candidate_summary.json', summary)
        if 'SC' in self.sources:
            dump(self.root/'sc_database_index.json', dict(schema=1, session_id=self.session,
                parameters=dict(rings=20, sectors=60, radius_m=80, height_encoding_offset_m=2),
                entries=[dict(descriptor_index=i, submap_id=i, cloud_sha256=self.records[i]['cloud_sha256'],
                    gravity_alignment_R=self.records[i]['gravity_alignment_R']) for i in sorted(self.records)]))
        return summary


def verify_archive(root):
    manifest = json.loads((root/'manifest.json').read_text())
    if manifest['status'] != 'complete' or sha(root/'submaps.jsonl') != manifest['submaps_sha256']:
        raise ValueError('incomplete or modified archive')
    records = [json.loads(s) for s in (root/'submaps.jsonl').read_text().splitlines()]
    if len(records) != manifest['submaps']:
        raise ValueError('submap count mismatch')
    for i, r in enumerate(records):
        if r['schema'] != 1 or r['submap_id'] != i or r['session_id'] != manifest['session_id']:
            raise ValueError('invalid submap identity')
        matrix(r['raw_anchor_pose'])
        for name, digest in [('cloud_path', 'cloud_sha256'), ('world_cloud_path', 'world_cloud_sha256')]:
            p = (root/r[name]).resolve()
            if root.resolve() not in p.parents or sha(p) != r[digest]:
                raise ValueError('invalid path or cloud hash')
    return records


def workers(args, root, out, native_std=False):
    found = {}
    if args.mode != 'std_only':
        found['SC'] = Worker([args.bin_dir/'sc_candidates', root, args.gap, args.seconds, args.top_k, args.sc_threshold], out/'sc.log')
    if args.mode != 'sc_only' and not native_std:
        found['STD'] = Worker([args.bin_dir/'std_candidates', root, args.std_config], out/'std.log')
    return found


def run_archive(args):
    import rosbag
    archive = Archive(args.out, args.session, args.scan_manifest)
    odoms = {}
    with rosbag.Bag(str(args.bag)) as bag:
        for _, msg, _ in bag.read_messages(topics=['/submap_pose']):
            stamp = msg.header.stamp.to_nsec()
            if stamp in odoms:
                raise ValueError('duplicate source submap stamp')
            odoms[stamp] = msg
        for _, msg, _ in bag.read_messages(topics=['/clouds_submap']):
            archive.add(odoms.pop(msg.header.stamp.to_nsec()), msg)
    if odoms:
        raise ValueError('missing submap clouds')
    print(json.dumps(archive.finish(dict(bag=str(args.bag), sha256=sha(args.bag))), indent=2))


def run_candidates(args):
    records = verify_archive(args.archive)
    args.out.mkdir(parents=True, exist_ok=False)
    dump(args.out/'config.json', {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()})
    manager = Candidates(args.out, records[0]['session_id'], args.mode, args.gap, args.seconds)
    jobs = workers(args, args.archive, args.out); success = False
    try:
        for record in records:
            manager.add_record(record)
            for source, worker in jobs.items():
                manager.add_batch(worker.query(record))
            manager.process_ready()
            dump(args.out/'status.json', dict(submaps=len(manager.records), completed=len(manager.completed),
                 unique_pairs=len(manager.merged), last_submap=record['submap_id'], pending=0))
        print(json.dumps(manager.finish(), indent=2)); success = True
    finally:
        for job in jobs.values():
            job.close(success)


def run_live(args):
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger, TriggerResponse
    rospy.init_node('loop_candidate_observer')
    args.out.mkdir(parents=True, exist_ok=False)
    archive = Archive(args.out/'archive', args.session, args.scan_manifest)
    manager = Candidates(args.out, args.session, args.mode, args.gap, args.seconds)
    jobs = workers(args, archive.root, args.out, native_std=True)
    lock = threading.Lock(); odoms, clouds, batches = {}, {}, []
    state = dict(drained=False, eof=False, fatal='', received=Counter(), busy=False)
    publisher = rospy.Publisher('/loop_pipeline/candidates', String, queue_size=100)

    def receive(kind, msg):
        with lock:
            if state['eof']:
                state['fatal'] = 'input_after_eof'; return
            state['received'][kind] += 1
            try:
                if kind == 'STD':
                    batches.append(json.loads(msg.data))
                elif kind == 'odom':
                    k = msg.twist.covariance[0]
                    if not math.isfinite(k) or k != int(k) or int(k) < len(archive.records) or int(k) in odoms:
                        raise ValueError('duplicate/invalid live submap ID')
                    odoms[int(k)] = msg
                else:
                    k = msg.header.stamp.to_nsec()
                    if k in clouds:
                        raise ValueError('duplicate live cloud stamp')
                    clouds[k] = msg
            except (ValueError, TypeError, KeyError) as e:
                state['fatal'] = str(e); return
            if len(odoms)+len(clouds)+len(batches) > 512:
                state['fatal'] = 'bounded input queue overflow'

    subscriptions = [rospy.Subscriber('/submap_pose', Odometry, lambda m: receive('odom', m), queue_size=100),
        rospy.Subscriber('/clouds_submap', PointCloud2, lambda m: receive('cloud', m), queue_size=100, buff_size=64*1024*1024)]
    if args.mode != 'sc_only':
        subscriptions.append(rospy.Subscriber('/loop_pipeline/std_candidates', String, lambda m: receive('STD', m), queue_size=100))

    def drain(_):
        with lock:
            # Producer must set expected_submaps after its own EOF acknowledgment.
            expected = rospy.get_param('~expected_submaps', -1)
            if expected < 0:
                return TriggerResponse(False, 'set ~expected_submaps after producer EOF')
            if state['fatal']:
                return TriggerResponse(False, state['fatal'])
            if state['drained']:
                return TriggerResponse(True, 'drained')
            if state['busy'] or odoms or clouds or batches or len(manager.completed) != expected:
                return TriggerResponse(False, 'pending inputs/candidate batches')
            state['eof'] = True
            return TriggerResponse(False, 'finalization requested')
    service = rospy.Service('~drain', Trigger, drain)
    success = False
    try:
        while not rospy.is_shutdown():
            with lock:
                if state['fatal']:
                    raise RuntimeError(state['fatal'])
                incoming = list(batches); batches.clear()
                i = len(archive.records); o = odoms.get(i); c = clouds.get(o.header.stamp.to_nsec()) if o else None
                if c is not None:
                    del odoms[i]; del clouds[o.header.stamp.to_nsec()]
                state['busy'] = bool(incoming) or c is not None
            if c is not None:
                record = archive.add(o, c); manager.add_record(record)
                for worker in jobs.values():
                    manager.add_batch(worker.query(record))
            for batch in incoming:
                manager.add_batch(batch)
            for pair in manager.process_ready():
                publisher.publish(String(json.dumps(pair)))
            with lock:
                state['busy'] = False
                if state['eof'] and not state['drained']:
                    archive.finish(); manager.finish()
                    for job in jobs.values():
                        job.close()
                    jobs.clear(); state['drained'] = True; success = True
                dump(args.out/'status.json', dict(state, submaps=len(archive.records), completed=len(manager.completed),
                    pending_odoms=len(odoms), pending_clouds=len(clouds), pending_batches=len(batches)))
            time.sleep(.02)
    except Exception as e:
        with lock:
            state['fatal'] = str(e); dump(args.out/'status.json', state)
        raise
    finally:
        for job in jobs.values():
            job.close(success)


def main():
    p = argparse.ArgumentParser(); sub = p.add_subparsers(dest='command', required=True)
    a = sub.add_parser('archive'); a.add_argument('--bag', type=Path, required=True)
    a.add_argument('--scan-manifest', type=Path, required=True); a.add_argument('--session', required=True); a.add_argument('--out', type=Path, required=True)
    for name in ['candidates', 'live']:
        a = sub.add_parser(name); a.add_argument('--out', type=Path, required=True)
        a.add_argument('--bin-dir', type=Path, required=True); a.add_argument('--std-config', type=Path, required=True)
        a.add_argument('--mode', choices=['std_only', 'sc_only', 'dual'], default='dual')
        a.add_argument('--gap', type=int, default=50); a.add_argument('--seconds', type=float, default=30)
        a.add_argument('--top-k', type=int, default=10); a.add_argument('--sc-threshold', type=float, default=.13)
        if name == 'candidates':
            a.add_argument('--archive', type=Path, required=True)
        else:
            a.add_argument('--scan-manifest', type=Path, required=True); a.add_argument('--session', required=True)
    args = p.parse_args()
    if args.command != 'archive' and (args.gap < 1 or args.seconds < 0 or args.top_k < 1):
        p.error('invalid exclusion/search configuration')
    {'archive': run_archive, 'candidates': run_candidates, 'live': run_live}[args.command](args)


if __name__ == '__main__':
    main()
