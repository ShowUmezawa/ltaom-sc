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
from .transformations import quaternion_matrix


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


