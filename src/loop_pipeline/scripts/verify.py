#!/usr/bin/env python3
"""Unified fixed-submap verification. Inputs stay immutable; each pair emits once."""
import argparse,json,math,time
from pathlib import Path
from collections import Counter
import numpy as np
import tf.transformations as tf
from pipeline import Worker,verify_archive,dump,sha,matrix

def transform(value):
    m=np.asarray(value,dtype=float).reshape(4,4)
    if not np.isfinite(m).all() or not np.allclose(m[3],[0,0,0,1],atol=1e-8) or not np.allclose(m[:3,:3].T@m[:3,:3],np.eye(3),atol=1e-5) or abs(np.linalg.det(m[:3,:3])-1)>1e-5:
        raise ValueError('invalid SE3')
    return m

def gravity(record):
    m=np.eye(4);m[:3,:3]=np.array(record['gravity_alignment_R']).reshape(3,3)
    return transform(m)

def pose(m):
    q=tf.quaternion_from_matrix(m);q/=np.linalg.norm(q)
    return [float(x) for x in list(m[:3,3])+list(q)]

def typed(v):
    if isinstance(v,dict):return {k:typed(x) for k,x in v.items()}
    if isinstance(v,list):return [typed(x) for x in v]
    if v in ['true','false']:return v=='true'
    try:return float(v)
    except (ValueError,TypeError):return v

def hypotheses(pair,h,c,config):
    Xh,Xc=matrix(h['raw_anchor_pose']),matrix(c['raw_anchor_pose'])
    result=[]
    for e in pair['evidence']:
        if e['source']=='STD' and e.get('has_initial_transform'):
            if e['initial_frame']!='world_current_to_history':raise ValueError('STD initial frame')
            initial=np.linalg.inv(Xh)@transform(e['initial_transform'])@Xc
            result.append(dict(source='STD',initial_transform=initial.ravel().tolist(),quatro=False))
        elif e['source']=='SC':
            valid=all(r['gravity_quality']=='validated' or (config['allow_assumed_gravity'] and r['gravity_quality']=='frontend_world_z_assumed_unvalidated') for r in [h,c])
            if not valid:continue
            if e['yaw_frame']!='gravity_aligned':raise ValueError('SC yaw frame')
            yaw=float(e['yaw_current_to_history_rad']);Y=tf.rotation_matrix(yaw,[0,0,1])
            initial=np.linalg.inv(gravity(h))@Y@gravity(c)
            initial[:3,3]=(np.linalg.inv(Xh)@Xc)[:3,3]
            result.append(dict(source='SC',initial_transform=initial.ravel().tolist(),quatro=False))
            valid=all(r['gravity_quality']=='validated' or (config['allow_assumed_gravity'] and r['gravity_quality']=='frontend_world_z_assumed_unvalidated') for r in [h,c])
            result.append(dict(source='SC_Quatro',initial_transform=initial.ravel().tolist(),quatro=True,
                gravity_valid=valid,source_gravity=gravity(c).ravel().tolist(),target_gravity=gravity(h).ravel().tolist(),yaw_rad=yaw))
    return result

def verify_one(pair,records,archive,out,worker,cfg):
    msg=None;latencies=[]
    h,c=pair['historical_id'],pair['current_id']
    if not 0<=h<c<len(records) or pair['session_id']!=records[h]['session_id']:raise ValueError('pair identity')
    for field,index in [('historical_cloud_sha256',h),('current_cloud_sha256',c)]:
        if pair[field]!=records[index]['cloud_sha256']:raise ValueError('pair cloud hash')
    attempts=[];passed=[]
    for i,initial in enumerate(hypotheses(pair,records[h],records[c],cfg)):
        # Quatro is the SC fallback when its yaw+odometry seed fails.
        if initial['source']=='SC_Quatro' and any(a['source']=='SC' and a['result']['accepted'] for a in attempts):continue
        request=dict(initial,source=str(archive/records[c]['cloud_path']),target=str(archive/records[h]['cloud_path']))
        request['correspondences_path']=str(out/('%d_%d_%d.correspondences.csv'%(h,c,i)))
        result=typed(worker.query(request));latencies.append(result['milliseconds'])
        attempt=dict(source=initial['source'],request=request,result=result);attempts.append(attempt)
        if result['accepted']:passed.append(attempt)
    # Conflicting successful hypotheses are ambiguous, never averaged.
    conflict=False
    for a in passed:
        for b in passed:
            delta=transform(a['result']['transform'])@np.linalg.inv(transform(b['result']['transform']))
            if np.linalg.norm(delta[:3,3])>1 or abs(math.acos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1)))>.1:conflict=True
    reason='conflicting_hypotheses' if conflict else ('passed' if passed else (attempts[-1]['result']['reason'] if attempts else 'no_full_initial_transform'))
    event=dict(loop_id=pair['loop_id'],historical_id=h,current_id=c,reason=reason,attempts=attempts)
    if passed and not conflict:
        best=min(passed,key=lambda a:a['result']['quality']['point_plane_rmse_m'])
        msg=dict(pair);msg.update(state='verified',graph_action='pose3_trial',measurement_convention='T_Bh_Bc',
            measurement=pose(transform(best['result']['transform'])),sigma_order='rx_ry_rz_tx_ty_tz',
            sigmas=[cfg['rotation_sigma_rad']]*3+[cfg['translation_sigma_m']]*3,
            sources=sorted({e['source'] for e in pair['evidence']}),selected_hypothesis=best['source'],
            quality=best['result']['quality'],validator_version=cfg['validator_version'],config_sha256=sha(out/'config.json'))
        path=Path(best['request']['correspondences_path']);msg['correspondences_path']=str(path);msg['correspondences_sha256']=sha(path)

    return msg,event,latencies

def run(args):
    args.out.mkdir(exist_ok=False);cfg=json.loads(args.config.read_text())
    dump(args.out/'config.json',cfg);records=verify_archive(args.archive)
    pairs=[json.loads(s) for s in args.candidates.read_text().splitlines()]
    worker=Worker([args.bin,args.config],args.out/'registration.log')
    seen=set();reasons=Counter();accepted=[];latencies=[];started=time.monotonic()
    try:
        with (args.out/'verification.jsonl').open('x') as events,(args.out/'verified_loops.jsonl').open('x') as verified:
            for pair in pairs:
                key=(pair['historical_id'],pair['current_id'])
                if key in seen:raise ValueError('duplicate input pair')
                seen.add(key)
                msg,event,costs=verify_one(pair,records,args.archive,args.out,worker,cfg)
                latencies.extend(costs);reason=event['reason']
                if msg:
                    verified.write(json.dumps(msg,allow_nan=False)+'\n');verified.flush();accepted.append(msg)
                reasons[reason]+=1;events.write(json.dumps(event,allow_nan=False)+'\n');events.flush()
                dump(args.out/'progress.json',dict(processed=len(seen),total=len(pairs),verified=len(accepted),reasons=dict(reasons)))
        worker.close(success=True);worker=None
        dump(args.out/'summary.json',dict(status='passed',pairs=len(seen),verified=len(accepted),reasons=dict(reasons),
            registrations=len(latencies),elapsed_seconds=time.monotonic()-started,latency_ms_p50=float(np.median(latencies)) if latencies else 0,
            latency_ms_p95=float(np.percentile(latencies,95)) if latencies else 0,archive_manifest_sha256=sha(args.archive/'manifest.json'),
            candidates_sha256=sha(args.candidates),verified_sha256=sha(args.out/'verified_loops.jsonl'),qualification='geometric verification; no independently labelled precision/recall claim'))
    finally:
        if worker:worker.close()

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['archive','candidates','config','bin','out']:p.add_argument('--'+name,type=Path,required=True)
    run(p.parse_args())
