#!/usr/bin/env python3
"""Independent, persistent audit subscriber; does not share IMU relay callbacks."""
import argparse,json,signal,time,traceback
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos_event import SubscriptionEventCallbacks
from ltaom_interfaces.msg import Submap
from std_msgs.msg import String
from submap_hash import submap_hash,cloud_xyzi

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--session',required=True);a=p.parse_args()
    a.out.mkdir(exist_ok=False);rclpy.init();node=rclpy.create_node('submap_audit_observer')
    state=dict(ready=True,submaps=0,batches=0,last_scan_id=-1,fatal='',message_lost_events=0,incompatible_qos_events=0,max_payload_bytes=0)
    maps=(a.out/'submaps.jsonl').open('x',buffering=1);batches=(a.out/'candidates.jsonl').open('x',buffering=1)
    running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def save():
        state['updated_unix']=time.time()
        state['submap_publishers']=node.count_publishers('/loop_pipeline/submaps')
        state['candidate_publishers']=node.count_publishers('/loop_pipeline/std_candidates')
        tmp=a.out/'status.tmp';tmp.write_text(json.dumps(state,indent=2)+'\n');tmp.replace(a.out/'status.json')
    def event(name,msg):
        state[name]+=msg.total_count_change;save()
    callbacks=SubscriptionEventCallbacks(message_lost=lambda e:event('message_lost_events',e),incompatible_qos=lambda e:event('incompatible_qos_events',e))
    def on_submap(m):
        try:
            assert m.schema==1 and m.session_id==a.session,'schema/session mismatch'
            assert m.submap_id==state['submaps'],'noncontiguous or duplicate submap'
            assert m.first_scan_id==state['last_scan_id']+1 and m.first_scan_id<=m.last_scan_id,'scan coverage mismatch'
            assert m.cloud.header.stamp==m.association_time,'association timestamp mismatch'
            assert submap_hash(m)==m.semantic_sha256,'semantic digest mismatch'
            xyz=cloud_xyzi(m.cloud);assert np.isfinite(xyz).all(),'nonfinite points'
            p,q=m.anchor_pose.position,m.anchor_pose.orientation
            row=dict(submap_id=m.submap_id,first_scan_id=m.first_scan_id,last_scan_id=m.last_scan_id,points=len(xyz),semantic_sha256=m.semantic_sha256,
                     association_time_ns=m.association_time.sec*10**9+m.association_time.nanosec,raw_anchor_pose=[p.x,p.y,p.z,q.x,q.y,q.z,q.w])
            maps.write(json.dumps(row)+'\n')
            state.update(submaps=state['submaps']+1,last_scan_id=m.last_scan_id,last_submap_unix=time.time(),max_payload_bytes=max(state['max_payload_bytes'],len(m.cloud.data)))
            save()
        except Exception as e:
            state['fatal']=repr(e);save();raise
    def on_batch(m):
        try:
            batch=json.loads(m.data);batches.write(json.dumps(batch)+'\n');state['batches']+=1;save()
        except Exception as e:
            state['fatal']=repr(e);save();raise
    sub=node.create_subscription(Submap,'/loop_pipeline/submaps',on_submap,256,event_callbacks=callbacks)
    candidate=node.create_subscription(String,'/loop_pipeline/std_candidates',on_batch,256)
    try:
        save();last=0
        while running and rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()-last>=.5:save();last=time.monotonic()
    except Exception:
        traceback.print_exc();raise
    finally:
        save();maps.close();batches.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
