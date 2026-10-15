#!/usr/bin/env python3
"""Ordered streaming archive -> retrieval -> registration -> acknowledged Pose3."""
import argparse,json,math,resource,threading,time
from collections import Counter
from pathlib import Path
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger,TriggerResponse
from pipeline import Archive,Candidates,Worker,dump,sha
from verify import verify_one

p=argparse.ArgumentParser()
for name in ['out','bin-dir','std-config','config','scan-manifest','backend-status']:p.add_argument('--'+name,type=Path,required=True)
p.add_argument('--session',required=True);p.add_argument('--mode',choices=['std_only','sc_only','dual'],default='dual')
p.add_argument('--gap',type=int,default=50);p.add_argument('--seconds',type=float,default=30)
p.add_argument('--top-k',type=int,default=10);p.add_argument('--sc-threshold',type=float,default=.13)
a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
rospy.init_node('loop_pipeline',disable_signals=False)
cfg=json.loads(a.config.read_text());dump(a.out/'config.json',cfg)
archive=Archive(a.out/'archive',a.session,a.scan_manifest);manager=Candidates(a.out,a.session,a.mode,a.gap,a.seconds)
registration=Worker([a.bin_dir/'registration',a.config],a.out/'registration.log')
sc=None
if a.mode!='std_only':sc=Worker([a.bin_dir/'sc_candidates',archive.root,a.gap,a.seconds,a.top_k,a.sc_threshold],a.out/'sc.log')
lock=threading.Lock();odoms={};clouds={};batches=[];arrivals={}
state=dict(schema=1,phase='starting',fatal='',drained=False,eof=False,busy=False,verified=0,pairs=0,received=Counter(),max_pending=0,
 mode=a.mode,gravity_gated_sc_queries=0)
latencies=[];costs=[];reasons=Counter();pub=rospy.Publisher('/loop_pipeline/verified_loops',String,queue_size=100)
events=(a.out/'verification.jsonl').open('x');accepted=(a.out/'verified_loops.jsonl').open('x');timing=(a.out/'timing.jsonl').open('x')

def backend():
 if not a.backend_status.exists():return {}
 return json.loads(a.backend_status.read_text())

def wait_backend(predicate,why):
 end=time.monotonic()+90
 while not predicate(backend()):
  s=backend()
  if s.get('fatal'):raise RuntimeError('backend: '+s['fatal'])
  if rospy.is_shutdown() or time.monotonic()>end:raise RuntimeError('backend acknowledgement timeout: '+why)
  time.sleep(.02)

def receive(kind,msg):
 with lock:
  try:
   if state['eof']:raise ValueError('input_after_eof')
   state['received'][kind]+=1
   if kind=='STD':batches.append(json.loads(msg.data))
   elif kind=='odom':
    k=msg.twist.covariance[0]
    if not math.isfinite(k) or k<0 or k!=int(k) or int(k)<len(archive.records) or int(k) in odoms:raise ValueError('duplicate_or_invalid_submap')
    odoms[int(k)]=msg;arrivals[int(k)]=time.monotonic()
   else:
    k=msg.header.stamp.to_nsec()
    if k in clouds:raise ValueError('duplicate_cloud')
    clouds[k]=msg
   size=len(odoms)+len(clouds)+len(batches);state['max_pending']=max(state['max_pending'],size)
   if size>512:raise ValueError('bounded_queue_overflow')
  except Exception as e:state['fatal']=str(e)

subs=[rospy.Subscriber('/submap_pose',Odometry,lambda m:receive('odom',m),queue_size=512),
 rospy.Subscriber('/clouds_submap',PointCloud2,lambda m:receive('cloud',m),queue_size=256,buff_size=256*1024*1024)]
if a.mode!='sc_only':subs.append(rospy.Subscriber('/loop_pipeline/std_candidates',String,lambda m:receive('STD',m),queue_size=512))

def drain(_):
 with lock:
  if state['fatal']:return TriggerResponse(False,state['fatal'])
  if state['drained']:return TriggerResponse(True,'drained')
  expected=rospy.get_param('~expected_submaps',-1)
  if expected<0:return TriggerResponse(False,'missing_expected_submaps')
  if state['busy'] or odoms or clouds or batches or len(archive.records)!=expected or len(manager.completed)!=expected:
   return TriggerResponse(False,'pending_submaps_or_candidate_batches')
  state['eof']=True
  return TriggerResponse(False,'finalizing')
service=rospy.Service('~drain',Trigger,drain)
def save():
 dump(a.out/'status.json',dict(state,submaps=len(archive.records),completed_queries=len(manager.completed),pending_odoms=len(odoms),pending_clouds=len(clouds),pending_batches=len(batches),reasons=dict(reasons)))

success=False
try:
 state['phase']='ready';save()
 while not rospy.is_shutdown():
  with lock:
   if state['fatal']:raise RuntimeError(state['fatal'])
   incoming=list(batches);batches.clear();i=len(archive.records);o=odoms.get(i);c=clouds.get(o.header.stamp.to_nsec()) if o else None
   if c is not None:del odoms[i];del clouds[o.header.stamp.to_nsec()]
   state['busy']=bool(incoming) or c is not None
  if c is not None:
   start=time.monotonic();record=archive.add(o,c);manager.add_record(record)
   # Backend tails this committed record and independently checks ROS payload hashes.
   wait_backend(lambda s:s.get('submaps',0)>i,'submap '+str(i))
   if sc:
    b=sc.query(record)
    if not cfg['allow_assumed_gravity'] and record['gravity_quality']!='validated':
     b['candidates']=[];b['gated_reason']='gravity_quality_unvalidated';state['gravity_gated_sc_queries']+=1
    manager.add_batch(b)
   timing.write(json.dumps(dict(stage='archive_and_retrieval',submap_id=i,milliseconds=1000*(time.monotonic()-start)))+'\n');timing.flush()
  for b in incoming:manager.add_batch(b)
  before=set(manager.completed)
  for pair in manager.process_ready():
   start=time.monotonic();msg,event,times=verify_one(pair,archive.records,archive.root,a.out,registration,cfg)
   state['pairs']+=1;costs.extend(times);reasons[event['reason']]+=1;events.write(json.dumps(event,allow_nan=False)+'\n');events.flush()
   if msg:
    # Subscriber handshake and per-message terminal ACK: never silently drop a loop.
    wait_backend(lambda s:s.get('submaps',0)>msg['current_id'],'loop keys')
    if not pub.get_num_connections():raise RuntimeError('backend_verified_subscriber_disconnected')
    accepted.write(json.dumps(msg,allow_nan=False)+'\n');accepted.flush();pub.publish(String(json.dumps(msg,sort_keys=True,allow_nan=False)))
    state['verified']+=1;expected=state['verified']
    wait_backend(lambda s:s.get('accepted',0)+s.get('rejected',0)==expected and s.get('received',{}).get('verified_loops',0)==expected,'verified pair '+msg['loop_id'])
   timing.write(json.dumps(dict(stage='verification_and_commit',loop_id=pair['loop_id'],milliseconds=1000*(time.monotonic()-start)))+'\n');timing.flush()
  for i in sorted(manager.completed-before):
   ms=1000*(time.monotonic()-arrivals.pop(i));latencies.append(ms)
   timing.write(json.dumps(dict(stage='submap_end_to_end',submap_id=i,milliseconds=ms))+'\n');timing.flush()
  with lock:
   state['busy']=False
   if state['eof'] and not state['drained']:
    state['phase']='finalizing';archive.finish();manager.finish();registration.close(success=True);registration=None
    if sc:sc.close(success=True);sc=None
    events.close();accepted.close();timing.close()
    dump(a.out/'summary.json',dict(status='passed',mode=a.mode,submaps=len(archive.records),pairs=state['pairs'],verified=state['verified'],
     gravity_gated_sc_queries=state['gravity_gated_sc_queries'],reasons=dict(reasons),max_pending=state['max_pending'],
     submap_latency_p50_p95_max_ms=np.percentile(latencies,[50,95,100]).tolist() if latencies else [],
     registration_latency_p50_p95_ms=np.percentile(costs,[50,95]).tolist() if costs else [],
     peak_self_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,peak_child_rss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
     archive_sha256=sha(archive.root/'submaps.jsonl'),verified_sha256=sha(a.out/'verified_loops.jsonl')))
    state['drained']=True;state['phase']='drained';success=True
   save()
  time.sleep(.02)
except Exception as e:
 state['fatal']=str(e);state['phase']='failed';save();raise
finally:
 for worker in [registration,sc]:
  if worker:worker.close(success=False)
