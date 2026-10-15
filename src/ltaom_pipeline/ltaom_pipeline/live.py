"""Single-owner ROS2 streaming pipeline, exact IDs and terminal backend ACKs."""
import argparse,json,time,resource,os
from pathlib import Path
from collections import Counter
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from ltaom_interfaces.msg import Submap
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .archive import Archive
from .pipeline_core import Candidates,Worker,dump,sha
from .verify import verify_one

def main():
 p=argparse.ArgumentParser()
 for name in ['out','bin-dir','config','scan-manifest','backend-status']:p.add_argument('--'+name,type=Path,required=True)
 p.add_argument('--session',required=True);p.add_argument('--mode',choices=['std_only','sc_only','dual'],default='dual')
 p.add_argument('--gap',type=int,default=50);p.add_argument('--seconds',type=float,default=30);p.add_argument('--top-k',type=int,default=10);p.add_argument('--sc-threshold',type=float,default=.13)
 p.add_argument('--max-pending-bytes',type=int,default=1024**3);p.add_argument('--max-batch-bytes',type=int,default=16*1024**2)
 a=p.parse_args();
 if min(a.max_pending_bytes,a.max_batch_bytes)<=0:raise ValueError("buffer limits must be positive")
 a.out.mkdir(parents=True,exist_ok=False);rclpy.init();node=rclpy.create_node('loop_pipeline');node.declare_parameter('expected_submaps',-1)
 cfg=json.loads(a.config.read_text());dump(a.out/'config.json',cfg);archive=Archive(a.out/'archive',a.session,a.scan_manifest);manager=Candidates(a.out,a.session,a.mode,a.gap,a.seconds)
 registration=Worker([a.bin_dir/'registration',a.config],a.out/'registration.log');sc=None
 if a.mode!='std_only':sc=Worker([a.bin_dir/'sc_candidates',archive.root,a.gap,a.seconds,a.top_k,a.sc_threshold],a.out/'sc.log')
 queued={};batches=[];arrivals={};received=Counter();reasons=Counter();latencies=[];state=dict(pending_bytes=0,active_bytes=0,batch_bytes=0,peak_bytes=0,overflow=0,phase='starting',fatal='',busy=False,drained=False,eof=False,verified=0,pairs=0,max_pending=0,gravity_gated_sc_queries=0)
 records_pub=node.create_publisher(String,'/loop_pipeline/submap_records',256);verified_pub=node.create_publisher(String,'/loop_pipeline/verified_loops',256)
 ack_pub=node.create_publisher(String,'/loop_pipeline/consumption_ack',32);completed_prefix=0
 events=(a.out/'verification.jsonl').open('x');accepted=(a.out/'verified_loops.jsonl').open('x')
 def save():
  dump(a.out/'status.json',dict(state,submaps=len(archive.records),completed_queries=len(manager.completed),received=dict(received),pending_submaps=len(queued),pending_batches=len(batches)))
  rss=int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')
  dump(a.out/'queue_status.json',dict(updated_unix=time.time(),fatal=state['fatal'],rss_bytes=rss,peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
   queues=dict(pipeline_submap=dict(count=len(queued),bytes=state['pending_bytes'],active_bytes=state['active_bytes'],peak_bytes=state['peak_bytes'],max_count=512,max_bytes=a.max_pending_bytes,
    oldest_wait_ms=max([1000*(time.monotonic()-arrivals[i]) for i in queued]+[0]),high=(len(queued)>=384 or state['pending_bytes']+state['active_bytes']>=.75*a.max_pending_bytes)),
    candidate_batches=dict(count=len(batches),bytes=state['batch_bytes'],max_count=512,max_bytes=a.max_batch_bytes)),overflow=state['overflow']))
 def capacity(condition,reason):
  if condition:state['overflow']+=1;raise ValueError(reason)

 def on_packet(m):
  try:
   if state['eof'] or m.submap_id in queued or m.submap_id<len(archive.records) or m.session_id!=a.session:raise ValueError('duplicate, stale or wrong-session submap')
   capacity(len(queued)+len(batches)>=512 or len(m.cloud.data)>a.max_pending_bytes-state['pending_bytes']-state['active_bytes'],'pipeline submap capacity exceeded')
   state['pending_bytes']+=len(m.cloud.data);state['peak_bytes']=max(state['peak_bytes'],state['pending_bytes']+state['active_bytes'])
   queued[m.submap_id]=m;arrivals[m.submap_id]=time.monotonic();received['submap']+=1
   state['max_pending']=max(state['max_pending'],len(queued)+len(batches))
   if len(queued)+len(batches)>512:raise ValueError('bounded queue overflow')
  except Exception as e:state['fatal']=str(e);save()
 def on_batch(m):
  try:
   if state['eof']:raise ValueError('candidate after EOF')
   capacity(len(queued)+len(batches)>=512 or len(m.data.encode())>a.max_batch_bytes-state['batch_bytes'],'pipeline candidate capacity exceeded')
   state['batch_bytes']+=len(m.data.encode())
   batches.append(json.loads(m.data));received['STD']+=1
   state['max_pending']=max(state['max_pending'],len(queued)+len(batches))
   if len(queued)+len(batches)>512:raise ValueError('bounded queue overflow')
  except Exception as e:state['fatal']=str(e);save()
 def drain(_,res):
  expected=node.get_parameter('expected_submaps').value
  if not state['fatal'] and expected>0 and not state['busy'] and not queued and not batches and len(archive.records)==expected and len(manager.completed)==expected:state['eof']=True
  res.success=state['drained'] and not state['fatal'];res.message=state['fatal'] or ('drained' if res.success else 'await counts and finalize');return res
 sub=node.create_subscription(Submap,'/loop_pipeline/submaps',on_packet,256)
 std_sub=node.create_subscription(String,'/loop_pipeline/std_candidates',on_batch,256) if a.mode!='sc_only' else None
 service=node.create_service(Trigger,'/loop_pipeline/drain',drain)
 def backend():return json.loads(a.backend_status.read_text()) if a.backend_status.exists() else {}
 def wait_backend(predicate,why):
  end=time.monotonic()+90
  while not predicate(backend()):
   b=backend()
   if b.get('fatal') or state['fatal']:raise RuntimeError(b.get('fatal') or state['fatal'])
   if not rclpy.ok() or time.monotonic()>end:raise RuntimeError('backend acknowledgement timeout: '+why)
   rclpy.spin_once(node,timeout_sec=.02)
 try:
  state['phase']='ready';save()
  while rclpy.ok():
   rclpy.spin_once(node,timeout_sec=.01)
   if state['fatal']:raise RuntimeError(state['fatal'])
   i=len(archive.records);packet=queued.pop(i,None);incoming=list(batches);batches.clear();state['busy']=packet is not None or bool(incoming)
   state['batch_bytes']=0
   if packet is not None:
    state['pending_bytes']-=len(packet.cloud.data);state['active_bytes']=len(packet.cloud.data)
    record=archive.add(packet);manager.add_record(record)
    if not records_pub.get_subscription_count():raise RuntimeError('backend record subscriber disconnected')
    records_pub.publish(String(data=json.dumps(record,sort_keys=True,allow_nan=False)));wait_backend(lambda s:s.get('submaps',0)>i,'submap '+str(i))
    if sc:
     b=sc.query(record)
     if not cfg['allow_assumed_gravity'] and record['gravity_quality']!='validated':b['candidates']=[];b['gated_reason']='gravity_quality_unvalidated';state['gravity_gated_sc_queries']+=1
     manager.add_batch(b)
   for batch in incoming:manager.add_batch(batch)
   before=set(manager.completed)
   for pair in manager.process_ready():
    msg,event,cost=verify_one(pair,archive.records,archive.root,a.out,registration,cfg);state['pairs']+=1;reasons[event['reason']]+=1;events.write(json.dumps(event,allow_nan=False)+'\n');events.flush()
    if msg:
     if not verified_pub.get_subscription_count():raise RuntimeError('backend verified subscriber disconnected')
     payload=json.dumps(msg,sort_keys=True,allow_nan=False);accepted.write(payload+'\n');accepted.flush();verified_pub.publish(String(data=payload));state['verified']+=1;n=state['verified']
     wait_backend(lambda s:s.get('accepted',0)+s.get('rejected',0)==n and s.get('received_verified_loops',0)==n,'loop '+msg['loop_id'])
   for i in manager.completed-before:latencies.append(1000*(time.monotonic()-arrivals.pop(i)))
   packet=None  # Release the processed transport payload before returning credits.
   state['active_bytes']=0
   while completed_prefix in manager.completed:completed_prefix+=1
   ack_pub.publish(String(data=json.dumps(dict(schema=1,session_id=a.session,completed_submaps=completed_prefix))))
   state['busy']=False
   if state['eof'] and not state['drained']:
    archive.finish();manager.finish();registration.close();registration=None
    if sc:sc.close();sc=None
    events.close();accepted.close();dump(a.out/'summary.json',dict(status='passed',mode=a.mode,submaps=len(archive.records),pairs=state['pairs'],verified=state['verified'],reasons=dict(reasons),gravity_gated_sc_queries=state['gravity_gated_sc_queries'],max_pending=state['max_pending'],submap_latency_p50_p95_max_ms=np.percentile(latencies,[50,95,100]).tolist(),archive_sha256=sha(archive.root/'submaps.jsonl'),verified_sha256=sha(a.out/'verified_loops.jsonl')))
    state['drained']=True;state['phase']='drained'
   save()
 except (KeyboardInterrupt,ExternalShutdownException):
  if not state['drained']:raise
 except Exception as e:state['fatal']=str(e);state['phase']='failed';save();raise
 finally:
  for worker in [registration,sc]:
   if worker:worker.close(success=False)
  node.destroy_node();
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
