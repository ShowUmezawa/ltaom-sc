#!/usr/bin/env python3
"""ROS1 replay for built-in and custom sensor profiles."""
import argparse,csv,json,os,signal,subprocess,time,traceback,shutil
from pathlib import Path
import numpy as np
import yaml
from sensor_profile import add_profile_arguments, load_profile, frontend_parameters, check_bag_topics, snapshot_profile
import rosbag,rosgraph,rospy
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from pipeline import dump,sha,matrix
p=argparse.ArgumentParser()
p.add_argument('--bag',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
add_profile_arguments(p, required=True);p.add_argument('--mode',choices=['frontend','std_only','sc_only','dual'],default='dual')
p.add_argument('--rate',type=float,default=1);p.add_argument('--port',type=int,default=11444)
a=p.parse_args()
root=Path(__file__).resolve().parents[1]
dataset=load_profile(root,'ros1',a.dataset,a.sensor_config,a.mode!='frontend')
a.dataset=dataset['name']
config_values=frontend_parameters(dataset,str(a.out)+'/')
a.out.mkdir(parents=True,exist_ok=False)
snapshot_profile(dataset,a.out)
shutil.copy2(__file__,a.out/'runner_snapshot.py')
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:'+str(a.port),ROS_HOSTNAME='127.0.0.1',ROS_LOG_DIR=str(a.out/'ros'))
root=Path(__file__).resolve().parents[1];src=root/'src';build=Path(os.environ.get('LTAOM_BUILD_DIR',str(root/'build')));exe=build/'ros1_ws/devel/lib'
from run_metadata import record
record(root,a.out,a)
bag=a.bag.resolve()
lidar_topic=dataset['lidar'];imu_topic=dataset['bag_imu'];frontend_imu=dataset['imu']
with rosbag.Bag(str(bag)) as b:
 topics=b.get_type_and_topic_info().topics;duration=b.get_end_time()-b.get_start_time()
 check_bag_topics(dataset,{k:v.msg_type for k,v in topics.items()},'ros1')
 expected_lidar=topics[lidar_topic].message_count;expected_imu=topics[imu_topic].message_count
session=a.dataset+'_'+a.out.name
config=Path(dataset['geometry']) if dataset['geometry'] else None
std_config=Path(dataset['std']) if dataset['std'] else None
frontend_config=a.out/'frontend_params.yaml'
frontend_config.write_text(yaml.safe_dump(config_values))
children=[];files=[];critical={};peak_rss={};state=dict(schema=1,phase='starting',dataset=a.dataset,mode=a.mode,rate=a.rate,poses=0,submaps=0,started_unix=time.time())
observed=(a.out/'observed_odometry.tum').open('w',buffering=1);files.append(observed)

def spawn(command,name,required=True):
 f=(a.out/(name+'.log')).open('w');files.append(f)
 proc=subprocess.Popen(list(map(str,command)),stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append((name,proc))
 if required:critical[name]=proc
 return proc

def read(path):return json.loads(path.read_text()) if path.exists() else {}
def save():
 for name,proc in children:
  try:
   lines=Path('/proc/%d/status'%proc.pid).read_text().splitlines()
   rss=max([int(s.split()[1]) for s in lines if s.startswith(('VmRSS:','VmHWM:'))]+[0]);peak_rss[name]=max(peak_rss.get(name,0),rss)
  except FileNotFoundError:pass
 state['peak_rss_kib']=dict(peak_rss);state['updated_unix']=time.time();dump(a.out/'status.json',state)
def health():
 for name,proc in critical.items():
  if proc.poll() is not None:raise RuntimeError(name+' exited '+str(proc.returncode))
 for path in [a.out/'backend_status.json',a.out/'pipeline/status.json']:
  s=read(path)
  if s.get('fatal'):raise RuntimeError(str(path)+': '+s['fatal'])
def wait(fn,why,timeout=120):
 end=time.monotonic()+timeout;state['phase']=why;save()
 while not fn():
  health();save()
  if time.monotonic()>end:raise RuntimeError('timeout: '+why)
  time.sleep(.25)
def stop(name,proc):
 critical.pop(name,None)
 if proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=40)
  except subprocess.TimeoutExpired:
   os.killpg(proc.pid,signal.SIGKILL);proc.wait();state['forced_shutdown']=True

def odom(m):
 v=m.pose.pose;p,q=v.position,v.orientation
 observed.write(' '.join('%.12f'%x for x in [m.header.stamp.to_sec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w])+'\n')
 state['poses']+=1;state['last_pose_time']=m.header.stamp.to_sec()
def drain(name,timeout=120):
 rospy.wait_for_service(name,timeout=30);service=rospy.ServiceProxy(name,Trigger)
 wait(lambda:service().success,name,timeout)

try:
 spawn(['roscore','-p',a.port],'core')
 wait(lambda:subprocess.run(['rosparam','get','/rosversion'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0,'master')
 rospy.init_node('d3_runner',disable_signals=True)
 for f in [frontend_config,src/'loop_optimization/launch/loopopt_config_mulran.yaml']:
  subprocess.run(['rosparam','load',str(f)],check=True)
 params=dict(SaveDir=str(a.out)+'/',backend_mode='rebuildable',session_id=session,backend_audit_factors=True,
  SavePCD=True,PubCorrectedMap=False,frontend_feedback_en=False,ikdtree_rebuild_mode='sync',multisession_mode=0,adjacent_factors_en=False,loop_factors_en=a.mode!='frontend',
  loop_policy='native',factor_compression_en=False,loop_input_mode='verified_pose3',submap_records=str(a.out/'pipeline/archive/submaps.jsonl'),
  streaming_submap_records=True,export_loop_candidates=a.mode in ['std_only','dual'],candidate_only=True,std_retrieval_en=a.mode in ['std_only','dual'],
  flush_tail_submap=True,lcd_config_path=str(std_config),use_sim_time=False)
 for k,v in params.items():rospy.set_param('/'+k,v)
 dump(a.out/'inputs.json',dict(bag=str(bag),expected_lidar=expected_lidar,expected_imu=expected_imu,duration_s=duration,
  frontend_config_sha256=sha(frontend_config),geometry_config_sha256=sha(config) if config else None,std_config_sha256=sha(std_config) if std_config else None,parameters=params))
 if a.mode=='frontend':
  if dataset['relay']:spawn(['python3',root/'scripts/imu_relay.py','--profile',a.out/'sensor_profile.json'],'imu_relay')
  spawn([exe/'fast_lio/fastlio_node'],'frontend')
  sub=rospy.Subscriber('/Odometry',Odometry,odom,queue_size=10000)
  master=rosgraph.Master('/d3_frontend_runner')
  def connected_front():
   _,subs,_=master.getSystemState();subs=dict(subs)
   return lidar_topic in subs and imu_topic in subs and frontend_imu in subs
  wait(connected_front,'sensor_subscribers')
  subprocess.run(['rosparam','dump',str(a.out/'parameters.yaml')],check=True)
  start=time.monotonic();player=spawn(['rosbag','play','--quiet','--delay=2','-r',a.rate,str(bag),'--topics',lidar_topic,imu_topic],'play',False)
  wait(lambda:player.poll() is not None,'playing',duration/a.rate+180)
  if player.returncode:raise RuntimeError('rosbag play failed')
  state['playback_wall_s']=time.monotonic()-start
  rospy.set_param('/expected_lidar_messages',expected_lidar);rospy.set_param('/expected_imu_messages',expected_imu)
  drain('/frontend/drain',180);front=read(a.out/'frontend_status.json')
  assert front['received_lidar']==expected_lidar and front['received_imu']==expected_imu and front['invalid_lidar']==0
  assert front['consumed_lidar']+front['uncovered_tail']==expected_lidar
  wait(lambda:state['poses']==front['output_scans'],'observed_frontend_count')
  trajectory=np.loadtxt(a.out/'trajectory.tum',ndmin=2)
  assert len(trajectory)==front['output_scans'] and np.isfinite(trajectory).all() and np.all(np.diff(trajectory[:,0])>0)
  state.update(phase='passed',elapsed_s=time.monotonic()-start);save()
  dump(a.out/'result.json',dict(status='passed',frontend=front,performance=state,mode='frontend',
   limitations=['Frontend trajectory baseline; no backend or loop pipeline running','No independent dog ground truth']))
  (a.out/'STATUS').write_text('FRONTEND_REPLAY_PASSED\n')
  raise SystemExit(0)
 optimizer=spawn([exe/'loop_optimization/loop_optimization_ablation_node'],'backend')
 spawn(['python3',src/'loop_pipeline/scripts/live_pipeline.py','--out',a.out/'pipeline','--bin-dir',exe/'loop_pipeline',
  '--std-config',std_config,'--config',config,'--scan-manifest',a.out/'scan_manifest.csv','--backend-status',a.out/'backend_status.json',
  '--session',session,'--mode',a.mode,'--gap',dataset['gap'],'--seconds',dataset['seconds'],'--top-k',dataset['top_k'],'--sc-threshold',dataset['sc_threshold']],'pipeline')
 wait(lambda:read(a.out/'pipeline/status.json').get('phase')=='ready','pipeline_ready')
 if dataset['relay']:spawn(['python3',root/'scripts/imu_relay.py','--profile',a.out/'sensor_profile.json'],'imu_relay')
 spawn([exe/'fast_lio/fastlio_node'],'frontend')
 spawn(['/opt/ros/noetic/lib/nodelet/nodelet','manager','__name:=std_manager'],'std_manager')
 spawn(['rosrun','nodelet','nodelet','load','std_loop/loop_detection_plugin','std_manager','__name:=std_loader'],'std_loader')
 sub=rospy.Subscriber('/Odometry',Odometry,odom,queue_size=10000)
 record_topics=['/submap_pose','/clouds_submap','/inter_triangles','/inter2_triangles','/loop_pipeline/std_candidates','/loop_pipeline/verified_loops']
 # Own the native recorder PID: rosbag's Python wrapper can exit on SIGINT
 # before its child has renamed the .active file and closed its index.
 recorder=spawn(['/opt/ros/noetic/lib/rosbag/record','-b','512','-O',a.out/'backend_inputs.bag']+record_topics,'record')
 master=rosgraph.Master('/d3_runner')
 def connected():
  pubs,subs,_=master.getSystemState();subs=dict(subs);pubs=dict(pubs)
  return '/cloud_registered' in subs and '/aft_mapped_to_init' in subs and lidar_topic in subs and imu_topic in subs and frontend_imu in subs and '/submap_pose' in pubs and len(subs.get('/submap_pose',[]))>=3 and len(subs.get('/loop_pipeline/verified_loops',[]))>=2
 wait(connected,'sensor_and_record_subscribers')
 subprocess.run(['rosparam','dump',str(a.out/'parameters.yaml')],check=True)
 start=time.monotonic();player=spawn(['rosbag','play','--quiet','--delay=2','-r',a.rate,str(bag),'--topics',lidar_topic,imu_topic],'play',False)
 wait(lambda:player.poll() is not None,'playing',duration/a.rate+180)
 if player.returncode:raise RuntimeError('rosbag play failed')
 state['playback_wall_s']=time.monotonic()-start
 rospy.set_param('/expected_lidar_messages',expected_lidar);rospy.set_param('/expected_imu_messages',expected_imu)
 drain('/frontend/drain',180);front=read(a.out/'frontend_status.json')
 assert front['received_lidar']==expected_lidar and front['received_imu']==expected_imu and front['invalid_lidar']==0
 assert front['consumed_lidar']+front['uncovered_tail']==expected_lidar
 wait(lambda:state['poses']==front['output_scans'],'observed_frontend_count')
 rospy.set_param('/std_expected_scans',front['output_scans'])
 wait(lambda:read(a.out/'std_status.json').get('drained'),'std_eof',240);std=read(a.out/'std_status.json')
 assert std['consumed_scans']==front['output_scans'] and std['submaps']>=1
 rospy.set_param('/loop_pipeline/expected_submaps',std['submaps']);drain('/loop_pipeline/drain',240)
 pipeline=read(a.out/'pipeline/summary.json');assert pipeline['submaps']==std['submaps']
 rospy.set_param('/expected_submaps',std['submaps']);rospy.set_param('/expected_verified_loops',pipeline['verified'])
 wait(lambda:read(a.out/'backend_status.json').get('received',{}).get('inter_triangles',0)==std['submaps'],'adjacent_input_count')
 drain('/loop_optimizer/drain',120);back=read(a.out/'backend_status.json')
 assert back['submaps']==std['submaps'] and back['received']['clouds_submap']==std['submaps'] and back['received']['submap_pose']==std['submaps']
 assert back['received'].get('verified_loops',0)==pipeline['verified'] and back['received'].get('inter2_triangles',0)==max(0,std['submaps']-2)
 stop('record',recorder)
 assert recorder.returncode==0,('recorder_shutdown',recorder.returncode)
 with rosbag.Bag(str(a.out/'backend_inputs.bag')) as b:
  info=b.get_type_and_topic_info().topics
  for topic,n in [('/submap_pose',std['submaps']),('/clouds_submap',std['submaps']),('/loop_pipeline/verified_loops',pipeline['verified'])]:
   assert (info[topic].message_count if topic in info else 0)==n,(topic,n)
 stop('backend',optimizer);assert optimizer.returncode==0
 subprocess.run([str(exe/'loop_optimization/rebuild_factor_archive'),str(a.out/'factor_transactions.jsonl'),str(a.out/'rebuilt.json')],check=True)
 saved=np.loadtxt(a.out/'optimized_poses.txt',ndmin=2);rebuilt=read(a.out/'rebuilt.json');reconstructed=np.array([r['pose'] for r in rebuilt['poses']])
 poserr=float(np.linalg.norm(saved[:,1:4]-reconstructed[:,:3],axis=1).max())
 roterr=max(float(np.linalg.norm(matrix(r)[:3,:3]-matrix(s[1:])[:3,:3])) for r,s in zip(reconstructed,saved))
 assert poserr<1e-6 and roterr<1e-6 and len(saved)==std['submaps']
 factors=[f for t in map(json.loads,(a.out/'factor_transactions.jsonl').read_text().splitlines()) for f in t['factors']]
 loops=[f for f in factors if f['loop_id']];assert len(loops)==len({f['loop_id'] for f in loops})==back['accepted']
 assert len(factors)==len(saved)+len(loops)==rebuilt['factors']
 state.update(phase='passed',submaps=std['submaps'],elapsed_s=time.monotonic()-start);save()
 dump(a.out/'result.json',dict(status='passed',frontend=front,std=std,pipeline=pipeline,backend=back,performance=state,
  raw_factors=len(factors),pose3_loops=len(loops),rebuild_position_difference_m=poserr,rebuild_rotation_difference=roterr,
  limitations=['Single run; no detector precision/recall without independent labels','Dog SC gravity validity gated; no dog ground truth accuracy claim']))
 (a.out/'STATUS').write_text('ONLINE_REPLAY_PASSED\n')
except Exception as e:
 state.update(phase='failed',error=str(e));save();(a.out/'STATUS').write_text('FAILED\n');traceback.print_exc();raise
finally:
 for name,proc in reversed(children):stop(name,proc)
 for f in files:f.close()
 frontend_process=[proc for name,proc in children if name=='frontend']
 if frontend_process and frontend_process[0].returncode!=0:
  state.update(phase='failed',error='frontend shutdown exit '+str(frontend_process[0].returncode));dump(a.out/'status.json',state)
  (a.out/'STATUS').write_text('FAILED\n')
  if (a.out/'result.json').exists():
   result=read(a.out/'result.json');result.update(status='failed',reason=state['error']);dump(a.out/'result.json',result)
  raise RuntimeError(state['error'])
