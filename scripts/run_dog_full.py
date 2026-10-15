#!/usr/bin/env python3
"""Read original split ROS2 bag through ros2 bag play and native frontend topics."""
import argparse,json,os,signal,subprocess,time,traceback
from pathlib import Path
import numpy as np
import yaml,rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from nav_msgs.msg import Odometry
from ltaom_interfaces.msg import Submap
from std_msgs.msg import String
from submap_hash import submap_hash,cloud_xyzi
from std_srvs.srv import Trigger
from flow_control import FlowController
from dataset_profile import ImuRelay, ResourceMonitor, ros2_parameter_types
from sensor_profile import add_profile_arguments, load_profile, frontend_parameters, check_bag_topics, snapshot_profile
p=argparse.ArgumentParser();add_profile_arguments(p);p.add_argument('--bag',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--mode',choices=['std_only','sc_only','dual'],default='dual');p.add_argument('--rebuild-mode',choices=['sync','async'],default='sync');p.add_argument('--probe-pause-at',type=float);p.add_argument('--params-override',type=Path);a=p.parse_args()
repo=Path(__file__).resolve().parents[1]
dataset=load_profile(repo,'ros2',a.dataset,a.sensor_config,True)
a.dataset=dataset['name']
config=frontend_parameters(dataset,a.out/'frontend',a.rebuild_mode,a.params_override)
a.out.mkdir(parents=True,exist_ok=False)
snapshot_profile(dataset,a.out);session=a.out.name
repo=Path(__file__).resolve().parents[1];build=Path(os.environ.get('LTAOM_BUILD_DIR',str(repo/'build')))
from run_metadata import record
record(repo,a.out,a)
os.environ['ROS_LOG_DIR']=str(a.out/'ros');os.environ.setdefault('ROS_DOMAIN_ID','72');os.environ.setdefault('ROS_LOCALHOST_ONLY','1')
metadata=yaml.safe_load((a.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information'];topic_counts={t['topic_metadata']['name']:t['message_count'] for t in metadata['topics_with_message_count']}
check_bag_topics(dataset,{t['topic_metadata']['name']:t['topic_metadata']['type'] for t in metadata['topics_with_message_count']},'ros2')
lidar=dataset['lidar'];imu=dataset['imu'];bag_imu=dataset['bag_imu'];expected_lidar=topic_counts[lidar];expected_imu=topic_counts[bag_imu];duration=metadata['duration']['nanoseconds']/1e9
(a.out/'std').mkdir()
std_config=dict(SaveDir=str(a.out/'std')+'/',lcd_config_path=dataset['std'],submap_credit_window=8,candidate_only=True,flush_tail_submap=True,session_id=session,multisession_mode=0,export_loop_candidates=a.mode!='sc_only',std_retrieval_en=a.mode!='sc_only')
(a.out/'std_params.yaml').write_text(yaml.safe_dump({'ltaom_std':{'ros__parameters':std_config}}))
config=ros2_parameter_types(config)
(a.out/'params.yaml').write_text(yaml.safe_dump({'laserMapping':{'ros__parameters':config}}))
(a.out/'qos.yaml').write_text(yaml.safe_dump({t:dict(history='keep_last',depth=(256 if t==lidar else 8192),reliability='reliable',durability='volatile') for t in [lidar,bag_imu]}))
children=[];files=[];state=dict(phase='starting',poses=0,rate=1,dataset=dataset['description']+'; full native pipeline')
monitor=ResourceMonitor(a.out,children)
def save():
 monitor.sample()
 obs_path=a.out/'observer/status.json'
 if obs_path.exists():
  obs=json.loads(obs_path.read_text());state.update(submaps=obs['submaps'],observed_batches=obs['batches'],observer=obs)
 temp=a.out/'status.tmp';temp.write_text(json.dumps(dict(state,updated_unix=time.time()),indent=2));temp.replace(a.out/'status.json')
def spawn(cmd,name):
 f=(a.out/(name+'.log')).open('w');files.append(f);proc=subprocess.Popen(list(map(str,cmd)),stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(proc);return proc
observer=spawn(['python3',Path(__file__).with_name('submap_observer.py'),'--out',a.out/'observer','--session',session],'observer')
observer_deadline=time.monotonic()+30
while not (a.out/'observer/status.json').exists():
 if observer.poll() is not None or time.monotonic()>observer_deadline:raise RuntimeError('observer failed before algorithm startup')
 time.sleep(.05)
front=spawn([str(build/'install/ltaom_frontend/lib/ltaom_frontend/fastlio_node'),'--ros-args','--params-file',a.out/'params.yaml'],'frontend')
std=spawn([str(build/'install/ltaom_std/lib/ltaom_std/std_node'),'--ros-args','--params-file',a.out/'std_params.yaml'],'std')
backend_params=dict(output_dir=str(a.out/'backend'),session_id=session,map_stride=1)
(a.out/'backend_params.yaml').write_text(yaml.safe_dump({'ltaom_backend':{'ros__parameters':backend_params}}))
back=spawn([str(build/'install/ltaom_backend/lib/ltaom_backend/backend_node'),'--ros-args','--params-file',a.out/'backend_params.yaml'],'backend')
pipeline=spawn([str(build/'install/ltaom_pipeline/lib/ltaom_pipeline/live_pipeline'),'--out',a.out/'pipeline','--bin-dir',str(build/'install/ltaom_loop_core/lib/ltaom_loop_core'),'--config',dataset['geometry'],'--scan-manifest',a.out/'frontend/scan_manifest.csv','--backend-status',a.out/'backend/backend_status.json','--session',session,'--mode',a.mode,'--gap',dataset['gap'],'--seconds',dataset['seconds'],'--top-k',dataset['top_k'],'--sc-threshold',dataset['sc_threshold']],'pipeline')
rclpy.init();node=rclpy.create_node('replay_runner');relay=ImuRelay(node,a.out,dataset);observed=[]
play=None
flow=FlowController(node,a.out,{stage:a.out/stage/'queue_status.json' for stage in ['frontend','std','pipeline','backend']},probe_pause_at=a.probe_pause_at)
def receive(m):observed.append(m);state['poses']=len(observed)
sub=node.create_subscription(Odometry,'/Odometry',receive,10000)
def observer_status():
 return json.loads((a.out/'observer/status.json').read_text())
def wait(fn,why,timeout=60):
 state['phase']=why;end=time.monotonic()+timeout;next_save=0
 while not fn():
  if any(p.poll() is not None for p in [front,std,back,pipeline,observer]):raise RuntimeError('frontend, STD, backend or pipeline exited')
  obs=observer_status()
  if obs['fatal']:raise RuntimeError('observer: '+obs['fatal'])
  if time.monotonic()>end:raise RuntimeError('timeout '+why+'; observer='+json.dumps(obs))
  rclpy.spin_once(node,timeout_sec=.02)
  flow.tick(playing=play is not None and play.poll() is None)
  if time.monotonic()>next_save:save();next_save=time.monotonic()+1
try:
 wait(lambda:(a.out/'pipeline/status.json').exists() and json.loads((a.out/'pipeline/status.json').read_text()).get('phase')=='ready','pipeline_ready')
 wait(lambda:node.count_subscribers('/loop_pipeline/submaps')>=3 and node.count_subscribers('/loop_pipeline/submap_records')>0 and node.count_subscribers('/loop_pipeline/verified_loops')>0,'backend_subscribers')
 wait(lambda:node.count_subscribers(lidar)>0 and node.count_subscribers(imu)>0 and node.count_subscribers(bag_imu)>0,'sensor_subscribers')
 wait(lambda:observer_status()['submap_publishers']==1,'observer_publisher_discovery')
 wait(lambda:node.count_subscribers('/cloud_registered')>0 and node.count_subscribers('/aft_mapped_to_init')>0,'std_subscribers')
 start=time.monotonic();play=spawn(['ros2','bag','play',str(a.bag),'--rate','1','--delay','2','--wait-for-all-acked','60000','--topics',lidar,bag_imu,'--qos-profile-overrides-path',a.out/'qos.yaml'],'play')
 wait(flow.ready,'player_flow_services',30)
 wait(lambda:play.poll() is not None,'playing',duration+900);assert play.returncode==0
 flow.record('playback_finished')
 params=node.create_client(SetParameters,'/laserMapping/set_parameters');wait(params.service_is_ready,'parameter_service')
 request=SetParameters.Request();request.parameters=[p.to_parameter_msg() for p in [Parameter('expected_lidar_messages',value=expected_lidar),Parameter('expected_imu_messages',value=expected_imu)]];future=params.call_async(request);wait(future.done,'set_counts');assert all(r.successful for r in future.result().results)
 client=node.create_client(Trigger,'/frontend/drain');wait(client.service_is_ready,'drain_ready')
 deadline=time.monotonic()+120
 while True:
  future=client.call_async(Trigger.Request());wait(future.done,'frontend_drain');response=future.result()
  state['drain_response']=response.message;save()
  if response.success:break
  if time.monotonic()>deadline:raise RuntimeError('counted EOF timeout')
  rclpy.spin_once(node,timeout_sec=.1)
 relay.check(expected_imu)
 counts=json.loads((a.out/'frontend/frontend_status.json').read_text());assert counts['received_lidar']==expected_lidar and counts['received_imu']==expected_imu and counts['invalid_lidar']==0
 assert counts['consumed_lidar']+counts['uncovered_tail']==expected_lidar
 wait(lambda:len(observed)==counts['output_scans'],'observed_count')
 traj=np.loadtxt(a.out/'frontend/trajectory.tum');assert np.isfinite(traj).all() and np.all(np.diff(traj[:,0])>0) and len(traj)==counts['output_scans']
 params_std=node.create_client(SetParameters,'/ltaom_std/set_parameters');wait(params_std.service_is_ready,'std_parameters')
 request=SetParameters.Request();request.parameters=[Parameter('std_expected_scans',value=counts['output_scans']).to_parameter_msg()];future=params_std.call_async(request);wait(future.done,'std_set_count');assert all(r.successful for r in future.result().results)
 client_std=node.create_client(Trigger,'/std/drain');wait(client_std.service_is_ready,'std_drain_ready')
 deadline=time.monotonic()+120
 while True:
  future=client_std.call_async(Trigger.Request());wait(future.done,'std_drain');response=future.result();state['std_drain_response']=response.message;save()
  if response.success:break
  if time.monotonic()>deadline:raise RuntimeError('STD counted EOF timeout')
  rclpy.spin_once(node,timeout_sec=.1)
 std_counts=json.loads((a.out/'std/std_status.json').read_text());assert std_counts['consumed_scans']==counts['output_scans']
 wait(lambda:observer_status()['submaps']==std_counts['submaps'],'observed_submaps')
 submaps=[json.loads(line) for line in (a.out/'observer/submaps.jsonl').read_text().splitlines()]
 assert submaps[-1]['last_scan_id']==counts['output_scans']-1
 if a.mode!='sc_only':
  wait(lambda:observer_status()['batches']==len(submaps),'candidate_batches')
  batches=[json.loads(line) for line in (a.out/'observer/candidates.jsonl').read_text().splitlines()]
  assert [b['current_id'] for b in batches]==list(range(len(submaps)))
 else:
  batches=[json.loads(line) for line in (a.out/'observer/candidates.jsonl').read_text().splitlines()]
  assert not batches
 (a.out/'observed_submaps.json').write_text(json.dumps(submaps,indent=2)+'\n')
 (a.out/'observed_candidates.json').write_text(json.dumps(batches,indent=2)+'\n')
 def drain_stage(name,service,parameters):
  client=node.create_client(SetParameters,'/'+name+'/set_parameters');wait(client.service_is_ready,name+'_parameter_service')
  req=SetParameters.Request();req.parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in parameters.items()];future=client.call_async(req);wait(future.done,name+'_set_counts');assert all(r.successful for r in future.result().results)
  client=node.create_client(Trigger,service);wait(client.service_is_ready,service+'_ready');deadline=time.monotonic()+240
  while True:
   future=client.call_async(Trigger.Request());wait(future.done,service);r=future.result();state[name+'_drain_response']=r.message;save()
   if r.success:break
   if time.monotonic()>deadline:raise RuntimeError('timeout '+service+': '+r.message)
   rclpy.spin_once(node,timeout_sec=.05)
 drain_stage('loop_pipeline','/loop_pipeline/drain',dict(expected_submaps=std_counts['submaps']))
 pipe=json.loads((a.out/'pipeline/summary.json').read_text());assert pipe['submaps']==std_counts['submaps']
 drain_stage('ltaom_backend','/loop_optimizer/drain',dict(expected_submaps=std_counts['submaps'],expected_verified_loops=pipe['verified']))
 back_status=json.loads((a.out/'backend/backend_status.json').read_text());assert not back_status['fatal'] and back_status['invalid_verified_loops']==0 and back_status['accepted']+back_status['rejected']==pipe['verified']
 subprocess.run([str(build/'install/ltaom_backend/lib/ltaom_backend/rebuild_factor_archive'),str(a.out/'backend/factor_transactions.jsonl'),str(a.out/'rebuilt.json')],check=True,timeout=120)
 rebuilt=json.loads((a.out/'rebuilt.json').read_text());optimized=np.loadtxt(a.out/'backend/optimized_poses.txt',ndmin=2);poses=np.array([r['pose'] for r in rebuilt['poses']]);assert len(poses)==std_counts['submaps'] and np.max(np.abs(poses-optimized[:,1:]))<1e-6
 assert rebuilt['factors']==std_counts['submaps']+back_status['accepted']
 from ltaom_pipeline.pipeline_core import matrix
 def pcd(path):
  head,body=path.read_bytes().split(b'DATA binary\n',1)
  if b'FIELDS x y z intensity\n' not in head or b'SIZE 4 4 4 4\n' not in head:raise ValueError('unexpected map PCD layout')
  return np.frombuffer(body,dtype='<f4').reshape(-1,4)
 actual=pcd(a.out/'backend/cloud_result.pcd');records=list(map(json.loads,(a.out/'pipeline/archive/submaps.jsonl').read_text().splitlines()));offset=0;map_error=0.
 for i,r in enumerate(records):
  points=pcd(a.out/'pipeline/archive'/r['world_cloud_path']);T=matrix(optimized[i,1:])@np.linalg.inv(matrix(r['raw_anchor_pose']));expected=points[:,:3].astype(float)@T[:3,:3].T+T[:3,3]
  part=actual[offset:offset+len(points)];assert len(part)==len(points) and np.isfinite(part).all();map_error=max(map_error,float(np.linalg.norm(expected-part[:,:3],axis=1).max()));offset+=len(points)
 assert offset==len(actual) and map_error<.001,(offset,len(actual),map_error)
 d=np.linalg.norm(np.diff(traj[:,1:4],axis=0),axis=1);dt=np.diff(traj[:,0]);result=dict(status='passed',kind='native ROS2 frontend, STD, shared SC/geometry pipeline and iSAM2 backend',pipeline=pipe,backend=back_status,rebuilt_factors=rebuilt['factors'],map_export_max_difference_m=map_error,std=std_counts,mode=a.mode,candidate_batches=len(batches),candidate_pairs=sum(len(b['candidates']) for b in batches),frontend=counts,
  duration_s=float(traj[-1,0]-traj[0,0]),path_length_m=float(d.sum()),max_speed_m_s=float((d/dt).max()),max_dt_s=float(dt.max()),z_span_m=float(np.ptp(traj[:,3])),wall_s=time.monotonic()-start,
  limitation='Development clip; cross-ROS numerical parity and independent loop labels pending',dataset=dataset,ikdtree_rebuild_mode=a.rebuild_mode)
 reference=Path('/reference/trajectory.tum')
 if reference.exists():
  ref=np.loadtxt(reference);mask=(traj[:,0]>=ref[0,0])&(traj[:,0]<=ref[-1,0]);pred=np.column_stack([np.interp(traj[mask,0],ref[:,0],ref[:,k]) for k in [1,2,3]]);e=np.linalg.norm(pred-traj[mask,1:4],axis=1)
  result['ros1_frontend_position_difference']=dict(reference=str(reference),count=int(mask.sum()),rmse_m=float(np.sqrt(np.mean(e*e))),max_m=float(e.max()),qualification='Same raw initial frame; no alignment; interpolation; not a GT error')
 (a.out/'result.json').write_text(json.dumps(result,indent=2)+'\n');(a.out/'STATUS').write_text('ROS2_FULL_'+a.dataset.upper()+'_PASSED\n');state.update(phase='passed',**result);save();print(json.dumps(result,indent=2),flush=True)
except Exception as e:
 state.update(phase='failed',error=str(e));save();(a.out/'STATUS').write_text('FAILED\n');traceback.print_exc();raise
finally:
 node.destroy_node();rclpy.shutdown()
 for proc in reversed(children):
  if proc.poll() is None:
   os.killpg(proc.pid,signal.SIGINT)
   try:proc.wait(timeout=25)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 for f in files:f.close()

text=(a.out/'frontend.log').read_text()
text+=(a.out/'std.log').read_text()
if front.returncode!=0 or std.returncode!=0 or back.returncode!=0 or pipeline.returncode!=0 or observer.returncode!=0 or 'Error in destruction' in text or 'Failed to delete data' in text:
 result.update(status='failed',reason='frontend_shutdown',returncode=front.returncode)
 (a.out/'result.json').write_text(json.dumps(result,indent=2)+'\n');(a.out/'STATUS').write_text('FAILED\n')
 raise RuntimeError('unclean frontend shutdown')
