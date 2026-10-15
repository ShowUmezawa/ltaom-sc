#!/usr/bin/env python3
"""Read original split ROS2 bag through ros2 bag play and native frontend topics."""
import argparse,json,os,signal,subprocess,time,traceback
from pathlib import Path
import numpy as np
import yaml,rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from flow_control import FlowController
from dataset_profile import ImuRelay, ResourceMonitor, ros2_parameter_types
from sensor_profile import add_profile_arguments, load_profile, frontend_parameters, check_bag_topics, snapshot_profile
p=argparse.ArgumentParser();add_profile_arguments(p);p.add_argument('--bag',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--rebuild-mode',choices=['sync','async'],default='sync');p.add_argument('--probe-pause-at',type=float);p.add_argument('--params-override',type=Path);a=p.parse_args()
repo=Path(__file__).resolve().parents[1]
dataset=load_profile(repo,'ros2',a.dataset,a.sensor_config,False)
a.dataset=dataset['name']
config=frontend_parameters(dataset,a.out/'frontend',a.rebuild_mode,a.params_override)
a.out.mkdir(parents=True,exist_ok=False)
snapshot_profile(dataset,a.out)
repo=Path(__file__).resolve().parents[1];build=Path(os.environ.get('LTAOM_BUILD_DIR',str(repo/'build')))
from run_metadata import record
record(repo,a.out,a)
os.environ['ROS_LOG_DIR']=str(a.out/'ros');os.environ.setdefault('ROS_DOMAIN_ID','72');os.environ.setdefault('ROS_LOCALHOST_ONLY','1')
metadata=yaml.safe_load((a.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information'];topic_counts={t['topic_metadata']['name']:t['message_count'] for t in metadata['topics_with_message_count']}
check_bag_topics(dataset,{t['topic_metadata']['name']:t['topic_metadata']['type'] for t in metadata['topics_with_message_count']},'ros2')
lidar=dataset['lidar'];imu=dataset['imu'];bag_imu=dataset['bag_imu'];expected_lidar=topic_counts[lidar];expected_imu=topic_counts[bag_imu];duration=metadata['duration']['nanoseconds']/1e9
config=ros2_parameter_types(config)
(a.out/'params.yaml').write_text(yaml.safe_dump({'laserMapping':{'ros__parameters':config}}))
(a.out/'qos.yaml').write_text(yaml.safe_dump({t:dict(history='keep_last',depth=(256 if t==lidar else 8192),reliability='reliable',durability='volatile') for t in [lidar,bag_imu]}))
children=[];files=[];state=dict(phase='starting',poses=0,rate=1,dataset=dataset['description']+'; frontend only')
monitor=ResourceMonitor(a.out,children)
def save():
 monitor.sample()
 temp=a.out/'status.tmp';temp.write_text(json.dumps(dict(state,updated_unix=time.time()),indent=2));temp.replace(a.out/'status.json')
def spawn(cmd,name):
 f=(a.out/(name+'.log')).open('w');files.append(f);proc=subprocess.Popen(list(map(str,cmd)),stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(proc);return proc
front=spawn([str(build/'install/ltaom_frontend/lib/ltaom_frontend/fastlio_node'),'--ros-args','--params-file',a.out/'params.yaml'],'frontend')
rclpy.init();node=rclpy.create_node('replay_runner');relay=ImuRelay(node,a.out,dataset);observed=[]
play=None
flow=FlowController(node,a.out,{'frontend':a.out/'frontend/queue_status.json'},probe_pause_at=a.probe_pause_at)
def receive(m):observed.append(m);state['poses']=len(observed)
sub=node.create_subscription(Odometry,'/Odometry',receive,10000)
def wait(fn,why,timeout=60):
 state['phase']=why;end=time.monotonic()+timeout;next_save=0
 while not fn():
  if front.poll() is not None:raise RuntimeError('frontend exited')
  if time.monotonic()>end:raise RuntimeError('timeout '+why)
  rclpy.spin_once(node,timeout_sec=.02)
  flow.tick(playing=play is not None and play.poll() is None)
  if time.monotonic()>next_save:save();next_save=time.monotonic()+1
try:
 wait(lambda:node.count_subscribers(lidar)>0 and node.count_subscribers(imu)>0 and node.count_subscribers(bag_imu)>0,'sensor_subscribers')
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
 d=np.linalg.norm(np.diff(traj[:,1:4],axis=0),axis=1);dt=np.diff(traj[:,0]);result=dict(status='passed',kind='native ROS2 frontend only',frontend=counts,
  duration_s=float(traj[-1,0]-traj[0,0]),path_length_m=float(d.sum()),max_speed_m_s=float((d/dt).max()),max_dt_s=float(dt.max()),z_span_m=float(np.ptp(traj[:,3])),wall_s=time.monotonic()-start,
  limitation='Development clip; cross-ROS numerical parity and independent loop labels pending',dataset=dataset,ikdtree_rebuild_mode=a.rebuild_mode)
 reference=Path('/reference/trajectory.tum')
 if reference.exists():
  ref=np.loadtxt(reference);mask=(traj[:,0]>=ref[0,0])&(traj[:,0]<=ref[-1,0]);pred=np.column_stack([np.interp(traj[mask,0],ref[:,0],ref[:,k]) for k in [1,2,3]]);e=np.linalg.norm(pred-traj[mask,1:4],axis=1)
  result['ros1_frontend_position_difference']=dict(reference=str(reference),count=int(mask.sum()),rmse_m=float(np.sqrt(np.mean(e*e))),max_m=float(e.max()),qualification='Same raw initial frame; no alignment; interpolation; not a GT error')
 (a.out/'result.json').write_text(json.dumps(result,indent=2)+'\n');(a.out/'STATUS').write_text('ROS2_FRONTEND_REAL_BAG_PASSED\n');state.update(phase='passed',**result);save();print(json.dumps(result,indent=2),flush=True)
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
if front.returncode!=0 or 'Error in destruction' in text or 'Failed to delete data' in text:
 result.update(status='failed',reason='frontend_shutdown',returncode=front.returncode)
 (a.out/'result.json').write_text(json.dumps(result,indent=2)+'\n');(a.out/'STATUS').write_text('FAILED\n')
 raise RuntimeError('unclean frontend shutdown')
