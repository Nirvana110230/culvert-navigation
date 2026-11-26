"""Known analytic motion and fault injection on an isolated ROS domain only."""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_msgs.msg import TFMessage


def stamp(t):
    s, ns = divmod(t,10**9)
    return Time(sec=s,nanosec=ns)


def make_state(t, x=0.):
    s=Odometry()
    s.header.frame_id='camera_init'; s.child_frame_id='imu'; s.header.stamp=stamp(t)
    s.pose.pose.position.x=x; s.pose.pose.orientation.w=1.
    return s


def make_scan(t, x=1.):
    s=PointCloud2()
    s.header.frame_id='livo_lidar'; s.header.stamp=stamp(t)
    s.width=1; s.height=1; s.is_dense=True; s.point_step=16; s.row_step=16
    s.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
              for i,name in enumerate(('x','y','z','intensity'))]
    s.data=struct.pack('<ffff',x,2.,3.,7.)
    return s


def make_imu(t):
    m=Imu();m.header.frame_id='livox_frame';m.header.stamp=stamp(t)
    m.angular_velocity.y=0.4;m.linear_acceleration.z=1.
    return m


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='84' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('需要 ROS_LOCALHOST_ONLY=1 和专用域 84')
    rclpy.init()
    node=rclpy.create_node('round1_synthetic_probe')
    data={}; publishers={}
    logdir=Path(sys.argv[1])
    def spin(sec):
        end=time.monotonic()+sec
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=min(.03,end-time.monotonic()))
    spin(1.3)
    assert set(node.get_node_names())=={node.get_name()}
    for topic,typ in [('/clock',Clock),('/culvert_source/state',Odometry),('/culvert_source/scan',PointCloud2),('/livox/imu',Imu)]:
        publishers[typ]=node.create_publisher(typ,topic,20)
    for topic,typ in [('/culvert_test/odom',Odometry),('/culvert_test/localization_cloud',PointCloud2),('/culvert_test/imu',Imu),('/tf',TFMessage),('/tf_static',TFMessage),('/culvert_test/diagnostics',DiagnosticArray),('/diagnostics',DiagnosticArray)]:
        data[topic]=[]
        profile=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic=='/tf_static' else 30
        node.create_subscription(typ,topic,lambda m,t=topic:data[t].append(m),profile)
    report=[]
    def ok(name,condition,detail=None):
        report.append({'case':name,'pass':bool(condition),'detail':detail}); print(report[-1],flush=True)
        assert condition,name
    def start(profile):
        f=(logdir/(profile+'.log')).open('w')
        proc=subprocess.Popen(['ros2','launch','culvert_livo_adapter','adapter.launch.py','profile:='+profile,'use_sim_time:=true'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        f.close()
        return proc
    def stop(proc):
        os.kill(proc.pid,signal.SIGINT)
        try: rc=proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5);raise
        ok('launch_exit',rc==0,rc)
    def send(m):
        publishers[type(m)].publish(m);spin(.025)
    def clock(t):
        send(Clock(clock=stamp(t)))
    odom,cloud,imu=data['/culvert_test/odom'],data['/culvert_test/localization_cloud'],data['/culvert_test/imu']
    diag=data['/culvert_test/diagnostics']
    def latest():
        return {item.key:item.value for item in diag[-1].status[0].values}
    t=20_000_000_000
    proc=start('isolated_test')
    try:
        for _ in range(100):
            spin(.06)
            if publishers[Imu].get_subscription_count() and data['/tf_static'] and diag:break
        ok('discovery',publishers[Imu].get_subscription_count()>0 and bool(diag))
        clock(t);send(make_scan(t));send(make_state(t));send(make_imu(t))
        ok('warmup_no_zero_twist',not odom and not cloud)
        t+=100_000_000;clock(t);send(make_state(t,.1));send(make_scan(t));send(make_imu(t))
        ok('normal_pose_velocity',len(odom)==1 and abs(odom[-1].pose.pose.position.x-.3)<1e-8 and abs(odom[-1].twist.twist.linear.x-1)<1e-8)
        coords=struct.unpack_from('<ffff',bytes(cloud[-1].data))
        ok('single_transform',len(cloud)==1 and max(abs(a-b) for a,b in zip(coords,[.789,1.97671,2.94412,7]))<2e-6,coords)
        ok('imu_units_reference',len(imu)>=1 and abs(imu[-1].linear_acceleration.z-9.80665)<1e-7 and imu[-1].orientation_covariance[0]==-1)
        edges=[(v.header.frame_id,v.child_frame_id) for key in ('/tf','/tf_static') for msg in data[key] for v in msg.transforms]
        ok('tf_owner_and_edges',set(edges)=={('test_odom','test_base_link'),('test_base_link','test_imu')} and node.count_publishers('/tf')==1,edges)
        send(make_scan(t));ok('duplicate_cloud',len(cloud)==1)
        send(make_scan(t-10));ok('backward_cloud',len(cloud)==1)
        t+=100_000_000;clock(t);send(make_scan(t));send(make_state(t,.2));ok('cloud_before_pose',len(cloud)==2)
        t+=100_000_000;clock(t);send(make_state(t,.3));ok('unequal_rates',len(odom)==3 and len(cloud)==2)
        t+=100_000_000;clock(t);send(make_scan(t));spin(.55)
        ok('unpaired_timeout',len(cloud)==2 and int(latest()['pair_expired'])>=1)
        t+=100_000_000;clock(t)
        s=make_state(t);s.header.frame_id='wrong';send(s)
        s=make_state(t);s.pose.pose.orientation.w=0.;send(s)
        s=make_state(t,float('nan'));send(s)
        bad=make_imu(t);bad.angular_velocity.x=float('inf');k=len(imu);send(bad)
        ok('frame_nan_zero_quaternion_inf',len(odom)==3 and len(imu)==k)
        send(make_state(t,.5));send(make_scan(t,float('nan')))
        ok('nan_cloud_rejected',len(cloud)==2)
        t+=1_500_000_000;clock(t);s=make_state(t-900_000_000,.6);old=len(odom);send(s)
        ok('past_stamp_rejected',len(odom)==old)
        spin(1.6);ok('clock_pause_wall_watchdog',latest()['time_status']=='simulation_clock_paused' and diag[-1].status[0].level in (2,b'\x02'))
        clock(t+10_000_000);send(make_state(t+10_000_000,.5));send(make_imu(t+10_000_000))
        ok('long_dropout_requires_baseline',len(odom)==old)
        t+=110_000_000;clock(t);send(make_scan(t));send(make_state(t,.6));send(make_imu(t));spin(.12)
        ok('resume_same_clock_epoch',len(odom)==old+1 and len(cloud)==3)
        # Clock runs while source stays silent, distinguish from /clock pause.
        for _ in range(15):
            t+=100_000_000;clock(t);spin(.075)
        ok('live_clock_input_gap',diag[-1].status[0].level in (2,b'\x02') and latest()['time_status']=='running')
        # Re-establish baseline after an advancing-clock outage, then inject an implausible step.
        t+=100_000_000;clock(t);send(make_state(t,.6));send(make_imu(t))
        t+=100_000_000;clock(t);send(make_state(t,.7));send(make_scan(t));send(make_imu(t))
        before=len(odom)
        t+=100_000_000;clock(t);send(make_state(t,100.));send(make_scan(t))
        spin(.56)
        ok('kinematic_jump_suppressed',len(odom)==before,latest()['last_reject'])
        counters=latest()
        ok('diagnosed_duplicate_and_rollback',int(counters['duplicate_count'])>=1 and int(counters['rollback_count'])>=1)
        ok('no_actuation_topics',all(n not in ('/cmd_vel','/map') for n,_ in node.get_topic_names_and_types()))
    finally:
        stop(proc)
    proc=start('formal')
    try:
        for _ in range(100):
            spin(.07)
            if data['/diagnostics']:break
        stat=data['/diagnostics'][-1].status[0]
        ok('formal_unmeasured_extrinsics',stat.level in (2,b'\x02') and 'extrinsic' in stat.message and node.count_publishers('/culvert/odom')==0 and node.count_publishers('/tf')==0,stat.message)
    finally:
        stop(proc)
    spin(1.)
    ok('no_remaining_adapter',set(node.get_node_names())=={node.get_name()})
    (logdir/'synthetic.json').write_text(json.dumps(report,indent=2))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
