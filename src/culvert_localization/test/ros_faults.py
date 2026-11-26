#!/usr/bin/env python3
"""Real ROS2 transport, services, measurement clock and shutdown with a deterministic worker."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger


def rostime(t):
    sec,nano = divmod(t,10**9)
    return Time(sec=sec,nanosec=nano)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('localizer', type=Path)
    ap.add_argument('output', type=Path)
    args = ap.parse_args()
    if not os.environ.get('ROS_DOMAIN_ID','').isdigit() or not 85 <= int(os.environ['ROS_DOMAIN_ID']) <= 100 or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Use an isolated localhost ROS domain between 85 and 100')
    d = args.output.resolve()
    d.mkdir(parents=True,exist_ok=False)
    os.environ['CULVERT_FAULT_DIR'] = str(d)
    os.environ['OPENBLAS_NUM_THREADS'] = '1'
    os.environ['ROS_LOG_DIR'] = str(d/'roslog')
    rclpy.init()
    node = rclpy.create_node('culvert_r2_fault_probe')
    prefix = '/culvert_test/localization/'
    diag, events, candidates, corrections = [], [], [], []
    node.create_subscription(String,prefix+'decision',lambda m: events.append(json.loads(m.data)),64)
    node.create_subscription(DiagnosticArray,prefix+'diagnostics',lambda m: diag.append(m.status[0].message),10)
    node.create_subscription(PoseStamped,prefix+'candidate',candidates.append,5)
    node.create_subscription(TransformStamped,prefix+'correction',corrections.append,5)
    clock_pub = node.create_publisher(Clock,'/clock',20)
    pubs = {m:(node.create_publisher(Odometry,p,16),node.create_publisher(PointCloud2,c,16))
            for m,p,c in [('A','/culvert_source/state','/culvert_source/scan'),
                          ('B','/culvert_test/odom','/culvert_test/localization_cloud')]}
    reload_cli = node.create_client(Trigger,prefix+'reload_map')
    params_cli = node.create_client(SetParameters,'/culvert_localization/set_parameters')
    I = np.eye(4)
    extrinsic = I.copy()
    extrinsic[0,3] = .4
    seed = I.copy()
    seed[2,3] = 2.
    map_file = d/'fixture.bin'
    map_file.write_bytes(b'ROS transport fixture only')
    manifest = dict(path=str(map_file),id='fixture',version='1',frame='test_map',units='m',
                    gravity='synthetic',origin='synthetic',sha256=hashlib.sha256(map_file.read_bytes()).hexdigest())
    manifest_path = d/'manifest.json'
    def write_map():
        manifest_path.write_text(json.dumps(manifest))
    write_map()
    def fault(**kw):
        tmp = d/'pending_fault'
        tmp.write_text(json.dumps(kw))
        tmp.replace(d/'fault.json')
    fault()
    fault_exe = d/'fault_server'
    shutil.copyfile(Path(__file__).with_name('fault_server.py'),fault_exe)
    fault_exe.chmod(0o755)
    checks = []
    def check(name,truth,detail=''):
        checks.append(dict(case=name,passed=bool(truth),detail=detail))
        (d/'assertions.json').write_text(json.dumps(checks,indent=2))
        print(name,truth,detail,flush=True)
        assert truth,name
    def spin(sec):
        end = time.monotonic()+sec
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=min(.015,max(0.,end-time.monotonic())))
    def until(pred,timeout=3.):
        end = time.monotonic()+timeout
        while not pred() and time.monotonic()<end:
            spin(.015)
        return bool(pred())
    def event_from(n,reason=None,kind=None):
        return next((r for r in events[n:] if (not reason or r['reason']==reason) and (not kind or r['kind']==kind)),None)
    def seen(n,reason,kind=None):
        return until(lambda: event_from(n,reason,kind))
    spin(.5)
    check('isolated_domain',set(node.get_node_names()) == {node.get_name()})
    proc = None
    mode = 'A'
    def start(name, **options):
        nonlocal proc,mode
        mode = options.get('mode','A')
        cfg = dict(mode=mode, backend='icp', map_manifest=str(manifest_path),use_sim_time=True,
                   initial_pose_json=json.dumps(seed.tolist()), imu_from_lidar_json=json.dumps(extrinsic.tolist()),
                   pair_timeout_s=.14, stream_timeout_s=.65, max_stamp_age_s=.5, max_future_s=.03,
                   queue_limit=3, match_hz=10., budget_ms=180., server_path=str(fault_exe),
                   save_inputs=True, record_path=str(d/(name+'.jsonl')))
        cfg.update(options)
        cf = d/(name+'.yaml')
        cf.write_text(yaml.safe_dump({'culvert_localization':{'ros__parameters':cfg}}))
        with (d/(name+'.log')).open('w') as f:
            proc = subprocess.Popen([str(args.localizer.resolve()),'--ros-args','--params-file',str(cf)],
                                    stdout=f,stderr=subprocess.STDOUT)
        n = len(diag)
        if not until(lambda: len(diag)>n and pubs[mode][1].get_subscription_count()>0,8.):
            raise RuntimeError((d/(name+'.log')).read_text())
        if cfg['map_manifest']:
            assert until(lambda: '"kind":"map","reason":"loaded"' in Path(cfg['record_path']).read_text())
    def stop(name):
        nonlocal proc
        proc.send_signal(signal.SIGINT)
        code = proc.wait(timeout=5)
        proc = None
        check(name+'_clean_exit',code==0,code)
        until(lambda: 'culvert_localization' not in node.get_node_names())
    t=25_000_000_000
    def clock(value):
        clock_pub.publish(Clock(clock=rostime(value)))
        spin(.025)
    def step(ns=22_000_000):
        nonlocal t
        t += ns
        clock(t)
    def pose(value=None,frame=None,orientation=1.,x=0.):
        s=Odometry()
        s.header.frame_id=frame or ('camera_init' if mode=='A' else 'test_odom')
        s.child_frame_id='imu' if mode=='A' else 'test_base_link'
        s.header.stamp=rostime(t if value is None else value)
        s.pose.pose.orientation.w=orientation
        s.pose.pose.position.x=x
        pubs[mode][0].publish(s)
    def scan(value=None,x=None):
        s=PointCloud2()
        s.header.frame_id='livo_lidar' if mode=='A' else 'test_base_link'
        s.header.stamp=rostime(t if value is None else value)
        s.width,s.height,s.point_step,s.row_step=40,1,12,480
        s.fields=[PointField(name=k,offset=4*i,count=1,datatype=7) for i,k in enumerate('xyz')]
        coords=np.arange(120,dtype='<f4')
        if x is not None:
            coords[0]=x
        s.data=coords.tobytes()
        pubs[mode][1].publish(s)
    def pair(scan_first=False):
        step()
        f,g = (scan,pose) if scan_first else (pose,scan)
        f()
        spin(.01)
        g()
    def run_case(name,send,reason,kind=None):
        n=len(events)
        send()
        check(name,seen(n,reason,kind),reason)
        return event_from(n,reason,kind)
    def reload_map():
        fut=reload_cli.call_async(Trigger.Request())
        assert until(fut.done) and fut.result().success
    def set_param(name,value):
        req=SetParameters.Request()
        req.parameters=[Parameter(name=name,value=ParameterValue(type=ParameterType.PARAMETER_STRING,string_value=value))]
        fut=params_cli.call_async(req)
        assert until(fut.done) and fut.result().results[0].successful
    try:
        start('transport')
        clock(t)
        r=run_case('scan_before_pose',lambda: pair(True),'accepted','match')
        check('A_single_extrinsic',abs(r['local'][0][3]-.4)<1e-9 and abs(r['result']['T_map_W'][0][3]+.4)<1e-9)
        check('save_inputs',np.load(d/r['input_path'])['points'].shape==(40,3))
        run_case('pose_before_scan',pair,'accepted','match')
        expected=rostime(t)
        both_arrived=until(lambda:
            any(m.header.stamp==expected and m.header.frame_id==manifest['frame'] for m in candidates)
            and any(m.header.stamp==expected and m.header.frame_id==manifest['frame'] and
                    m.child_frame_id=='camera_init' for m in corrections))
        def observed(m):
            return [m.header.stamp.sec*10**9+m.header.stamp.nanosec, m.header.frame_id,
                    getattr(m,'child_frame_id',None)]
        check('candidate_measurement_stamp',both_arrived,
              dict(expected_ns=t, candidates=[observed(m) for m in candidates],
                   corrections=[observed(m) for m in corrections]))
        run_case('duplicate',scan,'duplicate')
        run_case('out_of_order',lambda: pose(t-1),'out_of_order')
        run_case('wrong_frame',lambda: pose(t+1,frame='wrong'),'frame_mismatch')
        run_case('invalid_quaternion',lambda: pose(t+1,orientation=0.),'invalid_quaternion')
        run_case('nonfinite_geometry',lambda: scan(t+1,x=float('nan')),'nonfinite_cloud')
        run_case('stale',lambda: scan(t-1_000_000_000),'stale_stamp')
        run_case('future',lambda: scan(t+100_000_000),'future_stamp')
        run_case('zero',lambda: scan(0),'zero_stamp')
        def no_pair():
            step()
            pose(t)
            scan(t+1)
        run_case('nanosecond_mismatch',no_pair,'pair_timeout')
        def flood():
            step()
            for i in range(7):
                scan(t+i)
                spin(.006)
        run_case('bounded_pair_queue',flood,'queue_overflow')
        step()
        n=len(diag)
        for _ in range(17):
            step()
            spin(.025)
        check('inputs_stopped_with_clock_advancing','input_stopped' in diag[n:])
        n=len(events)
        check('paused_clock',seen(n,'simulation_clock_paused','clock'))
        fault(delay_s=.30)
        run_case('late_backend_over_budget',pair,'match_timeout','match')
        fault(override={'generation':999})
        run_case('backend_identity_mismatch',pair,'result_identity_mismatch','match')
        fault(override={'T_map_W':np.zeros((4,4)).tolist()})
        run_case('reject_invalid_backend_SE3',pair,'invalid_result_invalid_SE3','match')
        fault(delay_s=.26)
        n=len(events)
        pair()
        assert seen(n,'match','submit')
        reload_map()
        check('late_reply_discarded_after_reload',seen(n,'late_result'))
        check('map_generation_changes',seen(n,'loaded') and event_from(n,'loaded')['generation']==2)
        fault()
        run_case('resume_after_same_map_reload',pair,'accepted','match')
        manifest['version']='2'
        write_map()
        n=len(events)
        reload_map()
        assert seen(n,'loaded')
        run_case('old_seed_blocked_in_new_map',pair,'initial_map_mismatch')
        bound=json.dumps(dict(map_id=manifest['id'],version=manifest['version'],sha256=manifest['sha256'],matrix=seed.tolist()))
        set_param('initial_pose_json',bound)
        n=len(events)
        reload_map()
        assert seen(n,'loaded')
        run_case('explicit_new_map_anchor',pair,'accepted','match')
        valid_sha=manifest['sha256']
        manifest['sha256']='f'*64
        write_map()
        n=len(events)
        reload_map()
        check('map_hash_load_failure',seen(n,'map_load_failed','map'))
        run_case('old_map_blocked_on_load_failure',pair,'map_load_failed','drop')
        manifest['sha256']=valid_sha
        write_map()
        n=len(events)
        reload_map()
        assert seen(n,'loaded')
        fault(delay_s=.52)
        n=len(events)
        pair()
        assert seen(n,'match','submit')
        check('hung_worker_deadline_while_running',seen(n,'match_timeout','deadline'))
        count_before=len(candidates)
        step()
        scan(t)
        check('ROS_responds_while_worker_busy',seen(n,'pair_timeout'))
        check('overdue_result_naturally_completed',seen(n,'match_timeout','match') and len(candidates)==count_before)
        fault()
        run_case('same_worker_next_match',pair,'accepted','match')
        fault(crash=True)
        run_case('worker_process_exit',pair,'worker_failure','worker_error')
        fault()
        run_case('worker_failure_requires_reload',pair,'worker_requires_reload','drop')
        n=len(events)
        reload_map()
        assert seen(n,'loaded')
        run_case('reload_worker_recovery',pair,'accepted','match')
        n=len(events)
        clock(t-2_000_000_000)
        check('clock_rollback_latched',seen(n,'clock_rollback_restart_required','clock'))
        run_case('rollback_requires_restart',pair,'clock_rollback_restart_required','input_reject')
        stop('transport')
        start('no_map',map_manifest='')
        run_case('map_absent',pair,'map_not_configured','drop')
        stop('no_map')
        start('no_seed',initial_pose_json='')
        run_case('seed_absent',pair,'no_initial_pose','drop')
        stop('no_seed')
        start('ambiguous',alternative_poses_json=json.dumps([I.tolist()]))
        run_case('ambiguous_external_priors',pair,'ambiguous_initial_hypotheses','drop')
        stop('ambiguous')
        start('B',mode='B',imu_from_lidar_json='',match_hz=2.,budget_ms=500.)
        r=run_case('B_accept',pair,'accepted','match')
        check('B_no_extrinsic_reapplication',np.array_equal(r['local'],I) and r['source_frame']=='test_base_link')
        n=len(events)
        for _ in range(30):
            pair()
            spin(.015)
        spin(.12)
        submitted = [e for e in events[n:] if e['kind']=='submit']
        gaps = np.diff([e['monotonic_ns'] for e in submitted])/1e9
        check('latest_only_and_two_hz',bool(event_from(n,'newer_complete_pair','drop')) and
              len(gaps)>0 and min(gaps)>.47, [float(x) for x in gaps])
        check('unique_scan_submission',len({e['scan_id'] for e in submitted})==len(submitted))
        check('no_actuation_or_tf',not any(node.count_publishers(k) for k in ('/tf','/tf_static','/cmd_vel','/map')))
        stop('B')
    finally:
        if proc:
            proc.terminate()
            try:
                proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
