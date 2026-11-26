#!/usr/bin/env python3
import json
import math
import signal
import time
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

from localization_io import Server, assign_transform, cloud_xyz, map_request, pose_matrix, se3


NS = 10**9
PREFIX = '/culvert_test/localization/'


def rostime(ns):
    sec, nano = divmod(ns, NS)
    return Time(sec=sec, nanosec=nano)


def nanos(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < NS:
        return 0
    return stamp.sec * NS + stamp.nanosec


class Localizer(Node):
    def __init__(self):
        super().__init__('culvert_localization')
        defaults = dict(mode='A', backend='icp', map_manifest='', initial_pose_json='',
                        alternative_poses_json='[]', imu_from_lidar_json='', match_hz=2.0,
                        threads=1, radius=12.0, budget_ms=350.0, pair_timeout_s=.25,
                        stream_timeout_s=.75, max_stamp_age_s=.75, max_future_s=.10,
                        queue_limit=8, record_path='', save_inputs=False, server_path='', map_load_timeout_s=30.)
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        cfg = self.cfg
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('requires use_sim_time=true and measurement /clock')
        limits = {'match_hz': (.1, 20), 'threads': (1, 8), 'radius': (.1, 200),
                  'budget_ms': (1, 10000), 'pair_timeout_s': (.01, 2),
                  'stream_timeout_s': (.1, 10), 'max_stamp_age_s': (.01, 10),
                  'max_future_s': (0, 1), 'queue_limit': (1, 32), 'map_load_timeout_s': (.1, 120)}
        for k, (low, high) in limits.items():
            if not low <= cfg[k] <= high:
                raise ValueError('invalid parameter '+k)
        self.mode = cfg['mode']
        if self.mode not in ('A','B') or cfg['backend'] not in ('icp', 'gicp'):
            raise ValueError('invalid mode or backend')
        if self.mode == 'A':
            self.world, self.child, self.source = 'camera_init', 'imu', 'livo_lidar'
            topics = ('/culvert_source/state', '/culvert_source/scan')
        else:
            self.world, self.child, self.source = 'test_odom', 'test_base_link', 'test_base_link'
            topics = ('/culvert_test/odom', '/culvert_test/localization_cloud')
        self.extrinsic = None
        if self.mode == 'A' and cfg['imu_from_lidar_json']:
            self.extrinsic = se3(json.loads(cfg['imu_from_lidar_json']))
        if cfg['save_inputs'] and not cfg['record_path']:
            raise ValueError('save_inputs requires record_path')
        self.file = open(cfg['record_path'], 'x', encoding='utf-8', buffering=1) if cfg['record_path'] else None
        self.input_dir = Path(cfg['record_path']+'.inputs') if cfg['save_inputs'] else None
        if self.input_dir:
            self.input_dir.mkdir()
        binary = cfg['server_path']
        if not binary:
            binary = str(Path(get_package_prefix('culvert_registration')) /
                         'lib/culvert_registration/registration_server')
        self.server = Server(binary)
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='registration')
        self.job = None
        self.epoch = self.token = 0
        self.generation = self.meta = self.seed = self.prior = None
        self.seed_issue = self.map_issue = ''
        self.pending_load = self.latest = None
        self.last_accept = self.ros_now = 0
        self.last_clock_wall = self.last_diag = self.accept_wall = self.last_submit = 0.
        self.clock_latched = False
        self.clock_status = 'waiting_for_clock'
        self.last_frame_issue = ''
        self.bound_seed = self.bound_map = None
        self.pairs = {}
        self.mark = {'scan': 0, 'pose': 0}
        self.arrival = {'scan': 0., 'pose': 0.}
        self.counts = Counter()
        self.pub_event = self.create_publisher(String, PREFIX+'decision', 64)
        self.pub_candidate = self.create_publisher(PoseStamped, PREFIX+'candidate', 5)
        self.pub_corr = self.create_publisher(TransformStamped, PREFIX+'correction', 5)
        self.pub_diag = self.create_publisher(DiagnosticArray, PREFIX+'diagnostics', 5)
        qos = QoSProfile(depth=cfg['queue_limit'], reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, topics[0], lambda m: self.input('pose', m), qos)
        self.create_subscription(PointCloud2, topics[1], lambda m: self.input('scan', m), qos)
        self.create_subscription(ClockMessage, '/clock', self.clock_msg,
                                 QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_service(Trigger, PREFIX+'reload_map', self.reload)
        self.timer = self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.add_on_set_parameters_callback(self.set_params)
        self.load()

    def set_params(self, ps):
        allowed = {'map_manifest', 'initial_pose_json', 'alternative_poses_json'}
        return SetParametersResult(successful=all(p.name in allowed and isinstance(p.value,str) for p in ps),
                                   reason='runtime map/seed params apply together on reload_map')

    def event(self, kind, reason, stamp=0, **fields):
        self.counts[kind+'.'+reason] += 1
        record = dict(kind=kind, reason=reason, accepted=False, stamp_ns=stamp,
                      scan_id=str(stamp) if stamp else None, epoch=self.epoch,
                      generation=self.generation, mode=self.mode, backend=self.cfg['backend'],
                      source_frame=self.source, local_world_frame=self.world,
                      map_id=self.meta['id'] if self.meta else None,
                      map_version=self.meta['version'] if self.meta else None,
                      map_sha256=self.meta['sha256'] if self.meta else None,
                      map_frame=self.meta['frame'] if self.meta else None,
                      measurement_clock_ns=self.ros_now, monotonic_ns=time.monotonic_ns(),
                      last_accepted_stamp_ns=self.last_accept)
        record.update(fields)
        line = json.dumps(record, allow_nan=False, separators=(',', ':'))
        self.pub_event.publish(String(data=line))
        if self.file:
            self.file.write(line+'\n')

    def discard(self, reason):
        self.token += 1
        for stamp, entry in self.pairs.items():
            self.event('drop', reason, stamp, received=list(entry['data']))
        self.pairs.clear()
        if self.latest:
            self.event('drop', reason, self.latest['stamp'])
        self.latest = None

    def load(self):
        # Close the output gate before parsing or waiting for the new target.
        self.discard('map_changed')
        self.epoch += 1
        self.meta = self.generation = self.seed = self.prior = self.pending_load = None
        self.last_accept = self.accept_wall = 0
        self.map_issue, self.seed_issue = 'map_loading', 'no_initial_pose'
        try:
            req = map_request(self.get_parameter('map_manifest').value)
            val = self.get_parameter('initial_pose_json').value
            self.pending_load = req
            if val:
                # A map switch invalidates a seed from a previous coordinate system.
                # A map-bound {map_id, version, sha256, matrix} explicitly reauthorizes it.
                obj = json.loads(val)
                map_bound = isinstance(obj, dict)
                if map_bound:
                    if (obj.get('map_id'), obj.get('version'), obj.get('sha256')) != (
                            req['id'], req['version'], req['sha256']):
                        raise ValueError('initial_map_mismatch')
                    obj = obj['matrix']
                signature = tuple(req[k] for k in ('id','version','sha256','frame','gravity','origin'))
                if (not map_bound and self.bound_map and self.bound_map != signature and
                        np.array_equal(self.bound_seed, se3(obj))):
                    raise ValueError('initial_map_mismatch')
                seed = se3(obj)
                alternatives = json.loads(self.get_parameter('alternative_poses_json').value)
                if not isinstance(alternatives, list) or len(alternatives) > 8:
                    raise ValueError('invalid_alternative_poses')
                if any(not np.allclose(seed, se3(x), rtol=0, atol=1e-5) for x in alternatives):
                    raise ValueError('ambiguous_initial_hypotheses')
                self.seed, self.seed_issue = seed, ''
                self.bound_seed, self.bound_map = seed.copy(), signature
        except (OSError, KeyError, TypeError, ValueError) as exc:
            if self.pending_load:
                self.seed_issue = str(exc)[:180]
            else:
                self.map_issue = str(exc)[:180]
        if not self.pending_load:
            self.event('map', self.map_issue)

    def reload(self, request, response):
        self.load()
        response.success = self.pending_load is not None
        response.message = 'loading epoch '+str(self.epoch) if response.success else self.map_issue
        return response

    def clock_msg(self, msg):
        value = nanos(msg.clock)
        if value < self.ros_now:
            self.clock_latched = True
        if self.ros_now != value:
            self.last_clock_wall = time.monotonic()
        self.ros_now = value
        self.check_clock()

    def check_clock(self):
        state = ('clock_rollback_restart_required' if self.clock_latched else
                 'waiting_for_clock' if not self.ros_now else
                 'simulation_clock_paused' if time.monotonic()-self.last_clock_wall > self.cfg['stream_timeout_s']
                 else 'running')
        if state != self.clock_status:
            self.clock_status = state
            if state != 'running':
                self.discard(state)
            self.event('clock', state)
        return state

    def freshness(self, stamp):
        state = self.check_clock()
        if state != 'running':
            return state
        if stamp <= 0:
            return 'zero_stamp'
        delta = (self.ros_now-stamp)/NS
        if delta < -self.cfg['max_future_s']:
            return 'future_stamp'
        if delta > self.cfg['max_stamp_age_s']:
            return 'stale_stamp'
        return ''

    def input(self, stream, msg):
        wall = time.monotonic()
        stamp = nanos(msg.header.stamp)
        self.counts['received.'+stream] += 1
        reason = self.freshness(stamp)
        if not reason and stamp <= self.mark[stream]:
            reason = 'duplicate' if stamp == self.mark[stream] else 'out_of_order'
        if not reason and (msg.header.frame_id != (self.source if stream=='scan' else self.world) or
                           (stream == 'pose' and msg.child_frame_id != self.child)):
            reason = 'frame_mismatch'
        if not reason:
            try:
                if stream == 'scan':
                    data = cloud_xyz(msg)
                else:
                    data = pose_matrix(msg.pose.pose)
                    if self.mode == 'A':
                        if self.extrinsic is None:
                            raise ValueError('missing_imu_from_lidar')
                        data = se3(data @ self.extrinsic)
            except (TypeError, ValueError) as exc:
                reason = str(exc)
        if reason:
            self.last_frame_issue = reason
            self.event('input_reject', reason, stamp, stream=stream)
            return
        self.mark[stream], self.arrival[stream] = stamp, wall
        self.expire(wall)
        entry = self.pairs.setdefault(stamp, dict(wall=wall, data={}))
        entry['data'][stream] = data
        if len(entry['data']) == 2:
            self.pairs.pop(stamp)
            entry.update(stamp=stamp, pair_wait_ms=(wall-entry['wall'])*1000.)
            self.counts['paired'] += 1
            reason = self.map_issue or self.seed_issue
            if reason:
                self.event('drop', reason, stamp)
                return
            if self.prior is None:
                self.prior = self.seed @ np.linalg.inv(entry['data']['pose'])
                self.event('initial', 'anchored', stamp, initial=self.seed.tolist(),
                           local=entry['data']['pose'].tolist())
            if self.latest:
                self.event('drop', 'newer_complete_pair', self.latest['stamp'])
            self.latest = entry
        while len(self.pairs) > self.cfg['queue_limit']:
            old = min(self.pairs)
            popped = self.pairs.pop(old)
            self.event('drop', 'queue_overflow', old, received=list(popped['data']))

    def expire(self, wall):
        for stamp, entry in list(self.pairs.items()):
            if wall-entry['wall'] > self.cfg['pair_timeout_s']:
                self.pairs.pop(stamp)
                self.event('drop', 'pair_timeout', stamp, received=list(entry['data']))

    def finish(self, job):
        op, epoch, token, future, ctx = job
        stamp = ctx.get('stamp', 0)
        try:
            r = future.result()
        except Exception as exc:
            if epoch == self.epoch:
                self.generation = None
                self.map_issue = 'worker_requires_reload'
            self.event('worker_error', 'worker_failure', stamp, detail=str(exc)[:200], result_epoch=epoch)
            return
        if epoch != self.epoch or (op == 'match' and token != self.token):
            self.event('drop', 'late_result', stamp, result_epoch=epoch, result=r)
            return
        if op == 'load':
            identity = ('id','version','sha256','frame','units','gravity','origin')
            if time.monotonic()-self.load_started > self.cfg['map_load_timeout_s']:
                self.map_issue = 'map_load_timeout'
                self.event('map', self.map_issue, result=r)
            elif (r.get('ok') is True and type(r.get('generation')) is int and r['generation'] > 0 and
                    isinstance(r.get('map'),dict) and
                    all(r.get('map',{}).get(k) == ctx[k] for k in identity)):
                self.meta, self.generation = ctx, r['generation']
                self.map_issue = ''
                self.event('map', 'loaded', result=r, seed_issue=self.seed_issue)
            else:
                self.map_issue = 'map_load_failed'
                self.event('map', self.map_issue, result=r)
            return
        wall = time.monotonic()
        age = (wall-ctx['started'])*1000
        local = ctx['local']
        reason = 'match_timeout' if (wall-ctx['pair_wall'])*1000 > self.cfg['budget_ms'] else self.freshness(stamp)
        if not reason and (wall-ctx['pair_wall'] > self.cfg['stream_timeout_s'] or
                           any(wall-v > self.cfg['stream_timeout_s'] for v in self.arrival.values())):
            reason = 'input_stopped'
        if not reason and not (r.get('ok') is True and r.get('accepted') is True and r.get('converged') is True):
            reason = str(r.get('reason') or 'backend_rejected')
        if not reason and any(r.get(k) != v for k,v in dict(
                generation=self.generation, stamp_ns=stamp, scan_id=str(stamp), map_id=self.meta['id'],
                map_version=self.meta['version'], map_sha256=self.meta['sha256'],
                target_frame=self.meta['frame'], source_frame=self.source, backend=self.cfg['backend']).items()):
            reason = 'result_identity_mismatch'
        if not reason:
            try:
                pose, correction = se3(r['T_target_source']), se3(r['T_map_W'])
                if not np.allclose(pose, correction @ local, atol=1e-4, rtol=0):
                    raise ValueError('inconsistent_correction')
                for key in ('rmse','inlier_ratio','weak_ratio','elapsed_ms'):
                    if not math.isfinite(float(r[key])):
                        raise ValueError('nonfinite_metric')
                if r.get('ambiguous') or r.get('degenerate'):
                    raise ValueError('ambiguous_or_degenerate')
            except (ValueError, TypeError, KeyError) as exc:
                reason = 'invalid_result_'+str(exc)[:100]
        if not reason:
            self.prior, self.last_accept, self.accept_wall = correction, stamp, wall
            p, t = PoseStamped(), TransformStamped()
            for msg in (p,t):
                msg.header.frame_id, msg.header.stamp = self.meta['frame'], rostime(stamp)
            t.child_frame_id = self.world
            assign_transform(pose, p.pose.position, p.pose.orientation)
            assign_transform(correction, t.transform.translation, t.transform.rotation)
            self.pub_candidate.publish(p)
            self.pub_corr.publish(t)
        self.last_frame_issue = reason
        self.event('match', reason or 'accepted', stamp, accepted=not bool(reason), result=r,
                   local=local.tolist(), initial=ctx['initial'], end_to_end_ms=(wall-ctx['pair_wall'])*1000,
                   wall_match_ms=age, queue_wait_ms=ctx['queue_wait_ms'], pair_wait_ms=ctx['pair_wait_ms'],
                   input_points=ctx['input_points'], submitted_points=ctx['submitted_points'], input_path=ctx['input_path'])

    def match(self, req, points, path, metadata):
        if path:
            with path.open('xb') as fp:
                np.savez(fp, points=points, initial=req['initial'], local=req['local'],
                         stamp_ns=req['stamp_ns'], source_frame=req['source_frame'],
                         generation=req['generation'], **metadata)
        req['points'] = points.tolist()
        return self.server.transact(req)

    def tick(self):
        wall = time.monotonic()
        self.check_clock()
        self.expire(wall)
        if self.job and self.job[3].done():
            completed, self.job = self.job, None
            self.finish(completed)
        if self.job and self.job[0] == 'match':
            ctx = self.job[4]
            if not ctx['deadline_logged'] and (wall-ctx['pair_wall'])*1000 > self.cfg['budget_ms']:
                ctx['deadline_logged'] = True
                if self.job[1:3] == (self.epoch,self.token):
                    self.last_frame_issue = 'match_timeout'
                self.event('deadline', 'match_timeout', ctx['stamp'], result_epoch=self.job[1],
                           total_age_ms=(wall-ctx['pair_wall'])*1000)
        if not self.job and self.pending_load:
            req, self.pending_load = self.pending_load, None
            self.load_started = wall
            fut = self.pool.submit(self.server.transact, req)
            self.job = ('load', self.epoch, self.token, fut, req)
        if (not self.job and self.latest and wall - self.last_submit >= 1./self.cfg['match_hz']):
            entry, self.latest = self.latest, None
            stamp, data = entry['stamp'], entry['data']
            reason = self.map_issue or self.seed_issue or self.freshness(stamp)
            if not reason and wall-entry['wall'] > self.cfg['stream_timeout_s']:
                reason = 'expired_in_queue'
            if not reason and (wall-entry['wall'])*1000 >= self.cfg['budget_ms']:
                reason = 'queue_time_budget'
            if reason:
                self.event('drop', reason, stamp)
            else:
                local = data['pose']
                initial = (self.prior @ local).tolist()
                points = data['scan']
                # Bound wire serialization and retain source scan order. Record both counts.
                step = max(1, math.ceil(len(points)/12000))
                selected = points[::step]
                ctx = dict(stamp=stamp, local=local, initial=initial, started=wall, deadline_logged=False,
                           pair_wall=entry['wall'], pair_wait_ms=entry['pair_wait_ms'],
                           queue_wait_ms=(wall-entry['wall'])*1000-entry['pair_wait_ms'],
                           input_points=len(points), submitted_points=len(selected))
                req = {k:self.cfg[k] for k in ('backend', 'threads', 'radius', 'budget_ms')}
                req['budget_ms'] -= (wall-entry['wall'])*1000
                req.update(op='match', generation=self.generation, stamp_ns=stamp, scan_id=str(stamp),
                           source_frame=self.source, local=local.tolist(), initial=initial)
                path = self.input_dir / f'{self.epoch}_{stamp}.npz' if self.input_dir else None
                ctx['input_path'] = str(Path(path.parent.name)/path.name) if path else None
                metadata = {('map_'+k): self.meta[k] for k in ('id','version','sha256','frame')}
                fut = self.pool.submit(self.match, req, selected, path, metadata)
                self.job = ('match', self.epoch, self.token, fut, ctx)
                self.last_submit = wall
                self.event('submit', 'match', stamp, queue_wait_ms=ctx['queue_wait_ms'], input_path=ctx['input_path'],
                           local=ctx['local'].tolist(), initial=ctx['initial'])
        if wall-self.last_diag >= .2:
            self.diagnose(wall)
            self.last_diag = wall

    def diagnose(self, wall):
        issue = self.map_issue or self.seed_issue
        if not issue and self.mode == 'A' and self.extrinsic is None:
            issue = 'missing_imu_from_lidar'
        if not issue and self.clock_status != 'running':
            issue = self.clock_status
        if not issue and any(wall-v > self.cfg['stream_timeout_s'] for v in self.arrival.values()):
            issue = 'input_stopped'
        if not issue:
            issue = self.last_frame_issue or ('no_recent_accept' if not self.last_accept or
                    wall-self.accept_wall > self.cfg['stream_timeout_s'] or self.freshness(self.last_accept) else '')
        s = DiagnosticStatus()
        s.name, s.hardware_id = 'culvert_localization_candidate', 'offline'
        s.level, s.message = (DiagnosticStatus.WARN, issue) if issue else (DiagnosticStatus.OK, 'fresh_candidate')
        fields = dict(clock_status=self.clock_status, epoch=self.epoch, generation=self.generation,
                      pending_pairs=len(self.pairs), match_in_flight=bool(self.job and self.job[0]=='match'),
                      ready_pairs=int(bool(self.latest)), last_accepted_stamp_ns=self.last_accept,
                      last_accept_age_s=(self.ros_now-self.last_accept)/NS if self.last_accept else None,
                      last_accept_wall_age_s=wall-self.accept_wall if self.last_accept else None,
                      localization_valid=not bool(issue), navigation_permit=False, counts=dict(self.counts))
        s.values = [KeyValue(key=k,value=json.dumps(v,separators=(',',':'))) for k,v in fields.items()]
        msg = DiagnosticArray()
        msg.header.stamp, msg.status = rostime(self.ros_now), [s]
        self.pub_diag.publish(msg)

    def close(self):
        self.timer.cancel()
        self.server.cancelled.set()
        self.pool.shutdown(wait=True, cancel_futures=True)
        self.server.close()
        if self.file:
            self.file.close()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stopping = False
    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        node = Localizer()
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
