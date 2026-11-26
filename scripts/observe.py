"""Observe isolated replay deliveries and compare messages with the same estimator stamp."""
import collections
import json
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry, Path as PosePath
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, Image
from tf2_msgs.msg import TFMessage


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def vec(value):
    return np.array([value.x, value.y, value.z])


def rotation(q):
    v = vec(q)
    skew = np.array([[0., -q.z, q.y], [q.z, 0., -q.x], [-q.y, q.x, 0.]])
    return np.eye(3) + 2. * (q.w * skew + skew @ skew)


def xyz(msg):
    offsets = {f.name: f.offset for f in msg.fields}
    dtype = np.dtype(dict(names=list('xyz'), formats=['>f4' if msg.is_bigendian else '<f4']*3,
                          offsets=[offsets[c] for c in 'xyz'], itemsize=msg.point_step))
    records = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                         strides=(msg.row_step, msg.point_step))
    return np.stack([records[c].ravel() for c in 'xyz'], axis=1)


def dist(values):
    if not len(values):
        return None
    return {key:float(v) for key,v in zip(('min','p50','p95','max'),np.percentile(values,[0,50,95,100]))}


def main():
    run = Path(sys.argv[1])
    mode = sys.argv[2]
    rclpy.init()
    node = rclpy.create_node('culvert_round1_observer')
    qos = QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT)
    seen = collections.Counter()
    times = collections.defaultdict(list)
    frames = collections.defaultdict(set)
    detail, poses, diag, edges, pair = {}, collections.defaultdict(list), [], collections.defaultdict(set), {}
    paired_errors=collections.defaultdict(list)
    errors, nonfinite = collections.defaultdict(list), collections.Counter()
    start = time.monotonic()
    def match(kind, side, stamp, value):
        key=(kind,stamp)
        other=pair.pop(key,None)
        if other is None:
            if len(pair)>128:
                pair.pop(next(iter(pair)))
            pair[key]=(side,value)
        elif other[0]!=side:
            left,right=(value,other[1]) if side=='src' else (other[1],value)
            if left.shape!=right.shape:
                raise ValueError('source/output shape mismatch')
            errors[kind].append(float(np.max(np.abs(left-right))))
            paired_errors[kind].append([stamp,errors[kind][-1]])
    def consume(topic, msg):
        seen[topic]+=1
        if isinstance(msg, TFMessage):
            for t in msg.transforms:
                edge=t.header.frame_id+' -> '+t.child_frame_id
                edges[topic].add(edge)
                times[edge].append([ns(t.header.stamp),time.monotonic()-start])
                poses[edge].append([ns(t.header.stamp),*vec(t.transform.translation),*vec(t.transform.rotation),t.transform.rotation.w])
                if edge=='test_odom -> test_base_link':
                    match('tf_position_m','src',ns(t.header.stamp),vec(t.transform.translation))
                    match('tf_rotation_matrix','src',ns(t.header.stamp),rotation(t.transform.rotation))
            return
        stamp = ns(msg.header.stamp)
        times[topic].append([stamp,time.monotonic()-start])
        frames[topic].add(msg.header.frame_id)
        if isinstance(msg, Imu):
            sample=np.r_[vec(msg.angular_velocity),vec(msg.linear_acceleration)]
            nonfinite[topic]+=int(not np.isfinite(sample).all())
            detail.setdefault(topic,dict(first_measurement=sample.tolist(),orientation_w=msg.orientation.w))
        elif isinstance(msg, Odometry):
            p,q = msg.pose.pose.position,msg.pose.pose.orientation
            twist=np.r_[vec(msg.twist.twist.linear),vec(msg.twist.twist.angular)]
            data=[stamp,*vec(p),*vec(q),q.w,*twist]
            poses[topic].append(data); nonfinite[topic]+=int(not np.isfinite(data).all())
            detail.setdefault(topic,dict(child=msg.child_frame_id,pose_cov=list(msg.pose.covariance),twist_cov=list(msg.twist.covariance)))
            if topic=='/culvert_source/state':
                match('base_position_m','src',stamp,vec(p)+rotation(q)@np.array([.2,0,.1]))
                match('base_rotation_matrix','src',stamp,rotation(q))
            elif topic=='/culvert_test/odom':
                match('base_position_m','out',stamp,vec(p)); match('base_rotation_matrix','out',stamp,rotation(q))
                match('tf_position_m','out',stamp,vec(p)); match('tf_rotation_matrix','out',stamp,rotation(q))
        elif isinstance(msg, PointCloud2):
            if not msg.width or not msg.height or not {'x','y','z'} <= {f.name for f in msg.fields}:
                detail.setdefault(topic+'_empty_or_missing_xyz',0); detail[topic+'_empty_or_missing_xyz']+=1
                return
            points=xyz(msg); nonfinite[topic]+=int((~np.isfinite(points)).any(axis=1).sum())
            detail.setdefault(topic,dict(fields=[[f.name,f.offset,f.datatype] for f in msg.fields],point_step=msg.point_step,first_count=len(points)))
            if topic=='/culvert_source/scan':
                if seen[topic] in (10,60,120):
                    frame=run/'samples'
                    frame.mkdir(exist_ok=True)
                    offset=next(f.offset for f in msg.fields if f.name=='intensity')
                    intensity=np.ndarray((msg.height,msg.width),dtype='<f4',buffer=msg.data,
                                         offset=offset,strides=(msg.row_step,msg.point_step)).ravel()
                    payload=np.column_stack((points,intensity)).astype('<f4')
                    name=str(stamp)+'.pcd'
                    lines=['VERSION .7','FIELDS x y z intensity','SIZE 4 4 4 4','TYPE F F F F','COUNT 1 1 1 1',f'WIDTH {len(points)}','HEIGHT 1','VIEWPOINT 0 0 0 1 0 0 0',f'POINTS {len(points)}','DATA binary','']
                    header='\n'.join(lines)
                    (frame/name).write_bytes(header.encode('ascii')+payload.tobytes())
                match('all_xyz_m','src',stamp,points.astype(float)+[-.211,-.02329,-.05588])
            if topic=='/culvert_test/localization_cloud':
                match('all_xyz_m','out',stamp,points)
        elif isinstance(msg, DiagnosticArray):
            diag.extend([{'level':int.from_bytes(s.level,'little') if isinstance(s.level,bytes) else int(s.level),'message':s.message,**{v.key:v.value for v in s.values}} for s in msg.status])
    specs={Odometry:['/aft_mapped_to_init','/LIVO2/imu_propagate','/culvert_source/state','/culvert_test/odom'],
           PointCloud2:['/cloud_registered','/cloud_effected','/Laser_map','/culvert_source/scan','/culvert_test/localization_cloud'],
           Imu:['/livox/imu','/culvert_test/imu'],PosePath:['/path'],
           DiagnosticArray:['/culvert_test/diagnostics','/diagnostics'], TFMessage:['/tf','/tf_static']}
    for typ,topics in specs.items():
        for topic in topics:
            profile=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic=='/tf_static' else qos
            node.create_subscription(typ,topic,lambda m,t=topic:consume(t,m),profile)
    finished=False
    def stop(*_):
        nonlocal finished
        finished=True
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    (run/'observer.ready').touch()
    while not finished:
        rclpy.spin_once(node,timeout_sec=.15)
    graph={t: [[p.node_name,p.topic_type] for p in node.get_publishers_info_by_topic(t)] for t,_ in node.get_topic_names_and_types()}
    summary={'counts':seen,'frames':{k:sorted(v) for k,v in frames.items()},'fields':detail,'nonfinite':nonfinite,
             'tf':{k:sorted(v) for k,v in edges.items()},'compared':{k:{'count':len(v),'error':dist(v)} for k,v in errors.items()},
             'paired_errors':paired_errors,'diag_tail':diag[-1:] ,'graph':graph,'elapsed_s':time.monotonic()-start}
    for name,data in [('observation',summary),('stamps',times),('poses',poses),('diagnostics',diag)]:
        (run/(name+'.json')).write_text(json.dumps(data,allow_nan=False))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
