"""Replay original serialized inputs, pacing by recording time and driving /clock from IMU measurement time."""
import argparse
import json
from pathlib import Path
import signal
import struct
import time
import os

import rclpy
import rosbag2_py
import yaml
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message
from ros_input import topics
from run_files import write_json


def main():
    p=argparse.ArgumentParser()
    p.add_argument('run',type=Path);p.add_argument('--bag',type=Path,required=True)
    p.add_argument('--rate',type=float,required=True);args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='83' or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not .01<=args.rate<=4:
        raise ValueError('requires isolated domain 83, localhost and a finite playback rate')
    params=yaml.safe_load((args.run/'resolved.yaml').read_text())['/**']['ros__parameters']
    wanted=topics(params); imu_topic=params['common.imu_topic']
    meta=yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag),storage_id=meta['storage_identifier']),rosbag2_py.ConverterOptions('',''))
    actual={t.name:t.type for t in reader.get_all_topics_and_types()}
    if any(actual.get(k)!=v[0] for k,v in wanted.items()):raise ValueError('bag topic or type mismatch')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted)))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('culvert_measurement_clock_player')
    clock=node.create_publisher(Clock,'/clock',10)
    pubs={t:node.create_publisher(get_message(typ),t,depth) for t,(typ,depth) in wanted.items()}
    counts={t:0 for t in pubs}
    manifest={'path':str(args.bag),'storage_id':meta['storage_identifier'],'rate':args.rate,'clock':'monotonic original IMU header; scheduling uses bag storage stamp',
              'published':counts,'stored_counts':{x['topic_metadata']['name']:x['message_count'] for x in meta['topics_with_message_count'] if x['topic_metadata']['name'] in pubs},'eof':False}
    stop=False
    def request(*_):
        nonlocal stop
        stop=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,request)
    try:
        end=time.monotonic()+25
        while not stop:
            if any([i for i in node.get_publishers_info_by_topic(t) if i.node_name!=node.get_name()] for t in [*pubs,'/clock']):
                raise RuntimeError('unexpected publisher in test domain')
            if (all(any(i.node_name=='laserMapping' for i in node.get_subscriptions_info_by_topic(t)) for t in pubs)
                    and clock.get_subscription_count()>0):break
            if time.monotonic()>end:raise TimeoutError('mapper or adapter clock subscription unavailable')
            rclpy.spin_once(node,timeout_sec=.04)
        first=None;last_imu=0
        while not stop and reader.has_next():
            topic,raw,t=reader.read_next()
            if first is None:first=t;origin=time.monotonic()
            target=origin+(t-first)*1e-9/args.rate
            while not stop and time.monotonic()<target:
                time.sleep(min(.04,max(0.,target-time.monotonic())))
            if stop:break
            if topic==imu_topic:
                if raw[:2]!=b'\x00\x01':raise ValueError('unsupported IMU CDR encoding')
                sec,nano=struct.unpack_from('<iI',raw,4)
                current=sec*10**9+nano
                if current>last_imu:
                    m=Clock();m.clock.sec=sec;m.clock.nanosec=nano;clock.publish(m)
                    if not last_imu:
                        # Establish the simulated epoch before the first IMU is delivered.
                        time.sleep(.10);origin+=.10
                    last_imu=current
            pubs[topic].publish(raw);counts[topic]+=1
        manifest['eof']=not stop and counts==manifest['stored_counts']
        manifest['dds_acks']={t:p.wait_for_all_acked(Duration(seconds=5)) for t,p in pubs.items()}
        if not manifest['eof'] or not all(manifest['dds_acks'].values()):raise RuntimeError('incomplete delivery')
    finally:
        manifest['stopped']=stop
        write_json(args.run/'replay.json',manifest)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
