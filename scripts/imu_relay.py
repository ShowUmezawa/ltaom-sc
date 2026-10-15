#!/usr/bin/env python3
"""Convert IMU units using the resolved replay sensor profile."""
import argparse
import json
from pathlib import Path
import rospy
from sensor_msgs.msg import Imu
from sensor_profile import convert_imu

parser = argparse.ArgumentParser()
parser.add_argument('--profile', type=Path, required=True)
args = parser.parse_args()
profile = json.loads(args.profile.read_text())
if not profile['relay'] or profile['imu'] == profile['bag_imu']:
    raise ValueError('relay requires conversion and distinct input/output topics')
rospy.init_node('imu_si_relay')
publisher = rospy.Publisher(profile['imu'], Imu, queue_size=200)

def forward(message):
    publisher.publish(convert_imu(message, profile))

subscriber = rospy.Subscriber(profile['bag_imu'], Imu, forward, queue_size=200, tcp_nodelay=True)
rospy.spin()
