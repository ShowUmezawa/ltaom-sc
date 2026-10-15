#!/usr/bin/env python3
"""Convert this bag's IMU acceleration from g to m/s^2 for FAST-LIO."""
import rospy
from sensor_msgs.msg import Imu


def main():
    rospy.init_node('imu_si_relay')
    publisher = rospy.Publisher('/imu/si', Imu, queue_size=200)

    def forward(message):
        message.linear_acceleration.x *= 9.80665
        message.linear_acceleration.y *= 9.80665
        message.linear_acceleration.z *= 9.80665
        publisher.publish(message)

    rospy.Subscriber('/imu/data', Imu, forward, queue_size=200, tcp_nodelay=True)
    rospy.spin()


if __name__ == '__main__':
    main()
