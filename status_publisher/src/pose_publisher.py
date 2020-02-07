#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import *


def main():
    rospy.init_node('pose_publisher')

    listener = tf.TransformListener()
    publisher = rospy.Publisher('/robot_pose', PoseStamped)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        r.sleep()

        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

            msg = PoseStamped()

            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = '/map'

            msg.pose.position.x = trans[0]
            msg.pose.position.y = trans[1]
            msg.pose.position.z = trans[2]

            msg.pose.orientation.x = rot[0]
            msg.pose.orientation.y = rot[1]
            msg.pose.orientation.z = rot[2]
            msg.pose.orientation.w = rot[3]

            publisher.publish(msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    rospy.spin()


if __name__ == '__main__':
    main()
