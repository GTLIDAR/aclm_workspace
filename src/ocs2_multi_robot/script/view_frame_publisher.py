#!/usr/bin/env python3
"""
Publishes a 'view' TF frame that follows the cargo frame's XYZ position
but maintains a fixed orientation (yaw=2, pitch=0, roll=0) relative to odom.
"""

import rospy
import tf2_ros
import geometry_msgs.msg
import math


def main():
    rospy.init_node("view_frame_publisher")

    source_frame = rospy.get_param("~source_frame", "cargo")
    parent_frame = rospy.get_param("~parent_frame", "odom")
    child_frame = rospy.get_param("~child_frame", "view")
    fixed_yaw = rospy.get_param("~fixed_yaw", 2.0)
    rate_hz = rospy.get_param("~rate", 50.0)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Pre-compute fixed quaternion from yaw only (pitch=0, roll=0)
    qz = math.sin(fixed_yaw / 2.0)
    qw = math.cos(fixed_yaw / 2.0)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform(parent_frame, source_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        # Copy position from cargo
        t.transform.translation = trans.transform.translation

        # Fixed orientation in odom: yaw=2, pitch=0, roll=0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        tf_broadcaster.sendTransform(t)
        rate.sleep()


if __name__ == "__main__":
    main()
