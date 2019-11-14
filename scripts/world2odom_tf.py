#! /usr/bin/env python
"""
Author:         Carl Stahoviak
Date Created:   Jan 28, 2019
Last Edited:    Jan 28, 2019

Task: To publish a transform from the base_link frame to the world frame,
such that the transform is completely described by the current pose publoshed
by the Vicon system on the topic: /vrpn_client_node/RadarQuad/pose

"""

import rospy
import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import PointCloud2

br = tf2_ros.TransformBroadcaster()

def pose_msg_cb(msg):
    # br = tf2_ros.TransformBroadcaster()
    tf = TransformStamped()

    tf.header.stamp = msg.header.stamp
    tf.header.frame_id = rospy.get_param('~frame_id')
    tf.child_frame_id = rospy.get_param('~child_frame_id')

    tf.transform.translation.x = msg.pose.position.x
    tf.transform.translation.y = msg.pose.position.y
    tf.transform.translation.z = msg.pose.position.z

    tf.transform.rotation.x = msg.pose.orientation.x
    tf.transform.rotation.y = msg.pose.orientation.y
    tf.transform.rotation.z = msg.pose.orientation.z
    tf.transform.rotation.w = msg.pose.orientation.w

    br.sendTransform(tf)

def odom_msg_cb(msg):
    # br = tf2_ros.TransformBroadcaster()
    tf = TransformStamped()

    tf.header.stamp = msg.header.stamp
    tf.header.frame_id = rospy.get_param('~frame_id')
    tf.child_frame_id = rospy.get_param('~child_frame_id')
    # tf.child_frame_id = msg.header.frame_id

    tf.transform.translation.x = msg.pose.pose.position.x
    tf.transform.translation.y = msg.pose.pose.position.y
    tf.transform.translation.z = msg.pose.pose.position.z

    tf.transform.rotation.x = msg.pose.pose.orientation.x
    tf.transform.rotation.y = msg.pose.pose.orientation.y
    tf.transform.rotation.z = msg.pose.pose.orientation.z
    tf.transform.rotation.w = msg.pose.pose.orientation.w

    br.sendTransform(tf)

def main():
    rospy.init_node('world2odom_tf')
    pose_topic = rospy.get_param('~pose_topic')

    if pose_topic == "camera/odom/sample":
        pose_sub = rospy.Subscriber(pose_topic, Odometry, odom_msg_cb)
    else:
        pose_sub = rospy.Subscriber(pose_topic, PoseStamped, pose_msg_cb)

    rospy.loginfo("Subscribing to %s", pose_topic)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit()

if __name__ == '__main__':
    main()
