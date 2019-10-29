#! /usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker

transform = tf.TransformBroadcaster()
pc_pub = rospy.Publisher("/mmWave_repub", PointCloud2, queue_size=10)

marker1_pub = rospy.Publisher("/marker1", Marker, queue_size=10)
marker2_pub = rospy.Publisher("/marker2", Marker, queue_size=10)
marker3_pub = rospy.Publisher("/marker3", Marker, queue_size=10)
marker4_pub = rospy.Publisher("/marker4", Marker, queue_size=10)
marker5_pub = rospy.Publisher("/marker5", Marker, queue_size=10)
marker6_pub = rospy.Publisher("/marker6", Marker, queue_size=10)

def pose_repub(msg):
    t = TransformStamped()
    t.child_frame_id = "base_link"
    t.header.stamp = rospy.Time(0)
    t.header.frame_id = msg.header.frame_id

    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z

    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w

    transform.sendTransformMessage(t)

def pt_cloud_repub(msg):
    msg.header.frame_id = "base_link"
    msg.header.stamp = rospy.Time(0)
    pc_pub.publish(msg)


def object_pose_pub():
    r = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():

        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = "world"
        marker.ns = "objects"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration()
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 0.4

        # tables

        marker.type = Marker.CUBE
        marker.scale.x = 0.71
        marker.scale.y = 0.1
        marker.scale.z = 1.52

        marker.pose.position.x = -0.55
        marker.pose.position.y = 1.48
        marker.pose.position.z = 0.76
        marker1_pub.publish(marker)

        marker.pose.position.x = 1.425
        marker.pose.position.y = -0.77
        marker.pose.position.z = 0.76
        marker2_pub.publish(marker)

        # long thing with cylinder holding up
        # long thing
        marker.type = Marker.CUBE
        marker.scale.x = 2.39
        marker.scale.y = 0.05
        marker.scale.z = 1.2
        marker.pose.position.x = 3.705
        marker.pose.position.y = 1.22
        marker.pose.position.z = 0.6
        marker3_pub.publish(marker)

        # cylinder ( x is the same as ^)
        marker.type = Marker.CYLINDER
        marker.scale.x = 0.33
        marker.scale.y = 0.33
        marker.scale.z = 1.25
        marker.pose.position.y = 1.42
        marker.pose.position.z = 0.625
        marker4_pub.publish(marker)

        # cylinders
        marker.type = Marker.CYLINDER
        marker.scale.x = 0.33
        marker.scale.y = 0.33
        marker.scale.z = 1.25

        marker.pose.position.x = 6.03
        marker.pose.position.y = -.73
        marker.pose.position.z = 0.625
        marker5_pub.publish(marker)

        marker.pose.position.x = 8.06
        marker.pose.position.y = 1.56
        marker.pose.position.z = 0.625
        marker6_pub.publish(marker)



        r.sleep()


if __name__ == '__main__':
    rospy.init_node('post_tf')
    pose_topic = "/vrpn_client_node/SubT/pose"
    pt_cloud_topic = "/mmWaveDataHdl/RScan"
    p = rospy.Subscriber(pose_topic, PoseStamped, pose_repub)
    p1 = rospy.Subscriber(pt_cloud_topic, PointCloud2, pt_cloud_repub)

    rospy.loginfo("Subscribing to %s"%pose_topic)
    rospy.loginfo("Subscribing to %s"%pt_cloud_topic)
    # rospy.loginfo("Publishing static poses[1-5]")
    rospy.loginfo("Publishing static markers[1-6]")

    try:
        object_pose_pub()
    except rospy.ROSInterruptException: # ctrl-c
        pass




    rospy.spin()
