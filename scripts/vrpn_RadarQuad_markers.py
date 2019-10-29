#! /usr/bin/env python
"""
Author:         Carl Stahoviak
Date Created:   Jan 28, 2019
Last Edited:    Jan 29, 2019

Task: To publish the coordinate locations of a set of static markers to be
placed into the RVIZ environment for the DARPA demo happening on 02/01/2019

"""

import rospy
import math
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler


marker0_pub = rospy.Publisher("/marker0", Marker, queue_size=10)    # test object
marker1_pub = rospy.Publisher("/marker1", Marker, queue_size=10)    # skinny tube
marker2_pub = rospy.Publisher("/marker2", Marker, queue_size=10)    # 10" tube
marker3_pub = rospy.Publisher("/marker3", Marker, queue_size=10)    # 10" tube
marker4_pub = rospy.Publisher("/marker4", Marker, queue_size=10)    # 10" tube
marker5_pub = rospy.Publisher("/marker5", Marker, queue_size=10)    # wall 1
marker6_pub = rospy.Publisher("/marker6", Marker, queue_size=10)    # wall 2


def object_pose_pub():
    r = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():

        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = "world"
        marker.ns = "objects"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration()
        marker.color.r = 1      # red
        marker.color.g = 0      # green
        marker.color.b = 0      # blue
        marker.color.a = 0.4    # opacity

        # skinny tube #1
        marker.type = Marker.CYLINDER
        marker.scale.x = 0.0889      # x-direction diameter - 3.5"
        marker.scale.y = 0.0889      # y-direction diameter - 3.5"
        marker.scale.z = 1.553
        marker.pose.position.x = 0.206
        marker.pose.position.y = 1.863
        marker.pose.position.z = 0.5*marker.scale.z
        marker1_pub.publish(marker)

        # 10" diameter tube #2
        marker.type = Marker.CYLINDER
        marker.scale.x = 0.2413      # x-direction diameter - 9.5"
        marker.scale.y = 0.2413      # y-direction diameter - 9.5"
        marker.scale.z = 1.150
        marker.pose.position.x = 2.058
        marker.pose.position.y = 1.863
        marker.pose.position.z = 0.5*marker.scale.z
        marker2_pub.publish(marker)

        # 10" diameter tube #3
        marker.type = Marker.CYLINDER
        marker.scale.x = 0.2413      # x-direction diameter - 9.5"
        marker.scale.y = 0.2413      # y-direction diameter - 9.5"
        marker.scale.z = 1.150
        marker.pose.position.x = 4.065
        marker.pose.position.y = 1.886
        marker.pose.position.z = 0.5*marker.scale.z
        marker3_pub.publish(marker)

        # 10" diameter tube #4
        marker.type = Marker.CYLINDER
        marker.scale.x = 0.2413      # x-direction diameter - 9.5"
        marker.scale.y = 0.2413      # y-direction diameter - 9.5"
        marker.scale.z = 1.50
        marker.pose.position.x = 7.261
        marker.pose.position.y = 0.530
        marker.pose.position.z = 0.5*marker.scale.z
        marker4_pub.publish(marker)

        # wall #1
        marker.type = Marker.CUBE
        marker.scale.x = 4.101 - 1.645
        marker.scale.y = 0.05               # need to measure thickness
        marker.scale.z = 0.6477             # 25 inches high

        marker.pose.position.x = 1.645 + 0.5*marker.scale.x
        marker.pose.position.y = (-1.617 + (-1.583))/2
        marker.pose.position.z = 0.5*marker.scale.z + 0.4572
        marker5_pub.publish(marker)

        # wall #2
        marker.type = Marker.CUBE
        marker.scale.x = math.sqrt((8.049-6.508)**2 + (-3.369-(-1.517))**2)
        marker.scale.y = 0.05               # need to measure thickness
        marker.scale.z = 0.6477             # 25 inches high

        marker.pose.position.x = (8.049 + 6.508)/2
        marker.pose.position.y = (-3.369 + (-1.517))/2
        marker.pose.position.z = 0.5*marker.scale.z + 0.4572

        q = quaternion_from_euler(0, 0, 0.15 + math.atan((8.049-6.508)/(-1.517-(-3.369))))
        marker.pose.orientation.x = q[0];
        marker.pose.orientation.y = q[1];
        marker.pose.orientation.z = q[2];
        marker.pose.orientation.w = q[3];
        marker6_pub.publish(marker)

        # test object
        marker.type = Marker.CUBE
        marker.scale.x = 2
        marker.scale.y = 0.5              # need to measure thickness
        marker.scale.z = 1             # 25 inches high

        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0.5*marker.scale.z + 0.4572

        q = quaternion_from_euler(0, 0, math.pi/4)
        marker.pose.orientation.x = q[0];
        marker.pose.orientation.y = q[1];
        marker.pose.orientation.z = q[2];
        marker.pose.orientation.w = q[3];
        marker0_pub.publish(marker)

        r.sleep()


if __name__ == '__main__':
    rospy.init_node('static_marker_node')

    rospy.loginfo("Publishing static markers[1-6]")

    try:
        object_pose_pub()
    except rospy.ROSInterruptException: # ctrl-c
        pass

    rospy.spin()
