#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion


class RTABTranslate:
    def __init__(self):
        self.odom_in_sub = rospy.Subscriber("odom_in", Odometry, self.odom_callback)
        self.pose_pub = rospy.Publisher("pose", Pose2D, queue_size=1)

    def odom_callback(self, msg):
        out_msg = Pose2D()
        out_msg.x = msg.pose.pose.position.x
        out_msg.y = msg.pose.pose.position.y
        _, _, out_msg.theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.pose_pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node("rtab_translate")
    rt = RTABTranslate()
    rospy.spin()

