#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose2D

from tf.transformations import euler_from_quaternion


class MocapTranslate:
    def __init__(self):
        self.pose_in_sub = rospy.Subscriber("pose_in", PoseStamped, self.pose_callback)
        self.pose_pub = rospy.Publisher("pose", Pose2D, queue_size=1)

    def pose_callback(self, msg):
        out_msg = Pose2D()
        out_msg.x = msg.pose.position.x
        out_msg.y = msg.pose.position.y
        _, _, out_msg.theta = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.pose_pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node("mocap_translate")
    mt = MocapTranslate()
    rospy.spin()

