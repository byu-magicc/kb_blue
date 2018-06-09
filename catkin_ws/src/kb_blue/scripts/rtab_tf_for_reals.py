#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import tf


class RTABTranslate:
    def __init__(self):
        self.listener = tf.TransformListener()

        self.pose_pub = rospy.Publisher("pose", Pose2D, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.get_transform)

    def get_transform(self, event):
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/camera_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("cannot get tf transform")
            return

        out_msg = Pose2D()
        out_msg.x = trans[0]
        out_msg.y = trans[1]

        _, _, out_msg.theta = euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
        self.pose_pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node("rtab_tf_translate")
    rt = RTABTranslate()
    rospy.spin()

