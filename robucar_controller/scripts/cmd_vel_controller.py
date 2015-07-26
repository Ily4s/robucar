#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
from math import tan, atan, asin, degrees


__author__ = "Ilyas M. Abbas (ily4s.abbas@gmail.com)"


class Twist_to_ack(object):

    """docstring for Twist_to_ack"""

    def __init__(self):
        super(Twist_to_ack, self).__init__()
        rospy.init_node('cmd_vel_controller', anonymous=True)

        self.ack_msg = AckermannDriveStamped()
        self.ack_msg.drive = AckermannDrive()
        self.publisher = rospy.Publisher('auto_cmd',
                                         AckermannDriveStamped,
                                         queue_size=1)

    def convert(self, tw):
        v = tw.linear.x
        w = tw.angular.z
        lim = (v / 1.206) * tan(w)
        self.ack_msg.header.stamp = rospy.Time.now()
        self.ack_msg.drive.speed = v
        if round(v, 1) != 0.0:
            if lim < w:
                self.ack_msg.drive.steering_angle = degrees(
                    atan((1.206 / v) * w)
                )
            else:
                self.ack_msg.drive.steering_angle = degrees(
                    asin((1.206 / v) * w)
                )

        self.publisher.publish(self.ack_msg)


if __name__ == '__main__':

    conv = Twist_to_ack()
    rospy.Subscriber('/cmd_vel',
                     Twist,
                     conv.convert,
                     queue_size=1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS"
