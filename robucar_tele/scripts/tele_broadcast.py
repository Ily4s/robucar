#!/usr/bin/env python

import rospy
import sys
import socket
import struct
from tf.msg import tfMessage
from nav_msgs.msg import Odometry
from robucar_driver.msg import RobotDataStamped
from time import sleep

__author__ = "Ilyas M Abbas (ily4s.abbas@gmail.com)"


class Broadcast(object):

    def __init__(self):
        super(Broadcast, self).__init__()

        RHOST = '192.168.1.100'
        TF_RPORT = 13004
        ODOM_RPORT = 13005
        R_RPORT = 13006
        self.tf_server = (RHOST, TF_RPORT)
        self.odom_server = (RHOST, ODOM_RPORT)
        self.r_server = (RHOST, R_RPORT)
        self.tf_def = "<ddddd"
        self.odom_def = "<dddddddd"
        self.r_def = "<dddd"

        self.tf_time = rospy.Time.now().to_sec()
        self.tx = 0.0
        self.ty = 0.0
        self.rz = 0.0
        self.rw = 0.0

        self.odom_time = rospy.Time.now().to_sec()
        self.px = 0.0
        self.py = 0.0
        self.oz = 0.0
        self.ow = 0.0
        self.lx = 0.0
        self.ly = 0.0
        self.az = 0.0

        self.r_time = rospy.Time.now().to_sec()
        self.r_speed = 0.0
        self.r_fangle = 0.0
        self.r_rangle = 0.0

        try:
            self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error, msg:
            self.conn = None
            print msg.message
            sys.exit(1)

    def tf_update(self, msg):
        if (msg.transforms[0].header.frame_id == "map" and
                msg.transforms[0].child_frame_id == "odom"):
            self.tf_time = msg.transforms[0].header.stamp.to_sec()
            self.tx = msg.transforms[0].transform.translation.x
            self.ty = msg.transforms[0].transform.translation.y
            self.rz = msg.transforms[0].transform.rotation.z
            self.rw = msg.transforms[0].transform.rotation.w

            data = struct.pack(self.tf_def,
                               float(self.tf_time),
                               float(self.tx),
                               float(self.ty),
                               float(self.rz),
                               float(self.rw))

            self.conn.sendto(data, (self.tf_server))

    def r_update(self, msg):
        self.r_time = msg.header.stamp.to_sec()
        self.r_speed = msg.data.speed_average
        self.r_fangle = msg.data.angle_forward
        self.r_rangle = msg.data.angle_rear

        data = struct.pack(self.r_def,
                           float(self.r_time),
                           float(self.r_speed),
                           float(self.r_fangle),
                           float(self.r_rangle))

        self.conn.sendto(data, (self.r_server))

    def odom_update(self, msg):
        self.odom_time = msg.header.stamp.to_sec()
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.oz = msg.pose.pose.orientation.z
        self.ow = msg.pose.pose.orientation.w
        self.lx = msg.twist.twist.linear.x
        self.ly = msg.twist.twist.linear.y
        self.az = msg.twist.twist.angular.z

        data = struct.pack(self.odom_def,
                           float(self.odom_time),
                           float(self.px),
                           float(self.py),
                           float(self.oz),
                           float(self.ow),
                           float(self.lx),
                           float(self.ly),
                           float(self.az))

        self.conn.sendto(data, (self.odom_server))

    def close(self):
        self.conn.close()

    def __del__(self):
        self.close()


if __name__ == '__main__':

    rospy.init_node('robucar_tele_broadcater', anonymous=True)
    b = Broadcast()
    sleep(0.5)
    rospy.Subscriber('/tf', tfMessage, b.tf_update, queue_size=1)
    rospy.Subscriber('/odom', Odometry, b.odom_update, queue_size=1)
    rospy.Subscriber('/robot_data', RobotDataStamped, b.r_update, queue_size=1)

    rospy.spin()
