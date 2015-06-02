#!/usr/bin/env python


import rospy
import sys
import socket
import struct
from time import sleep
from threading import Thread
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from robucar_driver.msg import SimpleRobotData


__author__ = "Ilyas M Abbas (ily4s.abbas@gmail.com)"


class Tf_serv(Thread):

    def __init__(self):
        super(Tf_serv, self).__init__()

        self.setDaemon(True)

        HOST = '0.0.0.0'
        PORT = 13004
        self.tf_def = "<ddddd"
        self.s = None
        self.data = [0] * 5

        self.tf_time = rospy.Time.now()
        self.tx = 0.0
        self.ty = 0.0
        self.rz = 0.0
        self.rw = 0.0

        try:
            self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error, msg:
            self.conn = None
            print msg.message
            sys.exit(1)

        try:
            self.conn.bind((HOST, PORT))
        except socket.error, msg:
            self.conn.close()
            self.conn = None
            print msg.message

    def run(self):

        tf_broadcaster = TransformBroadcaster()

        print "tf run"

        while not rospy.is_shutdown():
            buf = self.conn.recvfrom(struct.calcsize(self.tf_def))
            self.data = struct.unpack(self.tf_def, buf[0])

            self.tf_time = rospy.Time.from_sec(self.data[0])
            self.tx = self.data[1]
            self.ty = self.data[2]
            self.rz = self.data[3]
            self.rw = self.data[4]

            tf_broadcaster.sendTransform((self.tx, self.ty, 0),
                                         (0.0, 0.0, self.rz, self.rw),
                                         self.tf_time,
                                         "odom",
                                         "map")

    def close(self):
        self.conn.close()

    def __del__(self):
        self.close()


class Odom_serv(Thread):

    def __init__(self):
        super(Odom_serv, self).__init__()

        self.setDaemon(True)

        HOST = '0.0.0.0'
        PORT = 13005
        self.odom_def = "<dddddddd"
        self.s = None
        self.data = [0] * 8

        self.odom_time = rospy.Time.now()
        self.px = 0.0
        self.py = 0.0
        self.oz = 0.0
        self.ow = 0.0
        self.lx = 0.0
        self.ly = 0.0
        self.az = 0.0

        try:
            self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error, msg:
            self.conn = None
            print msg.message
            sys.exit(1)

        try:
            self.conn.bind((HOST, PORT))
        except socket.error, msg:
            self.conn.close()
            self.conn = None
            print msg.message

    def run(self):

        pub = rospy.Publisher('odom', Odometry, queue_size=1)
        tf_broadcaster = TransformBroadcaster()
        odom = Odometry(
            header=rospy.Header(frame_id="odom"), child_frame_id='base_link')

        print "odom run"

        while not rospy.is_shutdown():
            buf = self.conn.recvfrom(struct.calcsize(self.odom_def))
            self.data = struct.unpack(self.odom_def, buf[0])

            self.odom_time = rospy.Time.from_sec(self.data[0])
            self.px = self.data[1]
            self.py = self.data[2]
            self.oz = self.data[3]
            self.ow = self.data[4]
            self.lx = self.data[5]
            self.ly = self.data[6]
            self.az = self.data[7]

            odom.header.stamp = self.odom_time
            odom.pose.pose.position.x = self.px
            odom.pose.pose.position.y = self.py
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation.x = 0
            odom.pose.pose.orientation.y = 0
            odom.pose.pose.orientation.z = self.oz
            odom.pose.pose.orientation.w = self.ow
            odom.twist.twist.linear.x = self.lx
            odom.twist.twist.linear.y = self.ly
            odom.twist.twist.angular.z = self.az

            pub.publish(odom)
            tf_broadcaster.sendTransform((self.px, self.py, 0),
                                         (0.0, 0.0, self.oz, self.ow),
                                         self.odom_time,
                                         "base_link",
                                         "odom")

    def close(self):
        self.conn.close()

    def __del__(self):
        self.close()


class R_serv(Thread):

    def __init__(self):
        super(R_serv, self).__init__()

        self.setDaemon(True)

        HOST = '0.0.0.0'
        PORT = 13006
        self.r_def = "<dddd"
        self.s = None
        self.data = [0] * 4

        self.r_time = rospy.Time.now()
        self.r_speed = 0.0
        self.r_fangle = 0.0
        self.r_rangle = 0.0

        try:
            self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error, msg:
            self.conn = None
            print msg.message
            sys.exit(1)

        try:
            self.conn.bind((HOST, PORT))
        except socket.error, msg:
            self.conn.close()
            self.conn = None
            print msg.message

    def run(self):

        pub = rospy.Publisher('myrobot_data', SimpleRobotData, queue_size=1)
        sdata = SimpleRobotData()

        print "r run"

        while not rospy.is_shutdown():
            buf = self.conn.recvfrom(struct.calcsize(self.r_def))
            self.data = struct.unpack(self.r_def, buf[0])

            self.r_time = rospy.Time.from_sec(self.data[0])
            self.r_speed = self.data[1]
            self.r_fangle = self.data[2]
            self.r_rangle = self.data[3]

            sdata.header.stamp = self.r_time
            sdata.speed_average = self.r_speed
            sdata.angle_forward = self.r_fangle
            sdata.angle_rear = self.r_rangle

            pub.publish(sdata)

    def close(self):
        self.conn.close()

    def __del__(self):
        self.close()

if __name__ == '__main__':

    rospy.init_node('robucar_tele_rpub', anonymous=True)
    tf = Tf_serv()
    odom = Odom_serv()
    r = R_serv()
    sleep(0.5)
    try:
        tf.start()
        odom.start()
        r.start()
        rospy.spin()
    except rospy.ROSInterruptException, e:
        print e
