#!/usr/bin/env python


import rospy
import sys
import socket
import struct
from time import sleep
from threading import Thread
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from robucar_driver.msg import JoyDrive
from robucar_driver.msg import JoyDriveStamped


__author__ = "Ilyas M. Abbas (ily4s.abbas@gmail.com)"


class Move(Thread):

    def __init__(self):
        super(Move, self).__init__()

        self.setDaemon(True)

        HOST = '0.0.0.0'
        PORT = 13007
        self.ctrl_def = "<ddddd"
        self.s = None
        self.data = [0] * 5

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

        # pub = rospy.Publisher('joy_cmd', JoyDriveStamped, queue_size=1)
        # r = rospy.Rate(10)

        simpleac = SimpleActionClient("move_base", MoveBaseAction)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '/map'

        while not rospy.is_shutdown():
            buf = self.conn.recvfrom(struct.calcsize(self.ctrl_def))
            self.data = struct.unpack(self.ctrl_def, buf[0])

            goal.target_pose.header.stamp = rospy.Time.from_sec(self.data[0])
            goal.target_pose.pose.position.x = self.data[1]
            goal.target_pose.pose.position.y = self.data[2]
            goal.target_pose.pose.orientation.z = self.data[3]
            goal.target_pose.pose.orientation.w = self.data[4]

            print self.data
            simpleac.send_goal(goal)

            # pub.publish(self.stampedJoy)

            # r.sleep()

    def close(self):
        self.conn.close()

    def __del__(self):
        self.close()


class Tele(Thread):

    def __init__(self):
        super(Tele, self).__init__()

        self.setDaemon(True)

        HOST = '0.0.0.0'
        PORT = 13003

        self.ctrl_def = "<ddds"
        self.s = None
        self.data = [0] * 4
        self.stampedJoy = JoyDriveStamped()
        self.stampedJoy.drive = JoyDrive()

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

        pub = rospy.Publisher('tele_cmd', JoyDriveStamped, queue_size=1)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            buf = self.conn.recvfrom(struct.calcsize(self.ctrl_def))
            self.data = struct.unpack(self.ctrl_def, buf[0])

            self.stampedJoy.header.stamp = rospy.Time.now()
            self.stampedJoy.drive.speed = self.data[0]
            self.stampedJoy.drive.steering_angle = self.data[1]
            self.stampedJoy.drive.cmode = self.data[2]
            self.stampedJoy.drive.mode = self.data[3]

            pub.publish(self.stampedJoy)

            r.sleep()

    def close(self):
        self.conn.close()

    def __del__(self):
        self.close()

if __name__ == '__main__':

    rospy.init_node('robucar_tele_base', anonymous=True)

    m = Move()
    t = Tele()
    sleep(0.5)
    try:
        m.start()
        t.start()
        rospy.spin()
    except rospy.ROSInterruptException, e:
        print e.message
