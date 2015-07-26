#!/usr/bin/env python

import rospy
import sys
import socket
import struct
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from time import sleep

__author__ = "Ilyas M Abbas (ily4s.abbas@gmail.com)"


class Broadcast(object):

    def __init__(self):
        super(Broadcast, self).__init__()

        RHOST = rospy.get_param('~ip',  'localhost')
        MOVE_RPORT = 13007
        TELE_RPORT = 13003
        self.move_server = (RHOST, MOVE_RPORT)
        self.tele_server = (RHOST, TELE_RPORT)
        self.move_def = "<ddddd"
        self.tele_def = "<ddds"

        self.time = rospy.Time.now().to_sec()
        self.px = 0.0
        self.py = 0.0
        self.oz = 0.0
        self.ow = 0.0

        self.speed = 0.0
        self.phi = 0.0
        self.manual = False
        self.mode = "normal"
        self.cmode = 0

        try:
            self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error, msg:
            self.conn = None
            print msg.message
            sys.exit(1)

    def move_update(self, msg):
        self.time = msg.header.stamp.to_sec()
        self.px = msg.pose.position.x
        self.py = msg.pose.position.y
        self.oz = msg.pose.orientation.z
        self.ow = msg.pose.orientation.w

        data = struct.pack(self.move_def,
                           float(self.time),
                           float(self.px),
                           float(self.py),
                           float(self.oz),
                           float(self.ow))

        self.conn.sendto(data, (self.move_server))

    def tele_update(self, joy):
        if joy.axes[0] == 0:
            self.phi = 0.0
        elif joy.axes[0] == -1:
            self.phi = -18.0
        else:
            self.phi = 18.0

        if joy.buttons[5] == 1:
            self.speed += 0.1
        if joy.buttons[7] == 1:
            self.speed -= 0.1
        if joy.buttons[0] == 1:
            self.mode = "n"
        if joy.buttons[1] == 1:
            self.mode = "d"
        if joy.buttons[2] == 1:
            self.mode = "p"
        if joy.buttons[3] == 1:
            self.speed = 0.0
        if joy.buttons[4] == 1:
            self.manual = not self.manual
            if self.manual:
                self.cmode = 1
            else:
                self.cmode = 0

        data = struct.pack(self.tele_def,
                           float(self.speed),
                           float(self.phi),
                           int(self.cmode),
                           str(self.mode))

        self.conn.sendto(data, (self.tele_server))

    def close(self):
        self.conn.close()

    def __del__(self):
        self.close()


if __name__ == '__main__':

    rospy.init_node('remote_commander', anonymous=True)
    b = Broadcast()
    sleep(0.5)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped,
                     b.move_update, queue_size=1)
    rospy.Subscriber('joy', Joy, b.tele_update, queue_size=1)

    rospy.spin()
