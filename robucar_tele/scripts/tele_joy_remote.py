#!/usr/bin/env python

import rospy
import sys
import socket
import struct
from sensor_msgs.msg import Joy
from time import sleep


__author__ = "Ilyas M Abbas (ily4s.abbas@gmail.com)"


class JControl(object):

    def __init__(self):
        super(JControl, self).__init__()

        RHOST = "localhost" #rospy.get_param('/robucar_driver/laptopip',  '192.168.0.102')
        RPORT = 13003
        self.server = (RHOST, RPORT)
        self.ctrl_def = "<ddds"

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

    def update(self, joy):

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

        data = struct.pack(self.ctrl_def,
                           float(self.speed),
                           float(self.phi),
                           int(self.cmode),
                           str(self.mode))

        self.conn.sendto(data, (self.server))

    def close(self):
        self.conn.close()

    def __del__(self):
        self.close()


if __name__ == '__main__':

    jc = JControl()
    sleep(0.5)
    rospy.init_node('robucar_joy_ctrl', anonymous=True)
    rospy.Subscriber('joy', Joy, jc.update, queue_size=1)
    rospy.spin()
