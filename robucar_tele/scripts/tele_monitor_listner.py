#!/usr/bin/env python


import rospy
import sys
import socket
import struct
from time import sleep
from robucar_driver.srv import DriveMode


__author__ = "Ilyas M. Abbas (ily4s.abbas@gmail.com)"


class Monitor(object):

    def __init__(self):
        super(Monitor, self).__init__()

        HOST = '0.0.0.0'
        PORT = 13008
        self.ctrl_def = "<d"
        self.data = [0]
        self.then = rospy.Time.now()
        self.now = rospy.Time.now()

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

    def start(self):

        rospy.wait_for_service('robucar_stop')
        stop = rospy.ServiceProxy('robucar_stop', DriveMode)

        buf = self.conn.recvfrom(struct.calcsize(self.ctrl_def))
        self.data = struct.unpack(self.ctrl_def, buf[0])
        self.then = rospy.Time.from_sec(self.data[0])
        self.conn.settimeout(0.4)

        while not rospy.is_shutdown():
            try:
                buf = self.conn.recvfrom(struct.calcsize(self.ctrl_def))
                self.data = struct.unpack(self.ctrl_def, buf[0])
                self.now = rospy.Time.from_sec(self.data[0])
                diff = (self.now - self.then).to_sec()
                if diff > 0.3:
                    print "HALT"
                    stop(1)
                self.then = self.now
            except Exception, e:
                print "HALT"
                stop(1)
                raise e

    def close(self):
        self.conn.close()

    def __del__(self):
        self.close()

if __name__ == '__main__':

    rospy.init_node('tele_monitor_listner', anonymous=True)

    m = Monitor()
    sleep(0.5)
    try:
        m.start()
        rospy.spin()
    except rospy.ROSInterruptException, e:
        print e.message
