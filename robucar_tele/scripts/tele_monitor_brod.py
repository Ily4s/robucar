#!/usr/bin/env python

import rospy
import sys
import socket
import struct


__author__ = "Ilyas M Abbas (ily4s.abbas@gmail.com)"


class Broadcast(object):

    def __init__(self):
        super(Broadcast, self).__init__()

        RHOST = 'localhost'
        RPORT = 13008
        self.server = (RHOST, RPORT)
        self.t_def = "<d"

        try:
            self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error, msg:
            self.conn = None
            print msg.message
            sys.exit(1)

    def start(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            time = rospy.Time.now().to_sec()
            data = struct.pack(self.t_def, float(time))
            self.conn.sendto(data, (self.server))
            r.sleep()

    def close(self):
        self.conn.close()

    def __del__(self):
        self.close()


if __name__ == '__main__':

    rospy.init_node('tele_monitor_brod', anonymous=True)
    b = Broadcast()
    b.start()
    rospy.spin()
