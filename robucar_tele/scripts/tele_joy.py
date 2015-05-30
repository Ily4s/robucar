#!/usr/bin/env python


import rospy
import sys
import socket
import struct
from robucar_driver.msg import JoyDrive
from robucar_driver.msg import JoyDriveStamped
from time import sleep


__author__ = "Ilyas M Abbas (ily4s.abbas@gmail.com)"


class Monitoring(object):

    def __init__(self):
        super(Monitoring, self).__init__()

        rospy.init_node('robucar_joy_mon', anonymous=True)

        HOST = "localhost" #rospy.get_param('/robucar_driver/laptopip',  '10.1.40.98')
        PORT = 13003
        print "connected: " + str(HOST) + ":" + str(PORT)
        self.ctrl_def = "<ddds"
        self.s = None
        self.data = [0] * 4
        self.stampedJoy = JoyDriveStamped()
        self.stampedJoy.drive = JoyDrive()

        print "host: " + str(HOST)

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

    def monitor(self):

        pub = rospy.Publisher('joy_cmd', JoyDriveStamped, queue_size=1)
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

    m = Monitoring()
    sleep(1)
    try:
        m.monitor()
    except rospy.ROSInterruptException:
        pass
