#!/usr/bin/env python


import struct
import sys
import socket
import rospy
from time import sleep
from robucar_driver.srv import DriveMode
from robucar_driver.msg import RobotData
from robucar_driver.msg import RobotDataStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from math import sin
from math import cos
from math import tan
from math import radians


__author__ = "Ilyas M Abbas (ily4s.abbas@gmail.com)"


class Monitoring(object):

    """This Class handles the TCP/IP connection as
     well as publishing data to topic"""

    def __init__(self):
        '''The init method will establish a TCP server
                litening on socket 10.1.40.98:8000'''
        super(Monitoring, self).__init__()

        rospy.init_node('robucar_mon', anonymous=True)

        HOST = rospy.get_param('~ip',  '10.1.40.98')
        PORT = rospy.get_param('~port', 8000)

        self.ctrl_def = "<dddddddhhhh"
        self.s = None
        self.data = [0] * 11
        self.stampedData = RobotDataStamped()
        self.stampedData.data = RobotData()

        self.L = 1.206
        self.x = 0
        self.y = 0
        self.th = 0
        self.mode = 0  # 0 for simpl 1 for dual steering
        self.then = rospy.Time.now()
        self.now = rospy.Time.now()

        for res in socket.getaddrinfo(HOST, PORT,
                                      socket.AF_UNSPEC, socket.SOCK_STREAM,
                                      0, socket.AI_PASSIVE):
            af, socktype, proto, canonname, sa = res
            try:
                self.s = socket.socket(af, socktype, proto)
            except socket.error, msg:
                self.s = None
                continue
            try:
                self.s.bind(sa)
                self.s.listen(1)
            except socket.error, msg:
                print msg.message
                self.s.close()
                self.s = None
                continue
            break
        if self.s is None:
            print 'could not open socket'
            sys.exit(1)

        self.conn, addr = self.s.accept()
        print 'Connected by', addr
        print "Serving at port " + str(PORT)

    def monitor(self):
        ''' This function starts the robucar_mon node and begin publishing
            the received data to robot_data using  RobotDataStamped msg'''

        pub = rospy.Publisher('robot_data', RobotDataStamped, queue_size=1)
        odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        tf_broadcaster = TransformBroadcaster()
        odom = Odometry(
            header=rospy.Header(frame_id="odom"), child_frame_id='base_link')
        self.then = rospy.Time.now()

        while not rospy.is_shutdown():
            buf = self.conn.recv(struct.calcsize(self.ctrl_def))
            self.data = struct.unpack(self.ctrl_def, buf)
            self.now = rospy.Time.now()
            v = self.data[0]
            phi = radians(self.data[5])

            if (self.now - self.then).to_sec() < 0.095:
                pass
            else:
                dt = (self.now - self.then).to_sec()
                if self.mode == 0:
                    dx = v * cos(self.th)
                    dy = v * sin(self.th)
                    dth = (v / self.L) * tan(phi)
                elif self.mode == 1:
                    dx = v * cos(self.th + phi)
                    dy = v * sin(self.th + phi)
                    dth = (2 * v / self.L) * sin(phi)
                elif self.mode == 2:
                    dx = v * cos(self.th)
                    dy = v * sin(self.th)
                    dth = 0.0

                self.x += dx * dt
                self.y += dy * dt
                self.th += dth * dt

                quaternion = Quaternion(*quaternion_from_euler(0, 0, self.th))

                odom.header.stamp = self.now
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.position.z = 0
                odom.pose.pose.orientation = quaternion
                odom.twist.twist.linear.x = dx
                odom.twist.twist.linear.y = dy
                odom.twist.twist.angular.z = dth

                self.stampedData.header.stamp = rospy.Time.now()
                self.stampedData.data.speed_average = self.data[0]
                self.stampedData.data.speed_FL = self.data[1]
                self.stampedData.data.speed_FR = self.data[2]
                self.stampedData.data.speed_RL = self.data[3]
                self.stampedData.data.speed_RR = self.data[4]
                self.stampedData.data.angle_forward = self.data[5]
                self.stampedData.data.angle_rear = self.data[6]
                self.stampedData.data.position_pan = self.data[7]
                self.stampedData.data.position_tilt = self.data[8]
                self.stampedData.data.speed_pan = self.data[9]
                self.stampedData.data.speed_tilt = self.data[10]

                pub.publish(self.stampedData)
                odom_pub.publish(odom)
                tf_broadcaster.sendTransform((self.x, self.y, 0),
                                             (quaternion.x, quaternion.y,
                                                 quaternion.z, quaternion.w),
                                             self.now,
                                             "base_link",
                                             "odom")

                # qfix = Quaternion(*quaternion_from_euler(0, 0, 0))
                # tf_broadcaster.sendTransform((0, 0, 0),
                #                              (qfix.x, qfix.y, qfix.z, qfix.w),
                #                              self.now,
                #                              "odom",
                #                              "map")

                self.then = self.now

    def drive_mode(self, req):
        try:
            if req.mode >= 0 and req.mode <= 2:
                self.mode = req.mode
                if self.mode == 0:
                    st = "simple"
                elif self.mode == 1:
                    st = "dual"
                elif self.mode == 2:
                    st = "park"
                print "mode " + st + " engaged"
            return True
        except Exception, e:
            print e.message
            return False

    def close(self):
        '''closes the tcp socket'''
        self.conn.close()

    def __del__(self):
        """ override the default destructor :
        close the tcp socket before exit"""
        self.close()
        print "RobuCar stopped, socket closed"


if __name__ == '__main__':

    print "starting monitoring server"
    m = Monitoring()
    s = rospy.Service('robucar_drive_mode', DriveMode, m.drive_mode)
    sleep(1)
    print "begin publishing"
    try:
        m.monitor()
    except rospy.ROSInterruptException:
        pass
