#!/usr/bin/env python


import struct
import socket
import rospy
from time import sleep
from threading import Lock
from robucar_driver.srv import RobotPTU
from robucar_driver.msg import RobucarDriveStamped


__author__ = "Ilyas M Abbas (ily4s.abbas@gmail.com)"


class Rdata(object):

    """This class is used to store and update data
    fetched from the robot_data topic"""

    def __init__(self):
        ''' init all data to 0 and init a threading lock '''
        self.datalock = Lock()
        self.speed = 0.0
        self.angle_forward = 0.0
        self.angle_rear = 0.0
        self.position_pan = 0
        self.position_tilt = 0
        self.speed_pan = 0
        self.speed_tilt = 0

    def ptu_update(self, req):
        ''' update ptu data '''
        try:
            self.datalock.acquire()
            self.position_tilt = req.position_tilt
            self.position_pan = req.position_pan
            self.speed_pan = req.speed_pan
            self.speed_tilt = req.speed_tilt
            self.datalock.release()
            return True
        except Exception, e:
            print e.message
            return False


class Control(object):

    """This Class handles the TCP/IP connection as well \
    as running control services"""

    def __init__(self):
        '''The init method will establish a TCP client \
        trying to connect to socket 10.1.40.56:8010    \
        if connection is unseccessful it will try again in 5 secs'''
        super(Control, self).__init__()
        HOST = rospy.get_param('/robucar_driver/robotip',   '10.1.40.56')
        PORT = rospy.get_param('/robucar_driver/robotport', 8010)
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ctrl_def = "<dddhhhh"
        self.rdata = Rdata()
        self.lock = Lock()

        a = self.conn.connect_ex((HOST, PORT))
        print a
        while a != 0:
            print "reconecting in 5\n"
            sleep(5)
            a = self.conn.connect_ex((HOST, PORT))
            print a

    def send(self, data):
        ''' This method will try to send data to robucar,
        returns False is it fails'''

        self.lock.acquire()
        try:
            self.conn.send(data)
            ret = True
        except:
            ret = False
        finally:
            self.lock.release()
            return ret

    def update_actuators(self, req):
        self.rdata.speed = req.drive.speed
        self.rdata.angle_forward = req.drive.angle_forward
        self.rdata.angle_rear = req.drive.angle_rear
        data = struct.pack(self.ctrl_def,
                           float(self.rdata.speed),
                           float(self.rdata.angle_forward),
                           float(self.rdata.angle_rear),
                           int(self.rdata.position_tilt),
                           int(self.rdata.position_pan),
                           int(self.rdata.speed_pan),
                           int(self.rdata.speed_tilt))

        return self.send(data)

    def close(self):
        '''closes the tcp socket'''
        self.conn.close()

    def __del__(self):
        """ override the default destructor : \
        close the tcp socket before exit"""
        self.close()
        print "RobuCar stopped, socket closed"


if __name__ == '__main__':

    c = Control()
    sleep(0.5)
    rospy.init_node('robucar_driver_ctrl', anonymous=True)
    rospy.Subscriber(
        'robucar_cmd', RobucarDriveStamped, c.update_actuators, queue_size=1)
    sleep(0.5)
    s = rospy.Service('robucar_ptu_serv', RobotPTU, c.rdata.ptu_update)
    print "robucar ready to receive commands"
    rospy.spin()
