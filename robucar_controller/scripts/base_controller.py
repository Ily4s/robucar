#!/usr/bin/env python


import rospy
from robucar_driver.srv import DriveMode
from robucar_driver.msg import JoyDriveStamped
from robucar_driver.msg import RobucarDrive
from robucar_driver.msg import RobucarDriveStamped
from ackermann_msgs.msg import AckermannDriveStamped
from math import sin
from math import asin
from math import radians
from math import degrees


__author__ = "Ilyas M. Abbas (ily4s.abbas@gmail.com)"


class Driver(object):

    """docstring for Driver"""

    def __init__(self):
        super(Driver, self).__init__()
        rospy.init_node('ackermann_controller', anonymous=True)

        self.max_angle = 18.0
        self.max_f_speed = 5.0
        self.max_b_speed = 1.0

        self.speed = 0.0
        self.phi = 0.0
        self.rphi = 0.0
        self.next_speed = 0.0
        self.next_phi = 0.0
        self.next_rphi = 0.0

        self.robucar_msg = RobucarDriveStamped()
        self.robucar_msg.drive = RobucarDrive()
        self.robucar_msg.drive.angle_rear = 0.0

        self.cmode = 0
        self.dmode = 0
        self.prev_dmode = 0

        self.joy_next_speed = 0.0
        self.joy_next_phi = 0.0
        self.joy_mode = 0

        self.ack_next_speed = 0.0
        self.ack_next_phi = 0.0
        self.ack_mode = 0

    def joy_update(self, msg):
        self.joy_next_speed = msg.drive.speed
        self.joy_next_phi = msg.drive.steering_angle
        self.cmode = msg.drive.cmode

        if msg.drive.mode == "d":
            self.joy_mode = 1
        elif msg.drive.mode == "p":
            self.joy_mode = 2
        else:
            self.joy_mode = 0

    def ack_update(self, msg):
        sa = msg.drive.steering_angle
        self.ack_next_speed = msg.drive.speed
        if abs(sa) <= 18:
            self.ack_next_phi = sa
            self.ack_mode = 0
        elif abs(sa) > 18:
            self.ack_next_phi = degrees(asin(sin(radians(sa)) / 2.0))
            self.ack_mode = 1

    def drive(self):
        pub = rospy.Publisher('robucar_cmd', RobucarDriveStamped, queue_size=1)
        rospy.wait_for_service('robucar_drive_mode')
        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            if self.cmode == 0:
                self.next_speed = self.ack_next_speed
                self.next_phi = self.ack_next_phi
                self.dmode = self.ack_mode

            elif self.cmode == 1:
                self.next_speed = self.joy_next_speed
                self.next_phi = self.joy_next_phi
                self.dmode = self.joy_mode

            if self.dmode == 0:
                self.next_rphi = 0.0
            elif self.dmode == 1:
                self.next_rphi = -self.next_phi
            elif self.dmode == 2:
                self.next_rphi = self.next_phi

            if self.dmode != self.prev_dmode:
                try:
                    dm = rospy.ServiceProxy('robucar_drive_mode', DriveMode)
                    dm(self.dmode)
                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e

            self.prev_dmode = self.dmode

            self.next_speed = self.max_f_speed if (
                self.next_speed > self.max_f_speed) else self.next_speed
            self.next_speed = -self.max_b_speed if (
                self.next_speed < -self.max_b_speed) else self.next_speed
            self.next_phi = self.max_angle if (
                self.next_phi > self.max_angle) else self.next_phi
            self.next_phi = -self.max_angle if (
                self.next_phi < -self.max_angle) else self.next_phi
            self.next_rphi = self.max_angle if (
                self.next_rphi > self.max_angle) else self.next_rphi
            self.next_rphi = -self.max_angle if (
                self.next_rphi < -self.max_angle) else self.next_rphi

            # then  = rospy.Time.now()
            #   dt = (rospy.Time.now() - then).to_sec()
            #   if dt < 0.2:
            #       sleep(0.2-dt)
            #   self.call(s,angle_inc)
            #   then  = rospy.Time.now()

            if round(self.next_speed, 2) > round(self.speed, 2):
                self.speed += 0.01
            elif round(self.next_speed, 2) < round(self.speed, 2):
                self.speed -= 0.01

            if round(self.next_phi, 1) > round(self.phi, 1):
                self.phi += 0.25
                self.phi = 18.0 if (self.phi > 18.0) else self.phi
            elif round(self.next_phi, 1) < round(self.phi, 1):
                self.phi -= 0.25
                self.phi = -18.0 if (self.phi < -18.0) else self.phi

            if round(self.next_rphi, 1) > round(self.rphi, 1):
                self.rphi += 0.25
                self.rphi = 18.0 if (self.rphi > 18.0) else self.rphi
            elif round(self.next_rphi, 1) < round(self.rphi, 1):
                self.rphi -= 0.25
                self.rphi = -18.0 if (self.rphi < -18.0) else self.rphi

            self.robucar_msg.header.stamp = rospy.Time.now()
            self.robucar_msg.drive.speed = self.speed
            self.robucar_msg.drive.angle_forward = self.phi
            self.robucar_msg.drive.angle_rear = self.rphi

            pub.publish(self.robucar_msg)

            r.sleep()

if __name__ == '__main__':

    drive = Driver()
    # s = rospy.Service('robucar_control_mode', DriveMode, drive.control_mode)
    rospy.Subscriber(
        'ackermann_cmd', AckermannDriveStamped, drive.ack_update, queue_size=1)
    rospy.Subscriber(
        'joy_cmd', JoyDriveStamped, drive.joy_update, queue_size=1)
    print "start spin"
    drive.drive()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS"
