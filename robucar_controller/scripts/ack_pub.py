#!/usr/bin/env python


import rospy
import sys
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped


__author__ = "Ilyas M Abbas (ily4s.abbas@gmail.com)"


if __name__ == '__main__':

    if len(sys.argv) != 3:
        print sys.argv
        sys.exit(1)

    rospy.init_node('ackermann_test_publisher', anonymous=True)
    pub = rospy.Publisher('auto_cmd', AckermannDriveStamped, queue_size=1)
    ack_msg = AckermannDriveStamped()
    ack_msg.drive = AckermannDrive()
    ack_msg.drive.speed = float(sys.argv[1])
    ack_msg.drive.steering_angle = float(sys.argv[2])

    then = rospy.Time.now()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if ((rospy.Time.now() - then).to_sec() > 10 and
                (rospy.Time.now() - then).to_sec() < 14):
            ack_msg.drive.speed = 0
            ack_msg.drive.steering_angle = 0

        pub.publish(ack_msg)
        r.sleep()