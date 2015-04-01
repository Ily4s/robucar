#!/usr/bin/env python

##########
## by: Ilyas M Abbas - ily4s.abbas@gmail.com 
##########

import struct
import sys
import socket
from time import sleep
import rospy
from robucar.msg import RobotData
from robucar_control import Rdata

''' simple script which subscripe to robot_data topic and 
echo data every 0.5 sec '''

if __name__ == '__main__':

	rdata = Rdata()
	sleep(1)
	rospy.init_node('robucar_echo', anonymous=True)
	rospy.Subscriber('robot_data', RobotData, rdata.update)
	while 1:
		print(rdata.speed_average ,rdata.speed_FL      ,rdata.speed_FR      ,rdata.speed_RL      ,rdata.speed_RR      ,rdata.angle_forward ,rdata.angle_rear    ,rdata.position_pan  ,rdata.position_tilt ,rdata.speed_pan     ,rdata.speed_tilt ) 
		sleep(0.5)
	rospy.spin()