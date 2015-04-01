#!/usr/bin/env python

##########
## by: Ilyas M Abbas - ily4s.abbas@gmail.com 
##########

import struct
import sys
import socket
import rospy
from time import sleep
from robucar.msg import RobotData


class Monitoring(object):

	"""This Class handles the TCP/IP connection as well as publishing data
	to topic"""

	def __init__(self):
		'''The init method will establish a TCP server 
		litening on socket 10.1.40.98:8000'''
		super(Monitoring, self).__init__()
		HOST = '10.1.40.98'
		PORT = 8000               
		self.s = None
		self.data = [0] * 11

		for res in socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC, socket.SOCK_STREAM, 0, socket.AI_PASSIVE):
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
				self.s.close()
				self.s = None
				continue
			break
		if self.s is None:
			print 'could not open socket'
			sys.exit(1)

		self.conn, addr = self.s.accept()
		print 'Connected by', addr
		print "Serving at port 8000"
				

	def monitor(self):
		''' This function starts the robucar_mon node and begin publishing
		    the received data to robot_data using  RobotData msg'''
		ctrl_def = "<dddddddhhhh"
		pub = rospy.Publisher('robot_data', RobotData, queue_size=10)
		rospy.init_node('robucar_mon', anonymous=True)
		#r = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			buf = self.conn.recv(struct.calcsize(ctrl_def))
			self.data = struct.unpack(ctrl_def, buf)
			#rospy.loginfo(str)
			pub.publish(self.data[0] ,
						self.data[1] ,
						self.data[2] ,
						self.data[3] ,
						self.data[4] ,
						self.data[5] ,
						self.data[6] ,
						self.data[7] ,
						self.data[8] ,
						self.data[9] ,
						self.data[10])
			#r.sleep()

	def close(self):
		'''closes the tcp socket'''
		self.conn.close()

	def __del__(self):
		""" override the default destructor : close the tcp socket before exit"""
		self.close()
		print "RobuCar stopped, socket closed"


if __name__ == '__main__':
	
	print "starting monitoring server"
	m = Monitoring()
	sleep(1)
	print "begin publishing"
	try:
		m.monitor()
	except rospy.ROSInterruptException: pass