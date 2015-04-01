#!/usr/bin/env python

##########
## by: Ilyas M Abbas - ily4s.abbas@gmail.com 
##########

import struct
import sys
import socket
import rospy
from time import sleep
from threading import Lock
from robucar.srv import RobotCtrl
from robucar.srv import RobotDrive
from robucar.srv import RobotPTU
from robucar.msg import RobotData


class Rdata(object):
	"""This class is used to store and update data
	fetched from the robot_data topic"""
	def __init__(self):
		''' init all data to 0 and init a threading lock '''
		self.datalock = Lock()
		self.speed_average = 0.0
		self.speed_FL      = 0.0
		self.speed_FR      = 0.0
		self.speed_RL      = 0.0
		self.speed_RR      = 0.0
		self.angle_forward = 0.0
		self.angle_rear    = 0.0
		self.position_pan  = 0  
		self.position_tilt = 0  
		self.speed_pan     = 0  
		self.speed_tilt    = 0  

	def update(self, data):
		''' update data '''
		self.datalock.acquire()
		self.speed_average =  data.speed_average
		self.speed_FL      =  data.speed_FL     
		self.speed_FR      =  data.speed_FR     
		self.speed_RL      =  data.speed_RL     
		self.speed_RR      =  data.speed_RR     
		self.angle_forward =  data.angle_forward
		self.angle_rear    =  data.angle_rear   
		self.position_pan  =  data.position_pan 
		self.position_tilt =  data.position_tilt
		self.speed_pan     =  data.speed_pan    
		self.speed_tilt    =  data.speed_tilt 
		self.datalock.release()

class Control(object):
	"""This Class handles the TCP/IP connection as well as running control services"""
	def __init__(self):
		'''The init method will establish a TCP client 
		trying to connect to socket 10.1.40.56:8010
		if connection is unseccessful it will try again in 5 secs'''
		super(Control, self).__init__()
		HOST = '10.1.40.56'
		PORT = 8010
		self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.ctrl_def = "<dddhhhh"
		self.rdata = Rdata()
		self.lock = Lock()

		a = self.conn.connect_ex((HOST, PORT))
		while a!=0:
			print "reconecting in 5\n"  
			sleep(5)
			a = self.conn.connect_ex((HOST, PORT))

	def close(self):
		'''closes the tcp socket'''
		self.conn.close()

	def __del__(self) :

		""" override the default destructor : close the tcp socket before exit"""
		self.close()
		print "RobuCar stopped, socket closed"


	def sendCommand(self,data):
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

	def RobuCommand(self,req):
		''' callback method for robu_control service: 
		format data then sending it by calling sendCommand '''

		data = struct.pack(self.ctrl_def, 
						   float (req.speed         ),
						   float (req.angle_forward ),
						   float (req.angle_rear    ),
						   int   (req.position_tilt ),
						   int   (req.position_pan  ),
						   int   (req.speed_pan     ),
						   int   (req.speed_tilt    ))
		
		return self.sendCommand(data)

	def DriveCommand(self,req):
		''' callback method for robu_Drive service: 
		format data then sending it by calling sendCommand.
		--- 
		note that this method is not recommended as it is 
		not well tested and add just for future hacking'''

		self.rdata.speed_average = req.speed
		self.rdata.angle_forward = req.angle_forward
		self.rdata.angle_rear    = req.angle_rear   
		data = struct.pack(self.ctrl_def, 
						   float (req.speed                ),
						   float (req.angle_forward        ),
						   float (req.angle_rear           ),
						   int   (self.rdata.position_tilt ),
						   int   (self.rdata.position_pan  ),
						   int   (self.rdata.speed_pan     ),
						   int   (self.rdata.speed_tilt    ))

		return self.sendCommand(data)
		

	def PTUCommand(self,req):
		''' callback method for robu_PTU service: 
		format data then sending it by calling sendCommand.
		--- 
		note that this method is not recommended as it is 
		not well tested and add just for future hacking'''

		self.rdata.position_tilt = req.position_tilt
		self.rdata.position_pan  = req.position_pan 
		self.rdata.speed_pan     = req.speed_pan    
		self.rdata.speed_tilt    = req.speed_tilt   

		data = struct.pack(self.ctrl_def, 
						   float (self.rdata.speed_average ),
						   float (self.rdata.angle_forward ),
						   float (self.rdata.angle_rear    ),
						   int   (req.position_tilt        ),
						   int   (req.position_pan         ),
						   int   (req.speed_pan            ),
						   int   (req.speed_tilt           ))

		return self.sendCommand(data)


if __name__ == '__main__':

	c = Control()
	sleep(0.5)
	rospy.init_node('robucar_ctrl', anonymous=True)
	rospy.Subscriber('robot_data', RobotData, c.rdata.update)
	sleep(0.5)
	s = rospy.Service( 'robu_control' , RobotCtrl  , c.RobuCommand  )
	s = rospy.Service( 'robu_Drive'   , RobotDrive , c.DriveCommand )
	s = rospy.Service( 'robu_PTU'     , RobotPTU   , c.PTUCommand   )
	print "robucar ready to receive commands"
	rospy.spin()