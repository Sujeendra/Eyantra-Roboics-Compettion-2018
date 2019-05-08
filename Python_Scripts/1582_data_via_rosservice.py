#!/usr/bin/env python
from plutodrone.srv import *
from std_msgs.msg import Int32
from std_msgs.msg import Int16
import rospy

class request_data():
	"""docstring for request_data"""
	def __init__(self):
		rospy.init_node('drone_board_data')
		self.x=0
		self.y=0
		self.z=0
		self.flag = rospy.Publisher('/alt',Int16,queue_size =1)
		self.flag2 = rospy.Publisher('/pitch',Int16,queue_size =1)
		self.flag3=rospy.Publisher('/roll',Int16,queue_size=1)
		data = rospy.Service('PlutoService', PlutoPilot, self.access_data)
		rospy.spin()
	    
		
		
		
		
		
		
		

	def access_data(self, req):
		 print "accx = " + str(req.accX), "accy = " + str(req.accY), "accz = " + str(req.accZ)
		 print "gyrox = " + str(req.gyroX), "gyroy = " + str(req.gyroY), "gyroz = " + str(req.gyroZ)
		 print "magx = " + str(req.magX), "magy = " + str(req.magY), "magz = " + str(req.magZ)
		 print "roll = " + str(req.roll), "pitch = " + str(req.pitch), "yaw = " + str(req.yaw)
		 print "altitude = " +str(req.alt)
		 self.x=req.alt
		 self.y=req.pitch
		 self.z=req.roll
		 self.flag.publish(self.x)
		 self.flag2.publish(self.y)
		 self.flag3.publish(self.z)
		 rospy.sleep(.1)

		 
		 
		 return PlutoPilotResponse(rcAUX2=1500)
	
		
		
		

		
test = request_data()

		
