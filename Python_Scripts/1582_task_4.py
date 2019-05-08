#!/usr/bin/env python
'''
* Team Id :1582
* Author List :Sujeendra R,Likth K,Keerthan S,Ponnamanda Bhaskar Sampath Sai Sriram
* Filename:1582_progress_task.py
* Theme:Hungry bird
* Functions:__init__(),disarm(),arm(),key_callback(),whycon_callback,Compute(),SetTunings(),SetSampleTime(),self.pathtable(),self.path(),__main__()
* Global Variables: None

'''

# Importing the required libraries

from plutodrone.msg import *
from plutodrone.srv import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import random

class Edrone():
	"""docstring for Edrone"""
	
	'''
	* Function Name:__init__()
	* Input:None
	* Output:Every local variables are initialised or declared before its use in main()
	* Logic:As the name itself suggest that it is initialisation function used to set intial required value to the 
	* variable 
	* Example Call:There is no need of calling it since it is implicit function
	'''
	def __init__(self):
		
		# initializing ros node with name path_plan
		rospy.init_node('path_plan')
		#Getting the drone_position from the whycon callback function 

		self.drone_position = [0.0,0.0,0.0] 
		#This is the setpoint or initial point to which drone should move it is near 1D position after that path plan will 
		#start whose x and y are set dynamically in vrep using  a key 'i'

		self.setpoint = [0,0,28] 
		#Declaring a cmd of message type PlutoMsg and initializing motor speed in each axis like pitch,roll yaw and throttle to 1500 which is neutral value

		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500	
		#Since we are tuning the drone in three axis i.e [pitch,roll,throttle]we need to set proportional,integral and derivative terms in each axis.
		self.Kp = [15,15.84,96]

		#ki corresponds to integral terms in each axis
		self.Ki = [0,0,0]

		#kd corresponds to derivative term in each axis
		self.Kd = [5,2,80]

		#this is a variable used as count which helps in disarming the drone after all foods are picked 
		self.q=17

		#Maximum limit to the motor' speed for each axis is set
		self.max_values = [1600,1600,1800]

		#Minimum limit to the motor speed for each axis is set
		self.min_values = [1400,1400,1400]

		#Difference between the drone position and set point will give this error wherin pid is applied to it to reduce error
		self.error=[0,0,0]

		#This will accumulate the error for the time being the drone is in flight 
		self.error_sum=[0,0,0]

		#this is a flag used as a condition to run pid node as per requirement i.e using 'a' and 'd' (arm and disarm) key we can start or stop the pathplan node
		self.mainflag=0

		#Output value which is added to 1500 in pitch direction after pid is applied
		self.out_pitch=0

		#Output value which is added to 1500 in throttle direction after pid is applied
		self.out_alt=0
		self.out=0

		#Output value which is added to 1500 in roll direction after pid is applied
		self.out_roll=0

		#Initial start time required to run pid in a sample of time
		self.start=0

		#final time is set started initially using time library
		self.last=time.time()

		#this is difference in time between start and last
		self.timechange=0

		#these are constants which changes after proportional,derivative,integral terms above declared are changed due to sampling time
		self.kp=[0,0,0]
		self.ki=[0,0,0]
		self.kd=[0,0,0]
		#This is the ratio of new sampling time to previous sampling time
		self.ratio=0.0

		#Final pid output value is stored in a list
		self.result=[self.out_pitch,self.out_roll,self.out_alt]

		#The input keyboard key value is read after key is pressed as per assignment in key_command.py plutodrone package
		self.flag=0

		#this will have the previous position of drone
		self.lastinput=[0,0,0]

		#Differential input is the difference between the current drone position and previous drone position
		self.dinput=[0,0,0]

		#This is a varibale used to publish a count or flag to vrep via /status_flag topic
		self.flagpub =0.0

		#This is the flag used to control of publishing count via a /status_flag topic initially set to 1
		self.flag2 =1

		#Will contain all the pathpoints coming from /vrep_waypoints topic initially dictionary is set to null value
		self.table = {}

		#This is the count or incrementer send to vrep to publish next successive path
		self.count =0

		#initial indexed/first point to which drone should go right after the path plan is done it is not zero because we need drone to move faster
		self.i=10

		#this variable will have last setpoint value plus skip points used to reset flag so that next path is computed
		self.value1=0

		#this will have actual z axis value w.r.t to pressure sensor after subtracting with ground value
		self.a=0.0

		#this will have altitude value after 7.68 scaling factor is multiplied to drone data
		self.aa=0.0

		#initial value of altitude from drone is stored during first count
		self.ini=0.0

		#this count is used in altitude method
		self.count1=0

		#whycon z value after conversion from pressure sensor value is stored
		self.w=0.0

		#centimeter to meter conversion value is stored
		self.c=0
		
		#Used as a flag to know if whycon callback has happend or not 
		self.pre=0
		
		#Number of path points computed in vrep 
		self.value=0

		#Absolute error or positive error is computed here required to give next setpoint
		self.error3 = [0,0,0]

		#Used to control pid node from main function after the drone is armed
		self.mainFlag = 0

		#Indexed skip points which helps in faster movement of drone it will skip points in path depending on count i.e. based on hoop position or dummy
		self.skip=0

		#count used to establish a whycon loose condition
		self.con=0

		#sampling time helps in converting discrete code to analog if it is very small
		self.sample_time = 0.1

		#This is changed as per user requirement which will not affect pid tuning
		self.NewSampleTime=0.001

		#Declaring or calling the publisher /drone_command used to command motor speed according to pid output
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)

		#Declaring/caling a publisher /status_flag used to send count or flag to vrep so tha next path is produced
		self.flag_1 = rospy.Publisher('/status_flag',Int16,queue_size =1)

		#Subscribing to whycon/poses topic helps in reading whycon x,y,z values from drone in this case.
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)

		#Subscribing to /input_key will helps in getting message after each key press from keyboard
		rospy.Subscriber('/input_key',Int16, self.key_callback)

		#Will be published by vrep which will send packed data of Posearray type contains path points.
		rospy.Subscriber('/vrep/waypoints',PoseArray,self.pathtable)

		#Will be published by drone_board_data node in dataviarosserice.py file
		rospy.Subscriber('/alt',Int16, self.altitude)

	#----------------------------------------------------------------------------------------------------------------------------------------------#
	'''
	* Function Name:key_callback()
	* Input:message from keycommand node
	* Output:msg.data which is a single integer values used to perform disarm() task while pid tuning to avoid drone crashing
	* Logic:using if statement to compare a key pressed assigned value in key_command.py with "a" key pressed drone will arm  since msg will be 70 and sets mainFlag which will runs pid method 
	  also sends data 8 via /status_flag topic to vrep so that drone in simualtor get emulated.
	* Example Call:since it is a callback function from subscriber it is automatically called when subscriber is defined
	'''		
			
	def key_callback(self,msg):
		#since message is of Int16 datatype it is accessed and stored in flag variable as shown below
		
		self.flag=msg.data
		#printing of message on a screen to check if the pressed key is correct
		print "msg" ,self.flag
		#if the key pressed is "d" then drone will disarm()
		if self.flag==0:
			#calling of disarm() method
			self.disarm()
			self.mainFlag=0
			self.mianflag=1
		#if key pressed is "a" then drone will arm 

		elif self.flag==70:
			#calling arm() method
			self.arm()
			#mainFlag is set such that during arming it should start computing error in pid method
			self.mainFlag = 1
			#two flags are used to arm and disarm the drone whenever required partcularly used during reposition
			self.mainflag=0
			#170 number of Int16 is published via a /status_flag topic which will emulate drone in simulation 
			self.flag_1.publish(170)
			#count during usual run
			self.q=16

		#key 5 is pressed whenever drone crashes after passing through one food tree which helps in arming the drone from start and start continuing from 
		#previously computed path where drone got crashed.
		elif self.flag==5:
			#calling arm() method
			self.arm()
			#mainFlag is set such that during arming it should start computing error in pid method
			self.mainFlag = 1
			self.mainflag=0
			#8 number of Int16 is published via a /status_flag topic which will emulate drone in simulation 
			self.flag_1.publish(180)
			self.q=13

		#key 6 is pressed whenever drone crashes after passing through two food tree which helps in arming the drone from start and start continuing from 
		#previously computed path where drone got crashed.
		elif self.flag==6:
			#calling arm() method
			self.arm()
			#mainFlag is set such that during arming it should start computing error in pid method
			self.mainFlag = 1
			self.mainflag=0
			#8 number of Int16 is published via a /status_flag topic which will emulate drone in simulation 
			self.flag_1.publish(190)
			self.q=8
		#-------------------------------------------------------------------------------------------------------------------------------------------------------


	'''
	* Function Name:disarm()
	* Input:None
	* Output:Publishes all drone_command values as mentioned above
	* Logic:simply changing the rcAUX4 values to 1100 such that drone disarms because 1500 is neutral 1500+ is positive and 1500- is negative
	* Example Call:self.disarm()

	'''
	def disarm(self):
		#command to rcAUX4 is set to 1100 such that rest all values remains at 1500 will disarm the drone
		self.cmd.rcAUX4 = 1100

		#these values are published via /drone_command topic
		self.command_pub.publish(self.cmd)

		#program sleeps for 1 second
		rospy.sleep(1)

	#------------------------------------------------------------------------------------------------------------------------
	'''
	* Function Name:altitude()
	* Input:message from publisher /alt topic which is published in dataviarosservice.py 
	* Output:whycon value in z coordinate 
	* Logic:drone board data will give pressure sensor altitude which is first converted to real world coordinate using scaling factor then real world is converted to 
	whycon coordinate using below mentioned formula.This helps in controlling of drone when whycon looses .
	* Example Call:it is automatically called when /alt topic publishes the req.alt data

	'''		

	def altitude(self,msg):
		#this count is needed to set the drone position from pressure sensor when kept on ground since it is not zero
		self.count1=self.count1+1
		if self.count1==1:
			#reading the initial value when count is 1 
			self.ini=msg.data
		#for count greater than 0  message or altitude is stored in self.a variable after subtracting from initial value on the ground level	
		else:
			self.a=msg.data-self.ini

			#scaling factor is multiplied to drone data to convert into real world coordinate
			self.aa=(7.68*self.a)
			self.c=self.aa/float(100)

			#this is the value in whycon coordinate i.e. in z-coordinate
			self.w=((3.3-self.c)*35)/3.3
			
	#---------------------------------------------------------------------------------------------------------------------------		
			

	'''
	* Function Name:arm()
	* Input:None
	* Output:Publishes all drone_command values as mentioned above
	* Logic:initially disarm() is called before arming of drone and simply changing the rcAUX4 values to 1500 will arm the drone
	* Example Call:self.arm()
	'''
		
	def arm(self):
		#it is a good practice to call disarm and arm method
		self.disarm()
		#rcThrottle command is set to 1000 will arm() the drone
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		#these values are published via /drone_command topic
		self.command_pub.publish(self.cmd) 
		#program starts for 1 second 
		rospy.sleep(1)
	#-----------------------------------------------------------------------------------------------------------------------------
	'''
	* Function Name:whycon_callback()
	* Input:message from usb camera and whycon/poses topics
	* Output:msg is of datatype Posearray gives position and orientation totallly 7 vlaues i.e. x,y,z and x,y,z,w
	* Logic:since droneposition has to be updated with whycon values we update it by accesing msg as shown below
	* Example Call:since it is a callback function from subscriber it is automatically called when subscriber is defined
	'''

	def whycon_callback(self,msg):
		
		#since message is of Posearray type x y and z coordinates ar accessed as shown below
		#this is dynamically setting the start setpoint when key 'i' is pressed with z kept constant initially to a particular height
		if self.flag==14:
			#rounded to 1 decimal place value
			self.setpoint[0] = round(msg.poses[0].position.x,1)
			self.setpoint[1] = round(msg.poses[0].position.y,1)
			
		
				
		#everytime flag pre as been set to 1 when whycon callback is called or whycon looses during run
		self.pre= 1
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		
		self.drone_position[2]=msg.poses[0].position.z
		
		
	#-----------------------------------------------------------------------------------------------------------------------------------------------	
		
			
		

		
	'''
	* Function Name:pathtable()
	* Input:Message from /vrep_waypoints topic which sends path points for real world after the path plan 
	* Output:Message is of data type Posearray whcih is packed into a dictionary we get 7*no of path points in vrep required for path planning
	* Logic:Using the real to whycon conversion factor for each axis we use only points from x,y,z direction only orientation is x,y,z,w 
	* is not required since start and goal dummy will have the same orientation.
	* Example Call:It is a callback function from subscriber it is automatically called when topic /vrep_waypoints is published by vrep and subscribed by python node path_plan
	'''
	
	def pathtable(self,msg):
		#even this message is of Posearray datatype packed into a dictionary hence initially stored in table variable
		self.table  = msg
		self.value=len(self.table.poses)
	#------------------------------------------------------------------------------------------------------------------------------------------------	
	'''
	* Function Name:Compute()
	* Input:None
	* Output:After applying PID gain formula we get an output in each axis
	* Logic:runing this node for a sample period of time i.e. for 10ms which computes error and differential input and integral sum known as error_sum
	* error_sum is limited to a certain value before using it further since it accumulates error
	* Example Call:self.Compute()
	'''
		
	def Compute(self):
		#start time is set using time module
		self.start=time.time()
		#timechange is the difference in time
		self.timechange=self.start-self.last
		#timechange is grater than or euqual to sample time then pid equations are applied for error values
		if self.timechange>=self.sample_time:
			#for loop applied in 3 axis such that each of the list elements are accessed
			for i in range(3):
				#error is calculated
				self.error[i]=self.drone_position[i]-self.setpoint[i]
				#differential input is calculated
				self.dinput[i]=self.drone_position[i]-self.lastinput[i]
				#absolute error is computed
				self.error3[i]=abs(self.error[i])
				#sum of errors is accumulated used to set integral terms in pid equations
				self.error_sum[i]+=self.error[i]*self.ki[i]
				#limiting the errorsum value if it exceeds 10
				if self.error_sum[i]>25:
					self.error_sum[i]=25
				#limiting the errorsum value if it is less than -10	
				if self.error_sum[i]<-25:
					self.error_sum[i]=-25
				#current position of drone is recorded such that it can be used for next iteration of compute() method
				self.lastinput[i]=self.drone_position[i]
			#seperate for loop for pid equations in each of the 3 axis is applied to get output value.
			for i in range(3):
				self.result[i]=(self.kp[i]*self.error[i])+(self.kd[i]*self.dinput[i])+(self.error_sum[i])
			#last time is set to current time
			self.last=self.start
		#----------------------------------------------------------------------------------------------------------------------------------	
	'''
	* Function Name:SetTunings()
	* Input:None
	* Output:It computes kp,ki,kd values by making use of new sample_time
	* Logic:Using for loop in each axis to get kp,ki and kd values
	* Example Call:self.SetTuninigs()
	'''   

	def SetTunings(self):
		#for loop to compute kp,ki,kd constants using Kp,Ki,Kd values set by us and sample time
		for i in range(3):
			self.kp[i]=self.Kp[i]
			#for integral term constant sample time is multipled as per integral formula
			self.ki[i]=self.Ki[i]*self.sample_time
			#for derivative constant sample time is divided as per derivative formula
			self.kd[i]=self.Kd[i]/self.sample_time
	#-----------------------------------------------------------------------------------------------------------------------------------
	'''
	* Function Name:SetSampleTime()
	* Input:None
	* Output:new ki and kd values are updated if a new sample time is used
	* Logic:First calculating the ratio between two times and applying in each axis using for loops
	* Example Call:self.SetSampleTime()
	'''
	def SetSampleTime(self):
		#if sample time is graeter than zero then it finds ratio
		if self.NewSampleTime > 0:
   			#finding ratio and setting sample time to new sample time finally again performing operations as done in SetTunings() method
			self.ratio=self.NewSampleTime/self.sample_time
			
			for i in range(3):
				self.ki[i] =self.Ki[i]*self.ratio
				self.kd[i]= self.Kd[i]/self.ratio
				self.sample_time = self.NewSampleTime
	#-------------------------------------------------------------------------------------------------------------------------------------------
	'''
	* Function Name:path()
	* Input:None
	* Output:Used to set setpoints according to the pathpoints published by /vrep_waypoints topic and publishes a flag or count to vrep via a topic called
	* * /status_flag which helps in planning next path
	* Logic: A falg is set whenever the error is less than 0.8 and last table entry value from /vrep_waypoints is utilised to setpoint.Using that flag and error<1 next setpoints from table
	* are set as setpoint.After the count is published flag is reset saying that path_points has to be again computed in vrep for next path.
	* Example Call:self.path()
	'''	
	def path(self):
			#This is our solution for whycon loose problem i.e. w.k.t if drone moves faster near hoop, then whycon loose doesnt pose a problem.And we have used a condition
			#in vrep such that whenver the path to be computed between the 2 poins we measure the distance in x and y if it is less than 0.01 in vrep world
			#hen path points are reduced so that drone moves faster this is he solution for our faster implementation of our task.
			if self.value==31:
				#31 path points are computed then we have skipped 20 points with initial point being indexed a 10 so that drone moves to 2 points only
				self.value1=50
				self.skip=20
				
				
			#else number of path points and skip points are increased	
			else:
				self.skip=10
			
				self.value1=50

			#Only if error in each axis is less than 1 then only next point from pathpoint table is accesed
			if self.error3[0]<1 and self.error3[1]<1 and self.error3[2]<1:
						#since we need to send count or flag to vrep for path plan we compute this only once untill last point from table is used as setpoint

						if self.flag2 == 1:
							self.count =self.count+1
							self.flagpub = self.count
							
							#Each count is published through /status_flag topic 
							self.flag_1.publish(self.flagpub)
							#again it is reset so that it no compued again till pathpoints are cleared
							self.flag2 = 0

						#Only if table is not empty path points are set as setpoints		
						if self.table != {}:
							#26 points are considered for path plan and self.i is incremented by skip points every iteration
							if self.i<self.value:
								#minus in x direction is used because our camera position is reversed in x direction
								self.setpoint[0]=-round(self.table.poses[self.i].position.x,1)
								self.setpoint[1]=round(self.table.poses[self.i].position.y,1)
								self.setpoint[2]=round(self.table.poses[self.i].position.z,1)
								self.i =self.i+self.skip
								
			#If last table point is accessed then we have to check whether drone as reached last setpoint hence we are comparing with 0.8 if it is true then flag is set for next path computation		
			if self.i ==self.value1 and self.error3[0]<1 and self.error3[1]<1 and self.error3[2]<1:
				self.flag2=1 
				#index value is initialised
				self.i =10
				#table is emptied
				self.table = {}
				#if count is 4 then the drone has reached home hence it is disarmed.This count is also manually found based on number of dummies
				if self.count == self.q:
					self.disarm()
	#---------------------------------------------------------------------------------------------------------------------------------------------------------
 	'''
	* Function Name:pid()
	* Input:None
	* Output:Publishes output values within max and min as said before after adding pid outputs to 1500 neutral value
	* Logic:First calling setsampletime() to compute new sample time if given and then tuning is 
	* performed by multiplying or dividing sample time with ki and kd values respectively finally a delay is called for 10ms to call compute() which computes result
	* Example Call:Calld explicitly by main() function
	'''  


	def pid(self):
			#each of the method declared above are called one by one with a delay of 10ms 

			self.SetSampleTime()
			#this method overwrites kp ki kd values with the SetSampleTime method however it is used for user convention to set sampling time if it is changed
			self.SetTunings()
			time.sleep(0.01)
			self.Compute()
			#result from Compute() method in each axis is added with 1500 neutral value to control drone motor speed as per motor mixing algorithm in plutox firmware
			self.cmd.rcPitch=1500+self.result[0]
			self.cmd.rcRoll=1500+self.result[1]
			self.cmd.rcThrottle=1500+self.result[2]
			
			#limitng the speed of drone in each axis if it exceeds maximum value or it is below minimum value
			if self.cmd.rcPitch>self.max_values[0]:
				self.cmd.rcPitch=self.max_values[0]

			elif self.cmd.rcPitch<self.min_values[0]:
				self.cmd.rcPitch=self.min_values[0]
				
			
			if self.cmd.rcRoll>self.max_values[1]:
				self.cmd.rcRoll=self.max_values[1]

			elif self.cmd.rcRoll<self.min_values[1]:
				self.cmd.rcRoll=self.min_values[1]
				
			  
			if self.cmd.rcThrottle>self.max_values[2]:
				self.cmd.rcThrottle=self.max_values[2]
					
			elif self.cmd.rcThrottle<self.min_values[2]:
				self.cmd.rcThrottle=self.min_values[2]
			
			#calling of path method which is declared above	
			
			self.path()
			#publishing motor speed via /drone_command topic 
			self.command_pub.publish(self.cmd)
			#this is the condition for whycon loose condition if whycon reading los for more han 50 iteration then pre flag is set to 0 in main method
			#since pid method is called everytime compared to whycon. 
			if self.pre==1:
				self.con =0
			if self.pre==0:
				self.con=self.con+1
				if self.con>50:
					#resetting the count to 0
					self.con=0
					#now that whycon reading is los we are taking drone poistion from /alt topic
					self.drone_position[2]=self.w
			
			
			
	#--------------------------------------------------------------------------------------------------------------------------------
			
	'''     
	* Function Name:__main__()
	* Input:None
	* Output:Calls pid node and creates class objects to use its functions
	* Logic:checks if rospy is not shutdown and then calls pid() using an instance
	* Example Call:None
	'''

if __name__ == '__main__':
#instance is created to call pid() method
	e_drone = Edrone()
	
#while main method is running mainflag is checked for 1 conditon then the pid node is started
	while not rospy.is_shutdown():
		#as said above pre flag is set to 0 after whycon callback 
		e_drone.pre=0
		#this is  the condition for drone to get arm and disarm whenever required without running the program again
		if e_drone.mainFlag==1 and e_drone.mainflag==0:
			e_drone.pid()
		#when key d is pressed program exits d is pressed inly when drone crashes so tha we can do reposition
		elif e_drone.mainflag==1:
			break


#*******************************************THANKS A LOT HOPING TO REACH THE FINALS AT IIT-B**************************************************

