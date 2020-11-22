#!/usr/bin/env python

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

class Edrone():
	def __init__(self):
		rospy.init_node("position_controller")

		self.set_loc = [19.0001,72.0000,1.0000]      
		self.current_loc=[0.0,0.0,0.0]

		self.prev_error = [0]
		self.error = Float32() # latitude,longitude,altitude
		self.error_p = [0.0]
		self.error_i = [0.0]
		self.error_d = [0.0]
		self.zero = Float32()
		self.prev_time = [0]
		self.now = 0   
		
		self.rpyt_cmd = edrone_cmd() # YOU HAVE MADE AN OBJECT OF EDRONE_CMD,BUT HAVE NOT MADE AN OBJECT OF FLOAT32 TO STORE ITS DATA
		self.rpyt_cmd.rcRoll = 0.0
		self.rpyt_cmd.rcPitch = 0.0
		self.rpyt_cmd.rcYaw = 0.0
		self.rpyt_cmd.rcThrottle = 0.0
		
		self.kp = [0.0]
		self.ki = [0.0]
		self.kd = [0.0]
		#JUST CHECK WHAT IS THE ERROR HERE IN INITIALIZING
		# THESE kp kd, ki are for storing original Kp,Kd,Ki when using sample_time
		
		self.Kp = [0]
		self.Ki = [0]
		self.Kd = [0]

		self.sample_time = 40 # 100 HZ SAMPLE TIME IS VERY LESS TIME,TRY BELOW 60 HZ

		self.out_altitude = Float32()
		self.zero =Float32()
		self.zero.data=0.0
		self.out_altitude.data=0.0  ######################## CORRECTION

		self.attitude_input_pub = rospy.Publisher('/drone_command', edrone_cmd,queue_size=1) 
		#self.latitude_error_pub = rospy.Publisher('/latitude_error',Float32,queue_size=1)
		#self.longitude_error_pub = rospy.Publisher('/longitude_error',Float32,queue_size=1)
		self.altitude_error_pub = rospy.Publisher('/altitude_error',Float32,queue_size=1)    # MAKE ITS OBJECT
		self.zero_error_pub = rospy.Publisher('/zero_error',Float32,queue_size=1)            # MAKE ITS OBJECT
		
		rospy.Subscriber('/edrone/gps', NavSatFix, self.drone_location)
		rospy.Subscriber('/pid_tuning_altitude',PidTune, self.altitude_set_pid)

	def drone_location(self,msg):
		self.current_loc[0] = msg.latitude
		self.current_loc[1] = msg.longitude
		self.current_loc[2] = msg.altitude
	   
	def altitude_set_pid(self, altitude):

		self.Kp[0] = altitude.Kp * 0.1  
		self.Ki[0] = altitude.Ki * 0.00
		self.Kd[0] = altitude.Kd * 2

	
	def pid_p(self): 

		self.now = time.time()*1000		
		self.time_change = self.now - self.prev_time[0]
		if self.time_change >= self.sample_time:
# THERE SHOULD BE A SAMPLE TIME FOR PID ALSO,WHICH MUST BE LESS THAN SAMPLE TIME OF YOUR LOOP,TRY AROUND 0.01 SECS
						 #ALSO TRY THE BELOW LINKS FOR PERFECT PID CODE
						 #http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
						 #http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-sample-time/
			self.error = self.set_loc[2] - self.current_loc[2]

			self.error_p[0] = self.error*self.Kp[0]
			self.error_d[0] = (self.error-self.prev_error[0])*self.Kd[0]  
			self.error_i[0] = (self.error+self.error_i[0])*self.Ki[0]

			self.out_altitude.data=self.error            #################### CORRECTED

			PID_ALT = self.error_p[0]+self.error_i[0]+self.error_d[0]   #################### CORRECTED
			self.out_throttle = 1500.00 + PID_ALT       ###################### CORRECTED 

			if self.out_throttle>2000:
				self.out_throttle = 2000
			elif self.out_throttle<1000:
				self.out_throttle = 1000

			self.rpyt_cmd = edrone_cmd()
			self.rpyt_cmd.rcRoll= 1500.0
			self.rpyt_cmd.rcYaw = 1500.0
			self.rpyt_cmd.rcPitch = 1500.0
			self.rpyt_cmd.rcThrottle = self.out_throttle

			
			self.prev_error[0] = self.error
			self.zero.data = 0
			self.prev_time[0] = self.now
			
			self.attitude_input_pub.publish(self.rpyt_cmd)
			#self.latitude_error_pub.publish(self.out_latitude)            
			#self.longitude_error_pub.publish(self.out_longitude)
			self.altitude_error_pub.publish(self.out_altitude) ############## NOW IT CAM BE PUBLISHED 
																	   
			self.zero_error_pub.publish(self.zero)

	


if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid_p()
		r.sleep()
