#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
		PUBLICATIONS            SUBSCRIPTIONS
		/roll_error             /pid_tuning_altitude
		/pitch_error            /pid_tuning_pitch
		/yaw_error              /pid_tuning_roll
		/edrone/pwm             /edrone/imu/data
								/edrone/drone_command

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		rospy.init_node('attitude_controller')  # initializing ros node with name drone_control

		# This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
		# [x,y,z,w]
		self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

		# This corresponds to your current orientation of eDrone converted in euler angles form.
		# [r,p,y]
		self.drone_orientation_euler = [0.0, 0.0, 0.0]

		# This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
		# [r_setpoint, p_setpoint, y_setpoint]
		self.setpoint_cmd = [0.0, 0.0, 0.0]

		# The setpoint of orientation in euler angles at which you want to stabilize the drone
		# [r_setpoint, p_psetpoint, y_setpoint]
		self.setpoint_euler = [0.0, 0.0, 0.0]

		# Declaring pwm_cmd of message type prop_speed and initializing values
		# Hint: To see the message structure of prop_speed type the following command in the terminal
		# rosmsg show vitarana_drone/prop_speed

		self.pwm_cmd = prop_speed()
		self.pwm_cmd.prop1 = 0.0
		self.pwm_cmd.prop2 = 0.0
		self.pwm_cmd.prop3 = 0.0
		self.pwm_cmd.prop4 = 0.0

		# initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
		# after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [0, 0, 0]
		self.Ki = [0, 0, 0]
		self.Kd = [0, 0, 0]
		# -----------------------Add other required variables for pid here ----------------------------------------------
		#
		self.thrust = 0.0
		self.prev_error = [0.0,0.0,0.0]
		self.max_values = [1024,1024,1024,1024]
		self.min_values = [0,0,0,0]
		self.error = [0.0,0.0,0.0]
		self.error_p = [0.0,0.0,0.0]
		self.error_i = [0.0,0.0,0.0]
		self.error_d = [0.0,0.0,0.0]
		self.prev_time = [0]
		self.now = 0
		self.out_roll = Float32()
		self.out_pitch = Float32()
		self.out_yaw = Float32()

		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, yaw]
		#        Add variables for limiting the values like self.max_values = [1024, 1024, 1024, 1024] corresponding to [prop1, prop2, prop3, prop4]
		#                                                   self.min_values = [0, 0, 0, 0] corresponding to [prop1, prop2, prop3, prop4]
		#
		# ----------------------------------------------------------------------------------------------------------
		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 20 # in seconds                                 ###################### TRY A SAMPLE FREQUENCY OF LESS THAN THAT OF POSITION CONTROLLER'S LOOP
																			
		# Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
		self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
		self.roll_error_pub = rospy.Publisher('/roll_error',Float32,queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error',Float32,queue_size=1)
		self.yaw_error_pub = rospy.Publisher('/yaw_error',Float32,queue_size=1)
		# -----------------------------------------------------------------------------------------------------------

		# Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
		rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
		rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
		# -------------------------Add other ROS Subscribers here-------
		rospy.Subscriber('/pid_tuning_pitch',PidTune, self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_yaw',PidTune, self.yaw_set_pid)
		# ------------------------------------------------------------------------------------------------------------

	# Imu callback function
	# The function gets executed each time when imu publishes /edrone/imu/data

	# Note: The imu publishes various kind of data viz angular velocity, linear acceleration, magnetometer reading (if present),
	# but here we are interested in the orientation which can be calculated by a complex algorithm called filtering which is not in the scope of this task,
	# so for your ease, we have the orientation published directly BUT in quaternion format and not in euler angles.
	# We need to convert the quaternion format to euler angles format to understand the orienataion of the edrone in an easy manner.
	# Hint: To know the message structure of sensor_msgs/Imu, execute the following command in the terminal
	# rosmsg show sensor_msgs/Imu

	def imu_callback(self, msg):

		self.drone_orientation_quaternion[0] = msg.orientation.x
		self.drone_orientation_quaternion[1] = msg.orientation.y
		self.drone_orientation_quaternion[2] = msg.orientation.z
		self.drone_orientation_quaternion[3] = msg.orientation.w

		# --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

	def drone_command_callback(self, msg):
		self.setpoint_cmd[0] = msg.rcRoll
		self.setpoint_cmd[1] = msg.rcPitch
		self.setpoint_cmd[2] = msg.rcYaw
		self.thrust = msg.rcThrottle         

		# ---------------------------------------------------------------------------------------------------------------

	# Callback function for /pid_tuning_roll
	# This function gets executed each time when /tune_pid publishes /pid_tuning_roll
	def roll_set_pid(self, roll):
		self.Kp[0] = roll.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = roll.Ki * 0.008
		self.Kd[0] = roll.Kd * 0.3

	# ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------
	def pitch_set_pid(self, pitch):
		self.Kp[1] = pitch.Kp * 0.06  
		self.Ki[1] = pitch.Ki * 0.008
		self.Kd[1] = pitch.Kd * 0.3

	def yaw_set_pid(self, yaw):
		self.Kp[2] = yaw.Kp * 0.06 
		self.Ki[2] = yaw.Ki * 0.008
		self.Kd[2] = yaw.Kd * 0.3
	# ----------------------------------------------------------------------------------------------------------------------

	def pid(self):

		self.now = time.time()*1000
		self.time_change = self.now - self.prev_time[0]
		if self.time_change >= self.sample_time:

			# Converting quaternion to euler angles
			(self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])
			

			self.sample_time = 20 # GIVE A SAMLE TIME TO YOUR PID,LESS THAN THE TIME OF YOUR LOOP

			# Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis
			self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
			self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
			self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30

			#error in each axis
			self.error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0] # for roll
			self.error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1] # for pitch
			self.error[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2] # for yaw

			   
			#calculating error for proportional
			self.error_p[0] = self.error[0]*self.Kp[0] #roll
			self.error_p[1] = self.error[1]*self.Kp[1] #pitch
			self.error_p[2] = self.error[2]*self.Kp[2] #yaw

			#calculating error for derivative
			self.error_d[0] = (self.error[0]-self.prev_error[0])*self.Kd[0] #roll
			self.error_d[1] = (self.error[1]-self.prev_error[1])*self.Kd[1] #pitch           ##################### CONSIDER TIME
			self.error_d[2] = (self.error[2]-self.prev_error[2])*self.Kd[2] #yaw

			#calculating eror in integral
			self.error_i[0] = (self.error[0]+self.error_i[0])*self.Ki[0] #roll
			self.error_i[1] = (self.error[1]+self.error_i[1])*self.Ki[1] #pitch              ##################### CONSIDER TIME
			self.error_i[2] = (self.error[2]+self.error_i[2])*self.Ki[2] #yaw

			#pid error
			self.out_roll = self.error_p[0]+self.error_i[0]+self.error_d[0]
			self.out_pitch = self.error_p[1]+self.error_i[1]+self.error_d[1]
			self.out_yaw = self.error_p[2]+self.error_i[2]+self.error_d[2]

			#converting throtle speed from 1000-2000 to 0-1024
			self.thrust = self.thrust*1.024-1024
			#computing pwm prop speed
			self.pwm_cmd.prop1 = self.thrust + self.out_yaw + self.out_pitch + self.out_roll
			self.pwm_cmd.prop2 = self.thrust - self.out_yaw - self.out_pitch + self.out_roll
			self.pwm_cmd.prop3 = self.thrust + self.out_yaw - self.out_pitch - self.out_roll
			self.pwm_cmd.prop4 = self.thrust - self.out_yaw + self.out_pitch - self.out_roll

			#limiting speed
			if self.pwm_cmd.prop1 > self.max_values[0]:
				self.pwm_cmd.prop1 = self.max_values[0]
			elif self.pwm_cmd.prop1 < self.min_values[0]:
				self.pwm_cmd.prop1 = self.min_values[0]
			
			if self.pwm_cmd.prop2 > self.max_values[1]:
				self.pwm_cmd.prop2 = self.max_values[1]
			elif self.pwm_cmd.prop2 < self.min_values[1]:
				self.pwm_cmd.prop2 = self.min_values[1]
			
			if self.pwm_cmd.prop3 > self.max_values[2]:
				self.pwm_cmd.prop3 = self.max_values[2]
			elif self.pwm_cmd.prop3 < self.min_values[2]:
				self.pwm_cmd.prop3 = self.min_values[2]
			
			if self.pwm_cmd.prop4 > self.max_values[3]:
				self.pwm_cmd.prop4 = self.max_values[3]
			elif self.pwm_cmd.prop4 < self.min_values[3]:
				self.pwm_cmd.prop4 = self.min_values[3]

			#updating previous error
			self.prev_error[0]=self.error[0]
			self.prev_error[1]=self.error[1]
			self.prev_error[2]=self.error[2]

			self.prev_time[0] = self.now

			self.pwm_pub.publish(self.pwm_cmd)
			self.roll_error_pub.publish(self.out_roll)
			self.pitch_error_pub.publish(self.out_pitch)
			self.yaw_error_pub.publish(self.out_yaw)

if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()
