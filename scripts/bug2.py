#! /usr/bin/env python

# import ros stuff
import rospy
# import ros message
from vitarana_drone.msg import*
from pid_tune.msg import PidTune
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
# import ros service
from std_srvs.srv import *

import math

class Edrone():

    srv_client_go_to_point_ = None
    srv_client_wall_follower_ = None

    yaw_ = 0
    pitch_ = 0
    roll_ = 0
    throttle_ = 0

    yaw_error_allowed_ = 2 * (math.pi / 180) # 2 degrees
    pitch_error_allowed_ = 2 * (math.pi / 180)
    roll_error_allowed_ = 2 * (math.pi / 180)
    throttle_error_allowed_ = 2 * (math.pi / 180)

    #robot state variables
    position_ = Point()
    initial_position_ = Point()
    initial_position_.x = rospy.get_param('initial_x')
    initial_position_.y = rospy.get_param('initial_y')
    initial_position_.z = rospy.get_param('initial_z')

    #goal
    desired_position_ = Point()
    desired_position_.x = rospy.get_param('des_pos_x')
    desired_position_.y = rospy.get_param('des_pos_y')
    desired_position_.z = rospy.get_param('des_pos_z')

    regions_ = None
    state_desc_ = ['Go to point', 'wall following']
    state_ = 0
    count_state_time_ = 0 # seconds the robot is in a state
    count_loop_ = 0
    # 0 - go to point
    # 1 - wall following

    # callbacks
    def clbk_odom(msg):
        global position_, yaw_, pitch_, roll_, throttle_
        
        # position
        position_ = msg.pose.pose.position
        
        # yaw, pitch, roll, throttle
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        yaw_ = euler[2]
        pitch_ = euler[1]
        roll_ = euler[0]
        throttle_ = euler[3]

    def clbk_laser(msg):
        global regions_
        regions_ = {
            'right':  min(min(msg.ranges[0:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'fleft':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:719]), 10),
        }

    def change_state(state):
        global state_, state_desc_
        global srv_client_wall_follower_, srv_client_go_to_point_
        global count_state_time_
        count_state_time_ = 0
        state_ = state
        log = "state changed: %s" % state_desc_[state]
        rospy.loginfo(log)
        if state_ == 0:
            resp = srv_client_go_to_point_(True)
            resp = srv_client_wall_follower_(False)
        if state_ == 1:
            resp = srv_client_go_to_point_(False)
            resp = srv_client_wall_follower_(True)

    def distance_to_line(p0):
        # p0 is the current position
        # p1 and p2 points define the line
        global initial_position_, desired_position_
        p1 = initial_position_
        p2 = desired_position_
        # here goes the equation
        up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x)) #check this equation in net
        lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
        distance = up_eq / lo_eq
        
        return distance
        

    def normalize_angle(angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def main():
        global regions_, position_, desired_position_, state_, yaw_,pitch_,roll_,throttle_, yaw_error_allowed_, pitch_error_allowed_, roll_error_allowed_, throttle_error_allowed_
        global srv_client_go_to_point_, srv_client_wall_follower_
        global count_state_time_, count_loop_
        
        rospy.init_node('bug0')
        
        sub_laser = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)
        sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
        
        rospy.wait_for_service('/go_to_point_switch')
        rospy.wait_for_service('/wall_follower_switch')
        rospy.wait_for_service('/gazebo/set_model_state')
        
        srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
        srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
        srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # set robot position
        model_state = ModelState()
        model_state.model_name = 'm2wr'
        model_state.pose.position.x = initial_position_.x
        model_state.pose.position.y = initial_position_.y
        model_state.pose.position.z = initial_position_.z
        resp = srv_client_set_model_state(model_state)
        
        # initialize going to the point
        change_state(0)
        
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if regions_ == None:
                continue
            
            distance_position_to_line = distance_to_line(position_)
            
            if state_ == 0:
                if regions_['front'] > 0.15 and regions_['front'] < 1:
                    change_state(1)
            
            elif state_ == 1:
                if count_state_time_ > 5 and \
                distance_position_to_line < 0.1:
                    change_state(0)
                    
            count_loop_ = count_loop_ + 1
            if count_loop_ == 20:
                count_state_time_ = count_state_time_ + 1
                count_loop_ = 0
                
            rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", distance_to_line(position_), position_.x, position_.y, position_.z)
            rate.sleep()

if __name__ == "__main__":
    
    e_drone = Edrone()
	r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid_p()
		r.sleep()
    main()