#! /usr/bin/env python

# import ros stuff
import rospy
from vitarana_drone.msg import*
from pid_tune.msg import PidTune
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

class Edrone():

    active_ = False

    # robot state variables
    position_ = Point()
    yaw_ = 0
    pitch_ = 0
    roll_ = 0
    throttle_ = 0
    # machine state
    state_ = 0
    # goal
    desired_position_ = Point()
    desired_position_.x = rospy.get_param('des_pos_x')
    desired_position_.y = rospy.get_param('des_pos_y')
    desired_position_.z = rospy.get_param('des_pos_z')
    # parameters
    yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
    pitch_precision_ = math.pi / 90 
    roll_precision_ = math.pi / 90
    throttle_precision_ = math.pi / 90
    dist_precision_ = 0.3

    # publishers
    pub = None

    # service callbacks
    def go_to_point_switch(req):
        global active_
        active_ = req.data
        res = SetBoolResponse()
        res.success = True
        res.message = 'Done!'
        return res

    # callbacks
    def clbk_odom(msg):
        global position_
        global yaw_, pitch_, roll_,throttle_
        
        # position
        position_ = msg.pose.pose.position
        
        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        roll_ = euler[0]
        pitch_ = euler[1]
        yaw_ = euler[2]
        throttle_ = euler[3]

    def change_state(state):
        global state_
        state_ = state
        print ('State changed to [%s]' % state_)

    def normalize_angle(angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def fix_roll(des_pos):
        global roll_, pub, roll_precision_, state_
        desired_roll = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
        err_roll = normalize_angle(desired_roll - roll_)
        
        rospy.loginfo(err_roll)
        
        twist_msg = Twist()
        if math.fabs(err_roll) > roll_precision_:
            twist_msg.angular.z = 0.7 if err_roll > 0 else -0.7
        
        pub.publish(twist_msg)
        
        # state change conditions
        if math.fabs(err_roll) <= roll_precision_:
            print 'Roll error: [%s]' % err_roll
            change_state(1)

    def fix_pitch(des_pos):
        global pitch_, pub, pitch_precision_, state_
        desired_pitch = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
        err_pitch = normalize_angle(desired_pitch - pitch_)
        
        rospy.loginfo(err_pitch)
        
        twist_msg = Twist()
        if math.fabs(err_pitch) > pitch_precision_:
            twist_msg.angular.z = 0.7 if err_pitch > 0 else -0.7
        
        pub.publish(twist_msg)
        
        # state change conditions
        if math.fabs(err_yaw) <= pitch_precision_:
            print 'Pitch error: [%s]' % err_pitch
            change_state(1)

    def fix_yaw(des_pos):
        global yaw_, pub, yaw_precision_, state_
        desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
        err_yaw = normalize_angle(desired_yaw - yaw_)
        
        rospy.loginfo(err_yaw)
        
        twist_msg = Twist()
        if math.fabs(err_yaw) > yaw_precision_:
            twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
        
        pub.publish(twist_msg)
        
        # state change conditions
        if math.fabs(err_yaw) <= yaw_precision_:
            print 'Yaw error: [%s]' % err_yaw
            change_state(1)

    def fix_throttle(des_pos):
        global throttle_, pub, throttle_precision_, state_
        desired_throttle = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
        err_throttle = normalize_angle(desired_throttle - throttle_)
        
        rospy.loginfo(err_throttle)
        
        twist_msg = Twist()
        if math.fabs(err_throttle) > throttle_precision_:
            twist_msg.angular.z = 0.7 if err_throttle > 0 else -0.7
        
        pub.publish(twist_msg)
        
        # state change conditions
        if math.fabs(err_throttle) <= throttle_precision_:
            print 'Throttle error: [%s]' % err_throttle
            change_state(1)

    def go_straight_ahead(des_pos):
        global yaw_, pub, yaw_precision_, state_
        desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
        err_yaw = desired_yaw - yaw_
        err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
        
        if err_pos > dist_precision_:
            twist_msg = Twist()
            twist_msg.linear.x = 0.6
            twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
            pub.publish(twist_msg)
        else:
            print 'Position error: [%s]' % err_pos
            change_state(2)
        
        # state change conditions
        if math.fabs(err_yaw) > yaw_precision_:
            print 'Yaw error: [%s]' % err_yaw
            change_state(0)

    def done():
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)

    def main():
        global pub, active_
        
        rospy.init_node('go_to_point')
        
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
        
        srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)
        
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not active_:
                continue
            else:
                if state_ == 0:
                    fix_yaw(desired_position_)
                elif state_ == 1:
                    go_straight_ahead(desired_position_)
                elif state_ == 2:
                    done()
                else:
                    rospy.logerr('Unknown state!')
            
            rate.sleep()

if __name__ == '__main__':
    
    e_drone = Edrone()
	r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid_p()
		r.sleep()
    main()