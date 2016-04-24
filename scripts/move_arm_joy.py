#!/usr/bin/env python

PKG = 'usma_ptu'

import roslib; roslib.load_manifest(PKG)

import time
from math import pi
from threading import Thread

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from dynamixel_controllers.srv import *
from dynamixel_msgs.msg import JointState as JointState

class MoveArm():
    def __init__(self):
        self.is_running = True
        self.step_size = 2.0 * 3.14 / 180.0
        self.joy_data = None
        self.prev_time = time.time()
        
        rospy.init_node('move_arm_joy', anonymous=True)
	self.shoulder_pan_axis = rospy.get_param('/axes_map/shoulder_pan_axis')
	self.shoulder_flex_axis = rospy.get_param('/axes_map/shoulder_flex_axis')
	self.elbow_flex_axis = rospy.get_param('/axes_map/elbow_flex_axis')
	self.wrist_flex_axis = rospy.get_param('/axes_map/wrist_flex_axis')
	self.wrist_roll_axis = rospy.get_param('/axes_map/wrist_roll_axis')
	self.gripper_axis = rospy.get_param('/axes_map/gripper_axis')

        rospy.Subscriber('/joy', Joy, self.read_joystick_data)
        self.shoulder_pan_pub = rospy.Publisher('/shoulder_pan_joint_controller/command', Float64, queue_size=10)
        self.shoulder_flex_pub = rospy.Publisher('/shoulder_flex_joint_controller/command', Float64, queue_size=10)
        self.elbow_flex_pub = rospy.Publisher('/elbow_flex_joint_controller/command', Float64, queue_size=10)
        self.wrist_flex_pub = rospy.Publisher('/wrist_flex_joint_controller/command', Float64, queue_size=10)
        self.wrist_roll_pub = rospy.Publisher('/wrist_roll_joint_controller/command', Float64, queue_size=10)
        self.gripper_pub = rospy.Publisher('/gripper_controller/command', Float64, queue_size=10)

        self.shoulder_pan_joint = 0.0
        self.shoulder_flex_joint = 0.0
        self.elbow_flex_joint = 0.0
        self.wrist_flex_joint = 0.0
        self.wrist_roll_joint = 0.0
        self.gripper = 0.0

    def read_joystick_data(self, data):
        self.joy_data = data
        cur_time = time.time()
        timediff = cur_time - self.prev_time
        self.prev_time = cur_time

    def update_arm_position(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.joy_data:
		self.shoulder_pan_joint += 1 * self.joy_data.axes[self.shoulder_pan_axis] * self.step_size
		self.shoulder_flex_joint += -1 * self.joy_data.axes[self.shoulder_flex_axis] * self.step_size
		self.elbow_flex_joint += 1 * self.joy_data.axes[self.elbow_flex_axis] * self.step_size
		self.wrist_flex_joint += 1 * self.joy_data.axes[self.wrist_flex_axis] * self.step_size
		self.wrist_roll_joint += -1 * self.joy_data.axes[self.wrist_roll_axis] * self.step_size
		self.gripper += -1 * self.joy_data.axes[self.gripper_axis] * self.step_size
	    	self.shoulder_pan_pub.publish(self.shoulder_pan_joint)
	    	self.shoulder_flex_pub.publish(self.shoulder_flex_joint)
	    	self.elbow_flex_pub.publish(self.elbow_flex_joint)
	    	self.wrist_flex_pub.publish(self.wrist_flex_joint)
	    	self.wrist_roll_pub.publish(self.wrist_roll_joint)
	    	self.gripper_pub.publish(self.gripper)
	    rate.sleep()

if __name__ == '__main__':
    try:
        move_arm = MoveArm()
	move_arm.update_arm_position()
    except rospy.ROSInterruptException: 
	pass
