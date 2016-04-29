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

class MovePTU():
    def __init__(self):
        self.is_running = True
        self.step_size = 1.0 * 3.14 / 180.0
        self.joy_data = None
        self.prev_time = time.time()
        
        rospy.init_node('move_ptu_joy', anonymous=True)
        self.pan_axis = rospy.get_param('/axes_map/pan')
        self.tilt_axis = rospy.get_param('/axes_map/tilt')

        rospy.Subscriber('/joy', Joy, self.read_joystick_data)
        self.servo_position_pan = rospy.Publisher('/pan_controller/command', Float64)
        self.servo_position_tilt = rospy.Publisher('/tilt_controller/command', Float64)

        self.pan_joint = 0.0
        self.tilt_joint = 0.0

    def read_joystick_data(self, data):
        self.joy_data = data
        cur_time = time.time()
        timediff = cur_time - self.prev_time
        self.prev_time = cur_time

    def update_ptu_position(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.joy_data:
                self.pan_joint += 1 * self.joy_data.axes[self.pan_axis] * self.step_size
                self.tilt_joint += 1 * self.joy_data.axes[self.tilt_axis] * self.step_size
                self.servo_position_pan.publish(self.pan_joint)
                self.servo_position_tilt.publish(self.tilt_joint)
	        rate.sleep()

if __name__ == '__main__':
    try:
        move_ptu = MovePTU()
        move_ptu.update_ptu_position()
    except rospy.ROSInterruptException: pass
