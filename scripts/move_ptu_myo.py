#!/usr/bin/env python

PKG = 'usma_ptu'

import roslib; roslib.load_manifest(PKG)

import time
from math import pi
from threading import Thread

import rospy
from std_msgs.msg import UInt8
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from dynamixel_controllers.srv import *
from dynamixel_msgs.msg import JointState as JointState

class MovePTU():
    def __init__(self):
        self.is_running = True
        self.step_size = 1.0 * 3.14 / 180.0
        self.prev_time = time.time()

	self.g = 0
	self.x = 0
	self.y = 0
	self.tapcount=0
	self.onoff = False
        
        rospy.init_node('move_ptu_myo', anonymous=True)
	self.pan_axis = rospy.get_param('/axes_map/pan')
	self.tilt_axis = rospy.get_param('/axes_map/tilt')

        rospy.Subscriber('/myo_gest', UInt8, self.read_myo_gest)
        rospy.Subscriber('/myo_imu', Imu, self.read_myo_imu)

        self.pan_joint = 0.0
	self.tilt_joint = 0.0

        self.servo_position_pan = rospy.Publisher('/pan_controller/command', Float64)
        self.servo_position_tilt = rospy.Publisher('/tilt_controller/command', Float64)

    def read_myo_gest(self, data):
	self.g = data.data
	if (self.g == 5):
		self.tapcount +=1
	if (self.tapcount%2==1):
		self.onoff = True
	if (self.tapcount%2==0):
		self.onoff = False

    def read_myo_imu(self,data):
	self.x = data.linear_acceleration.x

    def update_ptu_position(self):
        while self.is_running:
            if self.onoff:

                if (self.g == 3):
		    self.pan_joint += 1 * self.step_size
		elif (self.g == 2):
		    self.pan_joint -= 1 * self.step_size
		if (self.x > -0.2 and self.x <0.2):
		    self.tilt_joint += 1 * self.step_size
		if (self.x > 0.8 and self.x <1.0):
		    self.tilt_joint -= 1 * self.step_size

	    self.servo_position_pan.publish(self.pan_joint)
	    self.servo_position_tilt.publish(self.tilt_joint)
            time.sleep(0.05)

if __name__ == '__main__':
    try:
        move_ptu = MovePTU()
        t = Thread(target=move_ptu.update_ptu_position)
        t.start()
        rospy.spin()
        move_ptu.alive = False
        t.join()
    except rospy.ROSInterruptException: pass

