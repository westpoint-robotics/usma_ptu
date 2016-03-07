#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import UInt8
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64



twistMsg = Twist()
g = 0
x = 0
y = 0
tapcount=0

onoff = False

#fist: 		1
#finger spread:	4
#wave left:	2
#wave right: 	3
#double tap: 	5

def talker():
	global g
	global x
	rate = rospy.Rate(10)
	pub = rospy.Publisher('gvr_bot/cmd_vel', Twist, queue_size=10)

	if (g == 0):
		twistMsg.linear.x = 0
		twistMsg.angular.z = 0.0
	elif (g == 3):
		print "turn right"
		twistMsg.angular.z = -0.1
	elif (g == 2):
		print "turn left"
		twistMsg.angular.z = 0.1

	
	if (x > -0.2 and x <0.2):
		print "go forward"
		twistMsg.linear.x = .2
	if (x > 0.8 and x <1.0):
		print "go backward"
		twistMsg.linear.x = -.2

	pub.publish(twistMsg)
	print "Hello! Welcome to Soldier Machine Interaction."


def callbackGest(data):
	global g
	global onoff
	g = data.data
	global tapcount
	#rospy.loginfo(x);
	if (g == 5):
		tapcount +=1
	if (tapcount%2==1):
		onoff = True
	if (tapcount%2==0):
		onoff = False
	if (onoff):
		print "it's on"
 		talker()
	else:
		print "it's off"
		   

def callbackImu(data):
	global x
	global onoff
	x = data.linear_acceleration.x
	if (onoff):
		print "it's on"
 		talker()
	else:
		print "it's off"




def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/myo_gest", UInt8, callbackGest)
	rospy.Subscriber("/myo_imu", Imu, callbackImu)
	rospy.spin()


if __name__ == '__main__':
	listener()
