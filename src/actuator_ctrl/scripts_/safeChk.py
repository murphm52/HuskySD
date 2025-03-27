#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Reads potentiometer, current, and IMU readings to determine if any data present dangerous behavior.
# If dangerous behavior is detected, corrections or shutdown will be executed.
#
#

import rospy
import time
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import numpy as np

# Time loop variables
intv = 1000
tLast = 0

CSpub = rospy.Publisher('CS_sc', String, queue_size=10) # CS_sc = Current Sensor safety check
PTpub = rospy.Publisher('PT_sc', String, queue_size=10)

def callbackPT(data):
	global PTpub
	PT = np.array(data.data)
	PT1 = PT[0]
	PT2 = PT[1]
	PT3 = PT[2]
	PT4 = PT[3]
	PT5 = PT[4]
	PT6 = PT[5]
#	print("{:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f}".format(PT1,PT2,PT3,PT4,PT5,PT6))
#	return [PT1,PT2,PT3,PT4,PT5,PT6]
	PTpub.publish("coconut.jpg")
	print("PT data sent")

def callbackCS(data):
	global CSpub
	CS = np.array(data.data)
	CS1 = CS[0]
	CS2 = CS[1]
	CS3 = CS[2]
	CS4 = CS[3]
	CS5 = CS[4]
	CS6 = CS[5]
#	return [CS1,CS2,CS3,CS4,CS5,CS6]
	CSpub.publish("coconut.jpg")
	print("CS data sent")

def callbackIMUe(data): #euler angle
	print("{:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f}".format(data)) # prints analog voltage for troubleshooting

def callbackIMUq(data): #quaternion
	print("{:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f}".format(data)) # prints analog voltage for troubleshooting

def primary():
	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('safe_node', anonymous=True)
	rospy.Subscriber('potent', Float32MultiArray, callbackPT)
	rospy.Subscriber('current', Float32MultiArray, callbackCS)
	rospy.spin()


if __name__ == '__main__':
	try:
		primary()
	except rospy.ROSInterruptException:
		pass
