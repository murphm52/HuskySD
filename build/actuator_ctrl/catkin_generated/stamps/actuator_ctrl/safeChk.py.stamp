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
import numpy as np

def conv6(data): # Converts a data.data matrix of length 6 into 6 individual floats
	print("PT{} = {:>5}\t{:>5.3f}" .format(chanNum, chan.value, chan.voltage)) 
	# This prints the value and voltage
	d = np.array(data.data)
	d1 = d[0]
	d2 = d[1]
	d3 = d[2]
	d4 = d[3]
	d5 = d[4]
	d6 = d[5]
	return [d1,d2,d3,d4,d5,d6]

def callbackPT(data):
	PT = np.array(data.data)
	PT1 = PT[0]
	PT2 = PT[1]
	PT3 = PT[2]
	PT4 = PT[3]
	PT5 = PT[4]
	PT6 = PT[5]
	print("PT data received")
	#print("{:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f}".format(PT1,PT2,PT3,PT4,PT5,PT6)) # prints analog voltage for troubleshooting

def callbackCS(data):
	CS = np.array(data.data)
	CS1 = CS[0]
	CS2 = CS[1]
	CS3 = CS[2]
	CS4 = CS[3]
	CS5 = CS[4]
	CS6 = CS[5]
	print("CS data received")
	#print("{:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f}".format(CS1,CS2,CS3,CS4,CS5,CS6)) # prints analog voltage for troubleshooting

def callbackIMUe(data):
#	rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
#	print("{:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f}".format(data)) # prints analog voltage for troubleshooting
	print("IMU Euler angle data recieved")
	
def callbackIMUq(data):
#	rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
#	print("{:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f}".format(data)) # prints analog voltage for troubleshooting
	print("IMU quaternion data recieved")
	
def listen():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('safe_node', anonymous=True)
	print("PT1:   PT2:   PT3:   PT4:   PT5:   PT6:")
	rospy.Subscriber('potent', Float32MultiArray, callbackPT)
	rospy.Subscriber('current', Float32MultiArray, callbackCS)
	rospy.Subscriber('eulerAng', Float32MultiArray, callbackIMUe)
#	rospy.Subscriber('quatAng', Float32MultiArray, callbackIMUq)
	rospy.spin() # spin() simply keeps python from exiting until this node is stopped
	
if __name__ == '__main__':
	try:
		listen()
	except rospy.ROSInterruptException:
		pass
