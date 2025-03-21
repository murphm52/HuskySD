#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Outputs a PWM signals to six separate motor drivers
# Communicates with a node that reads the current sense (CS) value from the VNH5019.
# 

import rospy
import time
import board
import busio
import RPi.GPIO as GPIO
from std_msgs.msg import Float32 
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import String 
from adafruit_pca9685 import PCA9685
import numpy as np

def init():
	rospy.init_node('exec_node', anonymous=True)

def callbackPath(data):
#	global dis
#	global dcyc
	vec = np.array(data.data)
	dis = vec[0:6]
	dcyc = vec[6:12]
	print("Motor 1 set to duty cycle {:>5.3f}." .format(dcyc[0]))
	print("Motor 2 set to duty cycle {:>5.3f}." .format(dcyc[1]))
	print("Motor 3 set to duty cycle {:>5.3f}." .format(dcyc[2]))
	print("Motor 4 set to duty cycle {:>5.3f}." .format(dcyc[3]))
	print("Motor 5 set to duty cycle {:>5.3f}." .format(dcyc[4]))
	print("Motor 6 set to duty cycle {:>5.3f}." .format(dcyc[5]))
	print(format(dis)) # prints analog voltage for troubleshooting
#	rospy.loginfo(data.data)

def sendPWM():
	rospy.Subscriber('path', Float32MultiArray, callbackPath)
#	rate = rospy.Rate(10)  # 10 Hz
	rospy.spin()


if __name__=='__main__':
	try:
		init()
		sendPWM() # Go into the function itself to enable/disable specific actuators.
	except rospy.ROSInterruptException:
		pass # "pass" makes sure the node exits without issues
