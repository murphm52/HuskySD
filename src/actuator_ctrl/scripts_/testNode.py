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

def init():
	rospy.init_node('test_node', anonymous=True)

def callbackPath(data):
	#rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)
#	global dis
#	global dcyc
	vec = np.array(data.data)
	dis = vec[0,1,2,3,4,5]
	dcyc = vec[6,7,8,9,10,11]
	print("Motor 1 set to duty cycle {:>5.3f}." .format(dcyc[0]))
	print("Motor 2 set to duty cycle {:>5.3f}." .format(dcyc[1]))
	print("Motor 3 set to duty cycle {:>5.3f}." .format(dcyc[2]))
	print("Motor 4 set to duty cycle {:>5.3f}." .format(dcyc[3]))
	print("Motor 5 set to duty cycle {:>5.3f}." .format(dcyc[4]))
	print("Motor 6 set to duty cycle {:>5.3f}." .format(dcyc[5]))
	print("{:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f}".format(dis)) # prints analog voltage for troubleshooting

def sendPWM():
	rospy.Subscriber('path', String, callbackPath)
#	rate = rospy.Rate(10)  # 10 Hz
	rospy.spin()


if __name__=='__main__':
	try:
		init()
		sendPWM() # Go into the function itself to enable/disable specific actuators.
	except rospy.ROSInterruptException:
		pass # "pass" makes sure the node exits without issues
