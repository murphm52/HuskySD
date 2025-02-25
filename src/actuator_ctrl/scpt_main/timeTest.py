#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Outputs a PWM signal with a constant duty cycle to a VNH5019 motor driver carrer.
# Communicates with a node that reads the current sense (CS) value from the VNH5019.
# 

import rospy
import time
import board
import busio
import RPi.GPIO as GPIO
from std_msgs.msg import Float32 
from std_msgs.msg import Int32
from adafruit_pca9685 import PCA9685

# Time loop variables
intv = 2000
intv2 = 1000
tLast = 0
tLast2 = 0


def timeTest():
	direc = int(1)
	global tLast
	global tLast2
	while not rospy.is_shutdown():
		tNow = int(time.time()*1000)
		tNow2 = int(time.time()*1000)
		if (tNow - tLast) > intv:
			tLast = tNow
			if direc > 0:
				direc = -direc
				print(direc)
			else:
				direc = -direc
				print(direc)
		
		if (tNow2 - tLast2) > intv2:
			tLast2 = tNow2
			print("Current Sense")


if __name__=='__main__':
	try:
		timeTest()
	except rospy.ROSInterruptException:
		pass # "pass" makes sure the node exits without issues

