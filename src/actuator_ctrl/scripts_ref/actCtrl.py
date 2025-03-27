#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

## NODE DESCRIPTION ##
## OUTDATED ##
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

# INA and INB pinouts
INA = 26
INB = 20


GPIO.setmode(GPIO.BCM)
GPIO.setup(INA, GPIO.OUT)
GPIO.setup(INB, GPIO.OUT)

# Create the I2C bus interface.
i2c = board.I2C()  # uses board.SCL and board.SDA

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c, address=0x40)

# Set the PWM frequency to a value in hz.
pca.frequency = 490

# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.
dcycle0 = float(1) # Delcaring the duty cycle values

pwm0 = int(dcycle0 * 0xFFFF)
# The PWM cycle values must be converted to a value between 0 and 65535.
# The 0xFFFF is the maximum value that can be represented by a 16-bit
# unsigned integer. Multiplying a value between 0 and 1 with 0xFFFF
# creates a duty cycle value that can be used by the system.

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' I heard %5.3f', data.data)
    
def listen():
	rospy.Subscriber('length', Float32, callback)
	rospy.init_node('actCtrl', anonymous=True)
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

# Time loop variables
intv = 2000
intv2 = 1000
tLast = 0
tLast2 = 0
def sendPWM():
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
				pca.channels[0].duty_cycle = pwm0
				GPIO.output(INA, GPIO.HIGH)
				GPIO.output(INB, GPIO.LOW)
				print("INA ON\tINB OFF")
			else:
				direc = -direc
				pca.channels[0].duty_cycle = pwm0
				GPIO.output(INA, GPIO.LOW)
				GPIO.output(INB, GPIO.HIGH)
				print("INA OFF\tINB ON")
		
		if (tNow2 - tLast2) > intv2:
			tLast2 = tNow2
			# If the publisher is not active, the node will stay on this command until the
			# subscriber hears something, causing the sendPWM command to freeze
			#listen() 
			

if __name__=='__main__':
	try:
		sendPWM()
	except rospy.ROSInterruptException:
		pass # "pass" makes sure the node exits without issues
	finally:	# What the program does right before the node quits
		GPIO.output(INA, GPIO.LOW)
		GPIO.output(INB, GPIO.HIGH)
