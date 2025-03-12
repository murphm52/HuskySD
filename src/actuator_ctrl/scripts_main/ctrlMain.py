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
from std_msgs.msg import Int32
from adafruit_pca9685 import PCA9685

# INA and INB pinouts
INA1 = 21
INB1 = 26

INA2 = 20
INB2 = 19

INA3 = 16
INB3 = 13

INA4 = 25
INB4 = 22

INA5 = 24
INB5 = 27

INA6 = 23
INB6 = 17

INA = [INA1, INA2, INA3, INA4, INA5, INA6]
INB = [INB1, INB2, INB3, INB4, INB5, INB6]

# DO NOT USE THE INA AND INB MATRICIES FOR THIS FUNCTION
def setup(INA, INB):
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(INA, GPIO.OUT)
	GPIO.setup(INB, GPIO.OUT)



# Create the I2C bus interface.
i2c = board.I2C()  # uses board.SCL and board.SDA

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c, address=0x40)

# Set the PWM frequency to a value in hz.
pca.frequency = 100

# Set the PWM duty cycle for each channel. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.

# Delcaring the duty cycle values
dcyc0 = float(1) 
dcyc1 = float(1)
dcyc2 = float(1) 
dcyc3 = float(1) 
dcyc4 = float(0.2) 
dcyc5 = float(0.1) 

dcyc = [dcyc0, dcyc1, dcyc2, dcyc3, dcyc4, dcyc5]


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

def run(INA,INB,dcyc,pcaChan,direc):
	if direc > 0:
		pwm = int(dcyc * 0xFFFF)
		pca.channels[pcaChan].duty_cycle = pwm
		GPIO.output(INA, GPIO.HIGH)
		GPIO.output(INB, GPIO.LOW)
		print("Motor %i forward." % int(pcaChan+1))
	else:
		pwm = int(dcyc * 0xFFFF)
		pca.channels[pcaChan].duty_cycle = pwm
		GPIO.output(INA, GPIO.LOW)
		GPIO.output(INB, GPIO.HIGH)
		print("Motor %i backward." % int(pcaChan+1))

def sendPWM(INA, INB, dcyc):
		# The PWM cycle values must be converted to a value between 0 and 65535.
		# The 0xFFFF is the maximum value that can be represented by a 16-bit
		# unsigned integer. Multiplying a value between 0 and 1 with 0xFFFF
		# creates a duty cycle value that can be used by the system.
	direc = int(1)
	global tLast
	global tLast2
	while not rospy.is_shutdown():
		tNow = int(time.time()*1000)
		tNow2 = int(time.time()*1000)
		if (tNow - tLast) > intv:
			tLast = tNow
			run(INA[0],INB[0],dcyc[0],0,direc)
			run(INA[1],INB[1],dcyc[1],1,direc)
#			run(INA[2],INB[2],dcyc[2],2,direc)
#			run(INA[3],INB[3],dcyc[3],3,direc)
#			run(INA[4],INB[4],dcyc[4],4,direc)
#			run(INA[5],INB[5],dcyc[5],5,direc)
			direc = -direc
#		
#		if (tNow2 - tLast2) > intv2:
#			tLast2 = tNow2
#			If the publisher is not active, the node will stay on this command until the
#			# subscriber hears something, causing the sendPWM command to freeze
#			listen() 
def return2zero(INA, INB):
	run(INA[0],INB[0],1,0,-1)
	run(INA[1],INB[1],1,1,-1)
	time.sleep(6)			# Temporary
	# Replace time.sleep with a while loop that detects when all actuators stop moving 
	# based on their potentiometer feedbacks.

if __name__=='__main__':
	try:
		setup(INA1, INB1)
		setup(INA2, INB2)
#		setup(INA3, INB3)
#		setup(INA4, INB4)
#		setup(INA5, INB5)
#		setup(INA6, INB6)
		sendPWM(INA, INB, dcyc) # Go into the function itself to enable/disable specific actuators.
	except rospy.ROSInterruptException:
		pass # "pass" makes sure the node exits without issues
	finally: # All actuators fully retract at full duty cycle
		return2zero(INA, INB)
		GPIO.cleanup()

