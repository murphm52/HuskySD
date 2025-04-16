#!/usr/bin/env python3

## COPYRIGHT STATEMENT ##
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
# Outputs a 50% duty cycle PWM single on the 0th channel.
# Connect an LED and resistor in series to the pin
# to visualize duty cycle changes and its impact on brightness.

## NODE DESCRIPTION ##
# Outputs a PWM signals to six separate motor drivers based on the data published by ctrl_path over topic "path."

import rospy
import board
import busio
import RPi.GPIO as GPIO
from std_msgs.msg import Float32 
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from adafruit_pca9685 import PCA9685
import numpy as np

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
def initGPIO(INA, INB):
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(INA, GPIO.OUT)
	GPIO.setup(INB, GPIO.OUT)
	
def initI2C():
	global pca
	# Create the I2C bus interface.
	i2c = board.I2C()  # uses board.SCL and board.SDA
	pca = PCA9685(i2c, address=0x40)
	# Set the PWM frequency to a value in hz.
	pca.frequency = 100

def run(INA,INB,dcyc,pcaChan): # In this function, var dcyc is an individual float

	if dcyc > 0:
#		print("+Motor set to duty cycle {:>5.3f}." .format(dcyc))
		pwm = int(abs(dcyc) * 0xFFFF)
		pca.channels[pcaChan].duty_cycle = pwm
		GPIO.output(INA, GPIO.HIGH)
		GPIO.output(INB, GPIO.LOW)
	elif dcyc < 0:
#		print("-Motor set to duty cycle {:>5.3f}." .format(dcyc))
		pwm = int(abs(dcyc) * 0xFFFF)
		pca.channels[pcaChan].duty_cycle = pwm
		GPIO.output(INA, GPIO.LOW)
		GPIO.output(INB, GPIO.HIGH)
	elif dcyc == 0:
#		print("Motor stop (duty cycle = {:>5.3f})." .format(dcyc))
		GPIO.output(INA, GPIO.LOW)
		GPIO.output(INB, GPIO.LOW)

def return2zero(INA, INB, dcyc):			
	print("All motors stop")
	run(INA[0],INB[0],-1,0)
	run(INA[1],INB[1],-1,1)
	run(INA[2],INB[2],-1,2)
	run(INA[3],INB[3],-1,3)
	run(INA[4],INB[4],-1,4)
	run(INA[5],INB[5],-1,5)
	time.sleep(6)			# Temporary
	pca.channels[0].duty_cycle = 0
	pca.channels[1].duty_cycle = 0
	pca.channels[2].duty_cycle = 0
	pca.channels[3].duty_cycle = 0
	pca.channels[4].duty_cycle = 0
	pca.channels[5].duty_cycle = 0
	# Replace time.sleep with a while loop that detects when all actuators stop moving 
	# based on their potentiometer feedbacks.

def init():
	initGPIO(INA1, INB1)
	initGPIO(INA2, INB2)
	initGPIO(INA3, INB3)
	initGPIO(INA4, INB4)
	initGPIO(INA5, INB5)
	initGPIO(INA6, INB6)
	initI2C()
	rospy.init_node('exec_node', anonymous=True)

def callbackKill(data):
	rospy.loginfo(data.data)

def callbackPath(data):
	global INA
	global INB
	vec = np.array(data.data)
	dis = vec[0:6]
	dcyc = vec[6:12]
	print("Motor 1 set to duty cycle {:>5.3f}." .format(dcyc[0]))
	print("Motor 2 set to duty cycle {:>5.3f}." .format(dcyc[1]))
	print("Motor 3 set to duty cycle {:>5.3f}." .format(dcyc[2]))
	print("Motor 4 set to duty cycle {:>5.3f}." .format(dcyc[3]))
	print("Motor 5 set to duty cycle {:>5.3f}." .format(dcyc[4]))
	print("Motor 6 set to duty cycle {:>5.3f}." .format(dcyc[5]))

	run(INA[0],INB[0],dcyc[0],0)
	run(INA[1],INB[1],dcyc[1],1)
	run(INA[2],INB[2],dcyc[2],2)
	run(INA[3],INB[3],dcyc[3],3)
	run(INA[4],INB[4],dcyc[4],4)
	run(INA[5],INB[5],dcyc[5],5)
#	print(format(dcyc)) # prints analog voltage for troubleshooting
#	rospy.loginfo(data.data)

def main():
	# Initalize subscriber topics
	rospy.Subscriber('path', Float32MultiArray, callbackPath)
#	rospy.Subscriber('kill', Bool, callbackPath)
	rospy.spin()

if __name__=='__main__':
	try:
		init()
		main() # Go into the function itself to enable/disable specific actuators.
	except rospy.ROSInterruptException:
		pass # "pass" makes sure the node exits without issues
	finally:
		return2zero(INA, INB, dcyc)
		GPIO.cleanup()

