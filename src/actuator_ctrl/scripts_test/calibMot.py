#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Test the linear actuator at different duty cycles and figure out what duty cycle values
# return what speeds
# 

import rospy
import board
import busio
import time
import csv
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



# Define the duty cycle value to test:
dcyc = 1

# Initalize data matrix
dataMat = [] #Initalize data matrix

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
		# The PWM cycle values must be converted to a value between 0 and 65535.
		# The 0xFFFF is the maximum value that can be represented by a 16-bit
		# unsigned integer. Multiplying a value between 0 and 1 with 0xFFFF
		# creates a duty cycle value that can be used by the system.
		pwm = int(abs(dcyc) * 0xFFFF)
		pca.channels[pcaChan].duty_cycle = pwm
		GPIO.output(INA, GPIO.HIGH)
		GPIO.output(INB, GPIO.LOW)
#		print("Motor %i forward." % int(pcaChan+1))
	else:
		pwm = int(abs(dcyc) * 0xFFFF)
		pca.channels[pcaChan].duty_cycle = pwm
		GPIO.output(INA, GPIO.LOW)
		GPIO.output(INB, GPIO.HIGH)
#		print("Motor %i backward." % int(pcaChan+1))

def setSpeed():
	print("All motors set to duty cycle {:>5.3f}." .format(dcyc))
	run(INA[0],INB[0],dcyc,0)
	run(INA[1],INB[1],dcyc,1)
	run(INA[2],INB[2],dcyc,2)
	run(INA[3],INB[3],dcyc,3)
	run(INA[4],INB[4],dcyc,4)
	run(INA[5],INB[5],dcyc,5)

def init():
	initGPIO(INA1, INB1)
	initGPIO(INA2, INB2)
	initGPIO(INA3, INB3)
	initGPIO(INA4, INB4)
	initGPIO(INA5, INB5)
	initGPIO(INA6, INB6)
	initI2C()
	rospy.init_node('exec_node', anonymous=True)
	setSpeed()

def callbackPT(data):
	global dataMat
	global PTpub
	PT = np.array(data.data)
	PT1 = PT[0]
	PT2 = PT[1]
	PT3 = PT[2]
	PT4 = PT[3]
	PT5 = PT[4]
	PT6 = PT[5]
	runTime = PT[6]
	print("{:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f}".format(PT1,PT2,PT3,PT4,PT5,PT6,runTime))
	dataMat.append([PT1,PT2,PT3,PT4,PT5,PT6,runTime])

def main():
	# Initalize subscriber topics
	rospy.Subscriber('potent', Float32MultiArray, callbackPT)
	rospy.spin()

def return2zero(INA, INB):			
	print("All motors go back")
	run(INA[0],INB[0],-1,0)
	run(INA[1],INB[1],-1,1)
	run(INA[2],INB[2],-1,2)
	run(INA[3],INB[3],-1,3)
	run(INA[4],INB[4],-1,4)
	run(INA[5],INB[5],-1,5)
	time.sleep(10)			# Temporary
	pca.channels[0].duty_cycle = 0
	pca.channels[1].duty_cycle = 0
	pca.channels[2].duty_cycle = 0
	pca.channels[3].duty_cycle = 0
	pca.channels[4].duty_cycle = 0
	pca.channels[5].duty_cycle = 0
	# Replace time.sleep with a while loop that detects when all actuators stop moving 
	# based on their potentiometer feedbacks.

def newCSV():
	rows = len(dataMat)
	with open('cmotData100.csv', 'w', newline='') as csvfile:
		csvwriter = csv.writer(csvfile)
		
		# Write file headers
		csvwriter.writerow(["PT1","PT2","PT3","PT4","PT5","PT6","Time (ms)"])
		
		#Traverse along rows and add dataMat values to csv file
		for i in range(rows):
			csvwriter.writerow(dataMat[i])

if __name__=='__main__':
	try:
		init()
		main() # Go into the function itself to enable/disable specific actuators.
	except rospy.ROSInterruptException:
		pass # "pass" makes sure the node exits without issues
	finally:
	#	newCSV()
		return2zero(INA, INB)
		GPIO.cleanup()

