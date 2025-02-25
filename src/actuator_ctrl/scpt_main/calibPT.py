#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
#
# This node reads the VNH5019 CS pin and logs it into a CSV file
#

import rospy
import time
import board
import busio
import csv
import RPi.GPIO as GPIO
from std_msgs.msg import Float32
import adafruit_ads1x15.ads1115 as ADS
from adafruit_pca9685 import PCA9685
from adafruit_ads1x15.analog_in import AnalogIn

# Time loop variables
intv = 2
intv2 = 1
tLast = 0
tLast2 = 0

###################################
### Initalize I2C Communication ###
###################################
i2c = board.I2C()  # Create the I2C bus interface; uses board.SCL and board.SDA

#########################################
### Initalize the PCA9685 and VNH5019 ###
#########################################
INA = 26
INB = 20
GPIO.setmode(GPIO.BCM)
GPIO.setup(INA, GPIO.OUT)
GPIO.setup(INB, GPIO.OUT)
pca = PCA9685(i2c, address=0x40)
pca.frequency = 100 # Hz
dcycle0 = float(1) # Delcaring the duty cycle values
pwm0 = int(dcycle0 * 0xFFFF)

##########################################
### Initalize the ADS1115 and csRead() ###
###########################################
ads = ADS.ADS1115(i2c, address=0x48)
PT1 = AnalogIn(ads, ADS.P0)	### CHANGE DEPENDING ON PINOUT TO ADS1115 ###
rospy.init_node('calibPT', anonymous=True)
print("{:>5}\t{:>5}".format("raw", "v"))
dataMat = [] #Initalize data matrix


def newCSV():
	rows = len(dataMat)
	with open('calibCS_data.csv', 'w', newline='') as csvfile:
		csvwriter = csv.writer(csvfile)
		
		# Write file headers
		csvwriter.writerow(["CS Value","CS Voltage","Time (ms)"])
		
		#Traverse along rows and add dataMat values to csv file
		for i in range(rows):
			csvwriter.writerow(dataMat[i])

#	f = open("calibCS_data.csv","r")
#	if f.mode == 'r':
#		f1 = f.readlines()
#		for x in f1:
#			print (x)

#	with open(calibCS_data, 'w', newline='') as csvfile:
#		csvwriter = csv.writer(csvfile)
#		csvwriter.writerows(dataMat)
#	rospy.loginfo(f'Sensor data saved to calibCS_data')


# Time loop variables
intvLA = 2000
intvDC = 15000
intv2 = 100
tLast = 0
tLast2 = 0

def ptRead(runTime):
	global dataMat
#	global rows
	print("{:>5}\t{:>5.3f}".format(PT1.value, PT1.voltage)) # This prints the value and voltage
	PTval = float(PT1.value)
	PTvol = float(PT1.voltage)
	#data = [CSval,CSvol]
	dataMat.append([PTval,PTvol,runTime])

def linAct():
	direc = int(1)
	global tLast
	global tLast2
	while not rospy.is_shutdown():
		tNow = int(time.time()*1000)
		tNow2 = int(time.time()*1000)
		if (tNow - tLast) > intvLA:
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
			csRead() 

if __name__ == '__main__':
	try:
		print("INA ON\tINB OFF")
		dcMot()
#		linAct()
	except rospy.ROSInterruptException:
		pass
	finally:
		GPIO.output(INA, GPIO.LOW)
		GPIO.output(INB, GPIO.LOW)
		newCSV()
	
