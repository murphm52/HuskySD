#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
#
# This node reads the linear actuator's built-in potentiometer and logs it into a CSV file
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
	with open('calibPT_data.csv', 'w', newline='') as csvfile:
		csvwriter = csv.writer(csvfile)
		
		# Write file headers
		csvwriter.writerow(["PT Value","PT Voltage","Time (ms)"])
		
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




def ptRead(runTime):
	global dataMat
	print("{:>5}\t{:>5.3f}".format(PT1.value, PT1.voltage)) # This prints the value and voltage
	PTval = float(PT1.value)
	PTvol = float(PT1.voltage)
	dataMat.append([PTval,PTvol,runTime])

def linAct1():
	intv1 = 100
	intvLA = 5000
	ptNow = float(30000)  #placeholder value
	ptLast = float(0)
	direc = int(1)
	tLast1 = 0
	tLast2 = 0
	tMasL = int(time.time()*1000)
	tLast = int(time.time()*1000)
	while not rospy.is_shutdown():
		while (ptNow - ptLast) > 0:
			tMas = int(time.time()*1000)
			tNow1 = int(time.time()*1000)
			if (tNow1 - tLast1) > intv1:
				ptNow = float(PT1.value)
				ptLast = ptNow
				pca.channels[0].duty_cycle = pwm0
				GPIO.output(INA, GPIO.HIGH)
				GPIO.output(INB, GPIO.LOW)
				tLast1 = tNow1
				runTime = int(tMas-tMasL)
				ptRead(runTime)
				ptNow = float(PT1.value)
#				print(float(ptNow-ptLast))

		GPIO.output(INA, GPIO.LOW)
		GPIO.output(INB, GPIO.LOW)
#		tNow = 0
		
#		while (tNow-tLast) < intvLA:
#			tMas = int(time.time()*1000)
#			tNow1 = int(time.time()*1000)
#			if (tNow1 - tLast1) > intv1:
#				ptNow = float(PT1.value)
#				ptLast = ptNow
#				pca.channels[0].duty_cycle = pwm0
#				GPIO.output(INA, GPIO.HIGH)
#				GPIO.output(INB, GPIO.LOW)
#				tLast1 = tNow1
#				runTime = int(tNow-tLast)
#				ptRead(runTime)
#				ptNow = float(PT1.value)

def linAct2():
	intv1 = 100
	intv2 = 5000
	ptNow = float(30000)  #placeholder value
	ptLast = float(0)
	direc = int(1)
	tLast1 = 0
	tLast2 = 0
	tNow = 0
	tLast = int(time.time()*1000)
	intvLA = 5000
	while not rospy.is_shutdown() and (tNow-tLast) < intvLA:
		tNow1 = int(time.time()*1000)
		tNow = int(time.time()*1000)
		if (tNow1 - tLast1) > intv1:
			ptNow = float(PT1.value)
			ptLast = ptNow
			pca.channels[0].duty_cycle = pwm0
			GPIO.output(INA, GPIO.HIGH)
			GPIO.output(INB, GPIO.LOW)
			tLast1 = tNow1
			runTime = int(tNow-tLast)
			ptRead(runTime)
			ptNow = float(PT1.value)
#			print(float(ptNow-ptLast))
#		GPIO.output(INA, GPIO.LOW)
#		GPIO.output(INB, GPIO.LOW)
#		phase = phase + 1
#		while not rospy.is_shutdown(): #tNow1 > intv1:
#			tNow2 = int(time.time()*1000)
#			#tNow1 = int(time.time()*1000)
#			if (tNow1 - tLast1) > intv1:
#				ptNow = float(PT1.value)
#				ptLast = ptNow
#				pca.channels[0].duty_cycle = pwm0
#				tLast1 = tNow1
#				runTime = tNow1
#				ptRead(runTime)
#				ptNow = float(PT1.value)
#				print(float(ptNow-ptLast))
#		phase = phase + 1

def backforth():
	# Time loop variables
	intvLA = 2000
	intv1 = 5000
	intv2 = 100
	tLastLA = 0
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
			ptRead() 




if __name__ == '__main__':
	try:
		print("INA ON\tINB OFF")
#		linAct1()
		linAct2()
	except rospy.ROSInterruptException:
		pass
	finally:
		GPIO.output(INA, GPIO.LOW)
		GPIO.output(INB, GPIO.HIGH)
		newCSV()
	
