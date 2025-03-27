#!/usr/bin/env python3

## COPYRIGHT STATEMENT ##
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
# Outputs a 50% duty cycle PWM single on the 0th channel.
# Connect an LED and resistor in series to the pin
# to visualize duty cycle changes and its impact on brightness.


## NODE DESCRIPTION ##
# Outputs a PWM signal with a constant duty cycle to a VNH5019 motor driver carrer.

import rospy
import time
import board
import busio
import RPi.GPIO as GPIO
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

# Set the PWM frequency to 60hz.
pca.frequency = 100

# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.
dcycle0 = float(1) # Delcaring the duty cycle values

pwm0 = int(dcycle0 * 0xFFFF)
# The PWM cycle values must be converted to a value between 0 and 65535.
# The 0xFFFF is the maximum value that can be represented by a 16-bit
# unsigned integer. Multiplying a value between 0 and 1 with 0xFFFF
# creates a duty cycle value that can be used by the system.


def sendPWM():
	while True:
		pca.channels[0].duty_cycle = pwm0
		GPIO.output(INA, GPIO.HIGH)
		GPIO.output(INB, GPIO.LOW)
		print("INA ON\tINB OFF")
		time.sleep(2)
		pca.channels[0].duty_cycle = pwm0
		GPIO.output(INA, GPIO.LOW)
		GPIO.output(INB, GPIO.HIGH)
		print("INA OFF\tINA ON")
		time.sleep(2)

if __name__=='__main__':
	try:
		sendPWM()
	except rospy.ROSInterruptException:
		pass # "pass" makes sure the node exits without issues
	finally:	# What the program does right before the node quits
		GPIO.output(INA, GPIO.LOW)
		GPIO.output(INB, GPIO.HIGH)



