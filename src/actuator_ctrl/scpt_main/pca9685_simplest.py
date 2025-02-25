#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Outputs a 50% duty cycle PWM single on the 0th channel.
# Connect an LED and resistor in series to the pin
# to visualize duty cycle changes and its impact on brightness.

import rospy
import board
from adafruit_pca9685 import PCA9685

# Create the I2C bus interface.
i2c = board.I2C()  # uses board.SCL and board.SDA

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)

# Set the PWM frequency to 60hz.
pca.frequency = 60

# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.
dcycle0 = float(0.1) # Delcaring the duty cycle values
dcycle1 = float(0.9)

pwm0 = int(dcycle0 * 0xFFFF)
pwm1 = int(dcycle1 * 0xFFFF)
# The PWM cycle values must be converted to a value between 0 and 65535.
# The 0xFFFF is the maximum value that can be represented by a 16-bit
# unsigned integer. Multiplying a value between 0 and 1 with 0xFFFF
# creates a duty cycle value that can be used by the system.

pca.channels[0].duty_cycle = pwm0
pca.channels[1].duty_cycle = pwm1
