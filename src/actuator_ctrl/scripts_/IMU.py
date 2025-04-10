#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
#
# Reads the IMU data and publishes it to the safeChk.py node.
#
#

import rospy
import time
import board
import busio
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import adafruit_bno055

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)


last_val = 0xFFFF

def talker():
	pub = rospy.Publisher('eulerAng', Float32, queue_size=10)
	rospy.init_node('BNO', anonymous=True)
	rate = rospy.Rate(2) # 10hz
	#print("{:>5}\t{:>5}".format("raw", "v"))
	while not rospy.is_shutdown():
		euler = sensor.euler
		accel = sensor.acceleration
#		print("Euler angle: {}".format(euler))
		print("Acceleration: {}".format(accel))
		pub.publish(euler)
#		pub.publish(quat)
		rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
