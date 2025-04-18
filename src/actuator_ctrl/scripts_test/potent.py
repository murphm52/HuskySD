#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

#
# This node reads potentiometer feedback through the ADS1115 ADC module and sends it to the actCtrl.py node
#

import rospy
import time
import board
import busio
from std_msgs.msg import Float32
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create the ADC object using the I2C bus
# ads = ADS.ADS1115(i2c)
# you can specify an I2C adress instead of the default 0x48
ads = ADS.ADS1115(i2c, address=0x48)

# Create single-ended input on channel 0
chan = AnalogIn(ads, ADS.P0)	### CHANGE DEPENDING ON PINOUT TO ADS1115 ###

# Create differential input between channel 0 and 1
# chan = AnalogIn(ads, ADS.P0, ADS.P1)

#print("{:>5}\t{:>5}".format("raw", "v"))

def talker():
	pub = rospy.Publisher('length', Float32, queue_size=10)
	rospy.init_node('potent', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	print("{:>5}\t{:>5}".format("raw", "v"))
	while not rospy.is_shutdown():
		print("{:>5}\t{:>5.3f}".format(chan.value, chan.voltage)) # This prints the value and voltage
		PTval = float(chan.value)
		PTvol = float(chan.voltage)
		# rospy.loginfo(CSval) # This prints the loginfo
		pub.publish(PTvol)
		rate.sleep()
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
