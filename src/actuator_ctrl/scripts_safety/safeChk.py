#!/usr/bin/env python3
#
# Identifies the ADS1115 breakouts associated with the CS pins and reads the CS pins
# to ensure the current does not reach abnormal levels.
#
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
ads2 = ADS.ADS1115(i2c, address=0x49)
ads3 = ADS.ADS1115(i2c, address=0x4a)

# Create single-ended input on channel 0
CS1 = AnalogIn(ads2, ADS.P2)	### CHANGE DEPENDING ON PINOUT TO ADS1115 ###
CS2 = AnalogIn(ads2, ADS.P3)	### CHANGE DEPENDING ON PINOUT TO ADS1115 ###
CS3 = AnalogIn(ads3, ADS.P0)	### CHANGE DEPENDING ON PINOUT TO ADS1115 ###
CS4 = AnalogIn(ads3, ADS.P1)	### CHANGE DEPENDING ON PINOUT TO ADS1115 ###
CS5 = AnalogIn(ads3, ADS.P2)	### CHANGE DEPENDING ON PINOUT TO ADS1115 ###
CS6 = AnalogIn(ads3, ADS.P3)	### CHANGE DEPENDING ON PINOUT TO ADS1115 ###

# Create differential input between channel 0 and 1
# chan = AnalogIn(ads, ADS.P0, ADS.P1)

#print("{:>5}\t{:>5}".format("raw", "v"))

def CSread(CSchan,chanNum):
	print("CS%i = {:>5}\t{:>5.3f}" % chanNum .format(CSchan.value, CSchan.voltage)) # This may result in error
	CSval = float(CSchan.value)
	CSvol = float(CSchan.voltage)

def talker():
	pub = rospy.Publisher('current', Float32, queue_size=10)
	rospy.init_node('CS', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	print("{:>5}\t{:>5}".format("raw", "v"))
	while not rospy.is_shutdown():
		print("CS1 = {:>5}\t{:>5.3f}".format(CS1.value, CS1.voltage)) # This prints the value and voltage
		CS1val = float(chan.value)
		CS1vol = float(chan.voltage)
		# rospy.loginfo(CSval) # This prints the loginfo
		pub.publish(CSval)
		rate.sleep()
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
