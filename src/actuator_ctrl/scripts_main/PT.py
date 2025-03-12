#!/usr/bin/env python3
#
# Identifies the ADS1115 breakouts associated with the PT pins and reads the PT pins
# for all actuators to determine their lengths in real time.
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
ads1 = ADS.ADS1115(i2c, address=0x48)
ads2 = ADS.ADS1115(i2c, address=0x49)

# Create single-ended input on channel 0
PT1 = AnalogIn(ads1, ADS.P0)	### CHANGE DEPENDING ON PINOUT TO ADS1115 ###
PT2 = AnalogIn(ads1, ADS.P1)	### CHANGE DEPENDING ON PINOUT TO ADS1115 ###
PT3 = AnalogIn(ads1, ADS.P2)	### CHANGE DEPENDING ON PINOUT TO ADS1115 ###
PT4 = AnalogIn(ads1, ADS.P3)	### CHANGE DEPENDING ON PINOUT TO ADS1115 ###
PT5 = AnalogIn(ads2, ADS.P0)	### CHANGE DEPENDING ON PINOUT TO ADS1115 ###
PT6 = AnalogIn(ads2, ADS.P1)	### CHANGE DEPENDING ON PINOUT TO ADS1115 ###

# Create differential input between channel 0 and 1
# chan = AnalogIn(ads, ADS.P0, ADS.P1)

#print("{:>5}\t{:>5}".format("raw", "v"))

def PTread(chan, chanNum):
	print("PT{} = {:>5}\t{:>5.3f}" .format(chanNum, chan.value, chan.voltage)) 
	# This prints the value and voltage
	PTval = float(chan.value)
	PTvol = float(chan.voltage)
	return [PTval, PTvol]

def init():
#	pub = rospy.Publisher('PT', Float32, queue_size=10)
	rospy.init_node('PT', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	print("{:>5}\t{:>5}".format("raw", "v"))

def talker():
	while not rospy.is_shutdown():
		print("{:>5}\t{:>5.3f}".format(chan.value, chan.voltage)) # This prints the value and voltage
		PTval = float(chan.value)
		PTvol = float(chan.voltage)
	#	rospy.loginfo(CSval) # This prints the loginfo
	#	pub.publish(CSval)
	#	rate.sleep()
    

if __name__ == '__main__':
	try:
		init()
		while not rospy.is_shutdown():
			[PTval1, PTvol1] = PTread(PT1, 1)
			[PTval2, PTvol2] = PTread(PT2, 2)
			time.sleep(0.5)
	except rospy.ROSInterruptException:
		pass
