#!/usr/bin/env python3

## COPYRIGHT STATEMENT ##
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$
## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic


## NODE DESCRIPTION ##
# Identifies and reads the ADS1115 breakouts associated with the CS pins
# for all motor drivers to determine individual actuator current draw in real time.


import rospy
import time
import board
import busio
from std_msgs.msg import Float32MultiArray
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create the ADC object using the I2C bus
# ads = ADS.ADS1115(i2c)
# specify the I2C addresses for the ADS1115 boards
ads1 = ADS.ADS1115(i2c, address=0x48)
ads2 = ADS.ADS1115(i2c, address=0x49)
ads3 = ADS.ADS1115(i2c, address=0x4a)
CS1 = AnalogIn(ads1, ADS.P2)
CS2 = AnalogIn(ads1, ADS.P3)
CS3 = AnalogIn(ads2, ADS.P2)
CS4 = AnalogIn(ads2, ADS.P3)
CS5 = AnalogIn(ads3, ADS.P2)
CS6 = AnalogIn(ads3, ADS.P3)


def talker():
	pub = rospy.Publisher('current', Float32MultiArray, queue_size=10)
	rospy.init_node('CS_node', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	#print("{:>5}\t{:>5}".format("raw", "v"))
	while not rospy.is_shutdown():
		V1 = float(CS1.voltage)
		V2 = float(CS2.voltage)
		V3 = float(CS3.voltage)
		V4 = float(CS4.voltage)
		V5 = float(CS5.voltage)
		V6 = float(CS6.voltage)
		C1 = float(abs(V1/0.140)) #divide by 0.140 V/Amp
		C2 = float(abs(V2/0.140))
		C3 = float(abs(V3/0.140))
		C4 = float(abs(V4/0.140))
		C5 = float(abs(V5/0.140))
		C6 = float(abs(V6/0.140))
		print("{:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f}".format(C1,C2,C3,C4,C5,C6)) # prints analog voltage for troubleshooting
		csVol = Float32MultiArray()
		csVol.data = [V1,V2,V3,V4,V5,V6]
		pub.publish(csVol)
		rate.sleep()
    

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
