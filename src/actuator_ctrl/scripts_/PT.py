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
# Identifies the ADS1115 breakouts associated with the PT pins and reads the PT pins
# for all actuators to determine their lengths in real time.
#

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
PT1 = AnalogIn(ads1, ADS.P0)	
PT2 = AnalogIn(ads1, ADS.P1)	
PT3 = AnalogIn(ads2, ADS.P0)	
PT4 = AnalogIn(ads2, ADS.P1)	
PT5 = AnalogIn(ads3, ADS.P0)	
PT6 = AnalogIn(ads3, ADS.P1)	

def talker():
	pub = rospy.Publisher('potent', Float32MultiArray, queue_size=10)
	rospy.init_node('PT_node', anonymous=True)
	rate = rospy.Rate(10) # 10hz
#	print("{:>5}\t{:>5}".format("raw", "v"))
	while not rospy.is_shutdown():
		V1 = float(PT1.voltage)
		V2 = float(PT2.voltage)
		V3 = float(PT3.voltage)
		V4 = float(PT4.voltage)
		V5 = float(PT5.voltage)
		V6 = float(PT6.voltage)
		print("{:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f}".format(V1,V2,V3,V4,V5,V6)) # prints analog voltage for troubleshooting
		ptVol = Float32MultiArray()
		ptVol.data = [V1,V2,V3,V4,V5,V6]
		pub.publish(ptVol)
		rate.sleep()
    

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
