#!/usr/bin/env python3

import rospy
from std_msgs.msg import float32
from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA

# Initiate I2C bus and PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50 # This is our PWM signal in Hz (50Hz in this case)

def set_pwm_channel(channel, duty_cycle):
	duty_cycle = max(0, min(1, duty_cycle)) # Ensure duty cycle is b/w 0 and 1
	pwm_value = int(duty_cycle*0xFFFF)
	pca.channels[channel].duty_cycle = pwm_value
	
def pwm_callback(msg):
	channel = int(msg.data) # Assuming message data contains the channel number
	duty_cycle = msg.data - channel # Assuming the duty cycle us the fractional part
	set_pwm_channel(channel, duty_cycle)
	
def sendPWM():
	rospy.init_node('sendPWM', anonymous = True)
	rospy.Subscribe('pwm_control', Float32, pwm_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		sendPWM()
	except rospy.ROSInterruptExcepion:
		pass
