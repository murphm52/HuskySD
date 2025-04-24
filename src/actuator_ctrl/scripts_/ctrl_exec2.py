#!/usr/bin/env python3

## COPYRIGHT STATEMENT ##
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
# Outputs a 50% duty cycle PWM single on the 0th channel.
# Connect an LED and resistor in series to the pin
# to visualize duty cycle changes and its impact on brightness.

## NODE DESCRIPTION ##
# Outputs PWM signals to six motor drivers based on adjusted duty cycles published by act_control.py.

import rospy
import board
import busio
import RPi.GPIO as GPIO
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from adafruit_pca9685 import PCA9685
import numpy as np

# INA and INB pinouts
INA1 = 21
INB1 = 26

INA2 = 20
INB2 = 19

INA3 = 16
INB3 = 13

INA4 = 25
INB4 = 22

INA5 = 24
INB5 = 27

INA6 = 23
INB6 = 17

INA = [INA1, INA2, INA3, INA4, INA5, INA6]
INB = [INB1, INB2, INB3, INB4, INB5, INB6]

# Store adjusted duty cycles
adjusted_duty_cycles = [0] * 6

def initGPIO(INA, INB):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)

def initI2C():
    global pca
    # Create the I2C bus interface.
    i2c = board.I2C()  # uses board.SCL and board.SDA
    pca = PCA9685(i2c, address=0x40)
    # Set the PWM frequency to a value in hz.
    pca.frequency = 100

def run(INA, INB, dcyc, pcaChan):  # In this function, var dcyc is an individual float
    if dcyc > 0:
        pwm = int(abs(dcyc) * 0xFFFF)
        pca.channels[pcaChan].duty_cycle = pwm
        GPIO.output(INA, GPIO.HIGH)
        GPIO.output(INB, GPIO.LOW)
    elif dcyc < 0:
        pwm = int(abs(dcyc) * 0xFFFF)
        pca.channels[pcaChan].duty_cycle = pwm
        GPIO.output(INA, GPIO.LOW)
        GPIO.output(INB, GPIO.HIGH)
    elif dcyc == 0:
        GPIO.output(INA, GPIO.LOW)
        GPIO.output(INB, GPIO.LOW)

def callbackDutyCycle(data):
    global adjusted_duty_cycles
    # Update the adjusted duty cycles list with the incoming data
    adjusted_duty_cycles = data.data
    rospy.loginfo(f"Updated adjusted duty cycles: {adjusted_duty_cycles}")

def callbackPath(data):
    global INA
    global INB
    global adjusted_duty_cycles
    # Use adjusted duty cycles
    dcyc = 1
    print("Motor 1 set to duty cycle {:>5.3f}." .format(adjusted_duty_cycles[0]))
    print("Motor 2 set to duty cycle {:>5.3f}." .format(adjusted_duty_cycles[1]))
    print("Motor 3 set to duty cycle {:>5.3f}." .format(adjusted_duty_cycles[2]))
    print("Motor 4 set to duty cycle {:>5.3f}." .format(adjusted_duty_cycles[3]))
    print("Motor 5 set to duty cycle {:>5.3f}." .format(adjusted_duty_cycles[4]))
    print("Motor 6 set to duty cycle {:>5.3f}." .format(adjusted_duty_cycles[5]))

    run(INA[0],INB[0],adjusted_duty_cycles[0],0)
    run(INA[1],INB[1],adjusted_duty_cycles[1],1)
    run(INA[2],INB[2],adjusted_duty_cycles[2],2)
    run(INA[3],INB[3],adjusted_duty_cycles[3],3)
    run(INA[4],INB[4],adjusted_duty_cycles[4],4)
    run(INA[5],INB[5],adjusted_duty_cycles[5],5)

def emergency_stop():
    """Stops all actuators by setting their GPIO pins to LOW."""
    rospy.logwarn("Emergency stop triggered. Stopping all actuators.")
    for i in range(6):
        GPIO.output(INA[i], GPIO.LOW)
        GPIO.output(INB[i], GPIO.LOW)
        pca.channels[i].duty_cycle = 0  # Set PWM duty cycle to 0

def monitor_ctrl_path2():
    """Monitors the /ctrl_path2_active parameter and triggers emergency stop if it is False."""
    rate = rospy.Rate(1)  # Check the parameter at 1 Hz
    while not rospy.is_shutdown():
        ctrl_path2_active = rospy.get_param("/ctrl_path2_active", True)
        if not ctrl_path2_active:
            emergency_stop()
            break
        rate.sleep()

def init():
    # Initialize GPIO pins and I2C
    initGPIO(INA1, INB1)
    initGPIO(INA2, INB2)
    initGPIO(INA3, INB3)
    initGPIO(INA4, INB4)
    initGPIO(INA5, INB5)
    initGPIO(INA6, INB6)
    initI2C()
    rospy.init_node('exec_node', anonymous=True)
    

def main():
    global adjusted_duty_cycles
    # Subscribe to the /adjusted_duty_cycle topic
    rospy.Subscriber("/adjusted_duty_cycle", Float32MultiArray, callbackDutyCycle)

    # Subscribe to path topic (if additional data like desired lengths is needed)
    rospy.Subscriber('path', Float32MultiArray, callbackPath)

    # Start monitoring the /ctrl_path2_active parameter in a separate thread
    import threading
    monitor_thread = threading.Thread(target=monitor_ctrl_path2)
    monitor_thread.daemon = True
    monitor_thread.start()

    rospy.spin()

if __name__ == '__main__':
    try:
        init()
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
