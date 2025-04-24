#!/usr/bin/env python3

import rospy
import csv
import os
import math
import pandas as pd
from std_msgs.msg import Float32MultiArray, Float32

# PID parameters
Kp, Ki, Kd = 1.5, 0.1, 0.05

# Initialize global variables for PID
prev_error = [0] * 6  # One for each actuator
integral_error = [0] * 6  # One for each actuator
current_lengths = [0] * 6  # Real-time lengths of actuators
adjusted_duty_cycles = [0] * 6  # Store adjusted duty cycles

# Read in csv data
def csv_reader():
    csv_FilePath = os.path.expanduser('~/husky_ws/src/actuator_ctrl/scripts_/achievable_lengths.csv')
    try:
        csvdata = pd.read_csv(csv_FilePath, header=None)  # Load entire file as a DataFrame
        return csvdata
    except IOError:
        rospy.logerr("Failed to read CSV file")
        return None

# Callback function for potent topic
def potent_callback(data):
    global current_lengths
    current_lengths = data.data  # Update actuator lengths in real time
#   rospy.loginfo(f"Updated actuator lengths: {current_lengths}")

# PID control callback
def callback(actuator_index):
    global prev_error, integral_error, current_lengths, adjusted_duty_cycles
    
    # Ensure current_lengths is properly updated
    current_length = current_lengths[actuator_index - 1]  # Get the length for the specific actuator

    # Retrieve target length from ROS parameters
    target_length = rospy.get_param(f"/desired_length_actuator_{actuator_index}_row_1")  # Adjust row index dynamically if needed

    # Compute error
    error = target_length - current_length
    
    
    # Find row in CSV
    csv_Data = csv_reader()
    if csv_Data is not None:
    	
        # Find the row where the column corresponding to the current actuator column matches the error
        match_Row = csv_Data.loc[abs(csv_Data[actuator_index + 1] - error) <= 0.03]
        

        if not match_Row.empty:
            closest_duty_cycle = csv_Data.iloc[match_Row, 0]  # Get the corresponding duty cycle (column 0)
        else:
            rospy.logwarn(f"No match found for rounded correction: {round_Correction} within tolerance of 0.03")
            closest_duty_cycle = rospy.get_param(f"/desired_duty_cycle_actuator_{actuator_index}_row_{row_index}")
            
            
    # Store the adjusted duty cycle
    #print(correction)
    adjusted_duty_cycles[actuator_index - 1] = closest_duty_cycle

    # Publish corrected duty cycle
    duty_cycle_pub[actuator_index - 1].publish(correction)
#   rospy.loginfo(f"Actuator {actuator_index} duty cycle adjusted: {correction}")

    # Update previous error for the next iteration
    prev_error[actuator_index - 1] = error

# Initialize ROS node
rospy.init_node("act_control")
#rospy.loginfo("Running act_control node")

# Create publishers for each actuator's adjusted duty cycle
duty_cycle_pub = [rospy.Publisher(f"/adj_duty_cycle_{i+1}", Float32, queue_size=10) for i in range(6)]

# Subscribe to the potent topic
rospy.Subscriber("potent", Float32MultiArray, potent_callback)

# Create individual PID callbacks for each actuator using a factory function
def callback_factory(actuator_index):
    def wrapped_callback(event):
        callback(actuator_index)
    return wrapped_callback

for i in range(1, 7):
    rospy.Timer(rospy.Duration(0.1), callback_factory(i))  # Use the factory function to bind `i`

# Print all adjusted duty cycles side by side
rospy.Timer(rospy.Duration(0.1), lambda event: print(f"Adjusted Duty Cycles: {adjusted_duty_cycles}"))  # Print every 0.1 seconds

rospy.spin()  # Keep the node running