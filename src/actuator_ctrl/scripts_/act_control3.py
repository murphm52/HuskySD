#!/usr/bin/env python3

import rospy
import csv
import os
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32MultiArray  # Assuming potentiometer feedback is published as Float32MultiArray
from scipy.interpolate import interp1d

def load_csv_data(file_path):
    #"""Load CSV file data into a dictionary."""
    csv_data = {}
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            duty_cycle = float(row[0])
            lengths = [float(value) for value in row[1:]]
            csv_data[duty_cycle] = lengths
    return csv_data

def interpolate_duty_cycle(length, csv_lengths, csv_duty_cycles):
    #"""Interpolate the duty cycle based on the current length."""
    interpolator = interp1d(csv_lengths, csv_duty_cycles, bounds_error=False, fill_value=(0.1, 1))
    return interpolator(length)

def calculate_adjusted_duty_cycles(current_lengths, desired_lengths, desired_duty_cycles, csv_data, publisher):
    adjusted_duty_cycles = []
    tolerance = 0.5  # Increased tolerance
    smoothing_factor = 0.1  # Smoothing factor for duty cycle adjustments
    emergency_threshold = 75  # Threshold for triggering emergency stop

    # Initialize a list to store previous errors for each actuator
    previous_errors = [0] * len(current_lengths)

    for i, current_length in enumerate(current_lengths):
        desired_length = desired_lengths[i]
        csv_lengths = [row[i] for row in csv_data.values()]
        csv_duty_cycles = list(csv_data.keys())
        max_length = max(csv_lengths)

        # Calculate error
        error = desired_length - current_length
        print(f"Actuator {i+1}: Current Length = {current_length}, Desired Length = {desired_length}, Error = {error}")

        # Interpolate the duty cycle
        if abs(current_length - desired_length) > max_length + tolerance:
            adjusted_duty_cycle = 1.0 if error > 0 else -1.0
        else:
            adjusted_duty_cycle = interpolate_duty_cycle(abs(error), csv_lengths, csv_duty_cycles)
            if error < 0:
                adjusted_duty_cycle = -adjusted_duty_cycle

        # Smooth the duty cycle
        if len(adjusted_duty_cycles) > i:
            adjusted_duty_cycle = adjusted_duty_cycles[i] + smoothing_factor * (adjusted_duty_cycle - adjusted_duty_cycles[i])

        # Round the duty cycle to the hundredths place
        adjusted_duty_cycle = round(float(adjusted_duty_cycle), 2)

        # Emergency stop condition: Check if error is increasing and duty cycle is in the opposite direction
        if abs(error) >= emergency_threshold:
            #if (error - previous_errors[i] > 0 and adjusted_duty_cycle <= 0) or (error - previous_errors[i] < 0 and adjusted_duty_cycle >= 0):
            print("-------------------")
            print(f"Emergency stop triggered for Actuator {i}! Error = {error}, Previous Error = {previous_errors[i]}, Adjusted Duty Cycle = {adjusted_duty_cycle}")
            print("-------------------")
            rospy.logwarn("Emergency stop triggered! All duty cycles set to 0.")
            
            # Set all duty cycles to 0
            adjusted_duty_cycles = [0] * len(current_lengths)
            
            # Infinite loop to hold the node in emergency stop state
            emergency_stop_triggered = True
            while emergency_stop_triggered:
                publish_duty_cycles(adjusted_duty_cycles, publisher)
                rospy.sleep(1)  # Sleep to prevent excessive CPU usage
                return adjusted_duty_cycles

        # Update the previous error for the next iteration
        previous_errors[i] = error

        # Ignore small errors
        if abs(error) < tolerance:
            adjusted_duty_cycles.append(0.0)
            continue

        adjusted_duty_cycles.append(adjusted_duty_cycle)

    return adjusted_duty_cycles

def process_actuators(msg, csv_data, publisher):
    # """Callback function to process potentiometer feedback."""
    current_lengths = msg.data  # Assuming `data` field holds potentiometer feedback

    # Get parameters and check if they are valid
    desired_lengths = rospy.get_param(f"/desired_length", None)
    desired_duty_cycles = rospy.get_param(f"/desired_duty_cycle", None)

    # Initialize a history of desired lengths
    if not hasattr(process_actuators, "desired_lengths_history"):
        process_actuators.desired_lengths_history = [None, None]  # Store the last two sets of desired lengths

    if desired_lengths is None or desired_duty_cycles is None:
        rospy.logwarn("Missing or invalid parameters: desired_length or desired_duty_cycle. Triggering emergency stop.")
        # Trigger emergency stop by setting all duty cycles to 0
        adjusted_duty_cycles = [0] * len(current_lengths)
        publish_duty_cycles(adjusted_duty_cycles, publisher)
        # Infinite loop to hold the node in emergency stop state
        emergency_stop_triggered = True
        while emergency_stop_triggered:
            publish_duty_cycles(adjusted_duty_cycles, publisher)
            rospy.sleep(1)  # Sleep to prevent excessive CPU usage
        return

    if len(desired_lengths) != len(current_lengths) or len(desired_duty_cycles) != len(current_lengths):
        rospy.logwarn("Parameter lengths do not match current lengths. Triggering emergency stop.")
        # Trigger emergency stop by setting all duty cycles to 0
        adjusted_duty_cycles = [0] * len(current_lengths)
        publish_duty_cycles(adjusted_duty_cycles, publisher)
        # Infinite loop to hold the node in emergency stop state
        emergency_stop_triggered = True
        while emergency_stop_triggered:
            publish_duty_cycles(adjusted_duty_cycles, publisher)
            rospy.sleep(1)  # Sleep to prevent excessive CPU usage
        return

    ctrl_path2_active = rospy.get_param("/ctrl_path2_active", False)
    if not ctrl_path2_active:
        rospy.logwarn("ctrl_path2_active parameter is not set. Triggering emergency stop.")
        # Trigger emergency stop by setting all duty cycles to 0
        adjusted_duty_cycles = [0] * len(current_lengths)
        publish_duty_cycles(adjusted_duty_cycles, publisher)
        # Infinite loop to hold the node in emergency stop state
        emergency_stop_triggered = True
        while emergency_stop_triggered:
            publish_duty_cycles(adjusted_duty_cycles, publisher)
            print("here")
            rospy.sleep(1)
        return adjusted_duty_cycles


    # Update the history of desired lengths
    process_actuators.desired_lengths_history[0] = process_actuators.desired_lengths_history[1]
    process_actuators.desired_lengths_history[1] = desired_lengths

    # Calculate adjusted duty cycles
    adjusted_duty_cycles = calculate_adjusted_duty_cycles(current_lengths, desired_lengths, desired_duty_cycles, csv_data, publisher)
    print(f"Desired duty cycles: {desired_duty_cycles}, Adjusted duty cycles: {adjusted_duty_cycles}")  # Added print statement
    publish_duty_cycles(adjusted_duty_cycles, publisher)

def publish_duty_cycles(duty_cycles, publisher):
    #"""Publish the adjusted duty cycles."""
    duty_cycle_msg = Float32MultiArray(data=duty_cycles)
    publisher.publish(duty_cycle_msg)

def main():
    #print("Starting actuator error checker node...")
    # Initialize ROS node
    rospy.init_node("actuator_error_checker", anonymous=True)

    # Load CSV file data
    csv_path_param = rospy.get_param("~csv_path", "~/husky_ws/src/actuator_ctrl/scripts_/achievable_lengths.csv")
    csv_path = os.path.expanduser(csv_path_param)  # Expand `~` to the full home directory path
    if not os.path.isfile(csv_path):
        rospy.logerr(f"CSV file not found at {csv_path}")
        return

    csv_data = load_csv_data(csv_path)

    # Initialize ROS publisher
    publisher = rospy.Publisher("/adjusted_duty_cycle", Float32MultiArray, queue_size=10)

    # Subscribe to potentiometer feedback topic
    rospy.Subscriber("/potent", Float32MultiArray, lambda msg: process_actuators(msg, csv_data, publisher))

    rospy.spin()

if __name__ == "__main__":
    main()
