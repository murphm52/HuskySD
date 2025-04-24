#!/usr/bin/env python3

## NODE DESCRIPTION ##
# Reads the CSV path row by row and publishes the data to ctrl_exec.

import rospy
import csv
import os
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

def csv_reader():
    rospy.init_node('path_node', anonymous=True)
    rospy.set_param("/ctrl_path2_active", True)
    pubPath = rospy.Publisher('path', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 1 Hz (Adjust as needed)
    csv_file_path = os.path.expanduser('~/husky_ws/src/actuator_ctrl/scripts_/paths/xRot.csv')

    try:
        # Count the number of rows in the CSV file
        with open(csv_file_path, 'r') as csvfile:
            total_rows = sum(1 for _ in csvfile)
        rospy.set_param("/csv_length", total_rows)  # Set the total number of rows as a parameter
        rospy.loginfo(f"CSV file length (number of rows): {total_rows}")

        # Read and process the CSV file
        with open(csv_file_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            row_index = 1  # Start at row 1
            for row in reader:
                if rospy.is_shutdown():
                    break
                
                # Separate actuator leg lengths and duty cycles
                leg_lengths = list(map(float, row[:6]))  # First 6 columns
                duty_cycles = list(map(float, row[6:12]))  # Next 6 columns

                # Publish combined data
                combined_row_data = leg_lengths + duty_cycles
                msg = Float32MultiArray()
                msg.data = combined_row_data
                pubPath.publish(msg)
                rospy.loginfo(f"Published vector: {combined_row_data}")

                # Set desired lengths and duty cycles as separate parameters
                rospy.set_param("/desired_length", leg_lengths)  # Set actuator lengths
                rospy.set_param("/desired_duty_cycle", duty_cycles)  # Set duty cycles

                # Increment row index for reference
                row_index += 1

                rate.sleep()
    except IOError:
        rospy.logerr("Failed to read the CSV file.")
    finally:
        rospy.set_param("/ctrl_path2_active", False)
        rospy.loginfo("CSV reading completed. Node deactivated.")
if __name__ == '__main__':
    try:
        csv_reader()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Node interrupted.")
