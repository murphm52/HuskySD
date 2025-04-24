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
	pubPath = rospy.Publisher('path', Float32MultiArray, queue_size=10)
	rate = rospy.Rate(10)  # 10 Hz
	csv_file_path = os.path.expanduser('~/husky_ws/src/actuator_ctrl/scripts_/paths/6DOF.csv')

	try:
		with open(csv_file_path, 'r') as csvfile:
			reader = csv.reader(csvfile)
			for row in reader:
				if rospy.is_shutdown():
					break
				vec = list(map(float, row))
				msg = Float32MultiArray()
				msg.data = vec
				pubPath.publish(msg)
				rospy.loginfo(f"Published vector: {vec}")
				rate.sleep()
	except IOError:
		rospy.logerr("Failed to read the CSV file.")
if __name__ == '__main__':
	try:
		csv_reader()
	except rospy.ROSInterruptException:
		pass
	finally:
		rospy.loginfo("Node interrupted.")
