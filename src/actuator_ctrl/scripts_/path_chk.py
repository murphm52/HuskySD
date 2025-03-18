#!/usr/bin/env python3

import rospy
import csv
import os
from std_msgs.msg import Float32MultiArray

def csv_reader():
	rospy.init_node('path_chk', anonymous=True)
	pub = rospy.Publisher('path', Float32MultiArray, queue_size=10)
	rate = rospy.Rate(10)  # 10 Hz

	# Adjust the path to your CSV file
	csv_file_path = os.path.expanduser('~/husky_ws/src/actuator_ctrl/scripts_/sixDOF.csv')

	try:
		with open(csv_file_path, 'r') as csvfile:
			reader = csv.reader(csvfile)
			for row in reader:
				vec = list(map(float, row))
				msg = Float32MultiArray
				msg.data = vec
				pub.publish(msg)
				rospy.loginfo(f"Published vector: {vec}")
				rate.sleep()
	except IOError:
		rospy.logerr("Failed to read the CSV file.")

if __name__ == '__main__':
	try:
		csv_reader()
	except rospy.ROSInterruptException:
		pass
