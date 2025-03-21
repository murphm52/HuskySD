#!/usr/bin/env python3

import rospy
import csv
import os
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

def csv_reader():
	rospy.init_node('path_node', anonymous=True)
	pubPath = rospy.Publisher('path', Float32MultiArray, queue_size=10)
	rate = rospy.Rate(10)  # 10 Hz
	csv_file_path = os.path.expanduser('~/husky_ws/src/actuator_ctrl/scripts_/sixDOF.csv')

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
#		pubKill = rospy.Publisher('kill', boolean, queue_size=10)
#		pubKill.publish(1)
if __name__ == '__main__':
	try:
		csv_reader()
	except rospy.ROSInterruptException:
		pass
	finally:
#		pubKill = rospy.Publisher('kill', Bool, queue_size=10)
#		pubKill.publish(1)
		rospy.loginfo("Node interrupted.")
