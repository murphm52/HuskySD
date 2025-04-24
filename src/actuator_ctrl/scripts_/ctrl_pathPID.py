#!/usr/bin/env python3

## NODE DESCRIPTION ##
# Reads the CSV path row by row and publishes the data to ctrl_exec.

import rospy
import csv
import os
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import numpy as np
from simple_pid import PID

def callbackPT(data):
	global PTvec
	PTvec = np.array(data.data)
	PT1 = PTvec[0]
	PT2 = PTvec[1]
	PT3 = PTvec[2]
	PT4 = PTvec[3]
	PT5 = PTvec[4]
	PT6 = PTvec[5]
#	PTvec = [PT1, PT2, PT3, PT4, PT5, PT6]
#	print("{:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} ".format(PT1, PT2, PT3, PT4, PT5, PT6))
	return PTvec

def main():
#	global vec
	tol = 5; # mm, tolerance where PID controller will change output
	rospy.init_node('path_node', anonymous=True)
	pubPath = rospy.Publisher('path', Float32MultiArray, queue_size=10)
	rate = rospy.Rate(10)  # 10 Hz
	csv_file_path = os.path.expanduser('~/husky_ws/src/actuator_ctrl/scripts_/paths/6DOF.csv')
	
	# Initalize PID controller
	pid = PID(1.2, 0.1, 0.01, setpoint=0)
	pid.output_limits = (0,100)
	try:
		with open(csv_file_path, 'r') as csvfile:
			reader = csv.reader(csvfile)
			for row in reader:
				PT1 = PTvec[0]
				PT2 = PTvec[1]
				PT3 = PTvec[2]
				PT4 = PTvec[3]
				PT5 = PTvec[4]
				PT6 = PTvec[5]
				if rospy.is_shutdown():
					break
				vec = list(map(float, row))
				msg = Float32MultiArray()
				msg.data = vec
				L1 = vec[0]
				L2 = vec[1]
				L3 = vec[2]
				L4 = vec[3]
				L5 = vec[4]
				L6 = vec[5]
				pubPath.publish(msg)
				print("Desired Lengths: {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} ".format(L1,L2,L3,L4,L5,L6))
#				print(f"{vec}")
				print("Actual Lengths: {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} {:>5.3f} ".format(PT1, PT2, PT3, PT4, PT5, PT6))
				rate.sleep()
	except IOError:
		rospy.logerr("Failed to read the CSV file.")


if __name__ == '__main__':
	try:
		rospy.Subscriber('potent', Float32MultiArray, callbackPT)
		main()
	except rospy.ROSInterruptException:
		pass
	finally:
		rospy.loginfo("Node interrupted.")
