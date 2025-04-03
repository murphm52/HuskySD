#!/usr/bin/env python3

import rospy
import csv
from std_msgs.msg import Float32MultiArray

def csv_publisher():
    # Initialize the ROS node
    rospy.init_node('csv_vector_publisher', anonymous=True)
    publisher = rospy.Publisher('csv_vectors', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(1)  # Adjust as needed for your publishing frequency

    # Open the CSV file
    with open('/home/lundyn/husky_ws/src/sixDOF.csv', 'r') as csvfile:
        reader = csv.reader(csvfile)
        
        # Iterate through each row in the CSV
        for row in reader:
            # Convert the row to a list of floats (or your required type)
            vector = list(map(float, row))
            
            # Prepare a Float32MultiArray message
            msg = Float32MultiArray()
            msg.data = vector  # Assign the vector to the message data
            
            # Publish the message
            publisher.publish(msg)
            rospy.loginfo(f"Published vector: {vector}")
            
            # Sleep to maintain the publishing rate
            rate.sleep()

if __name__ == '__main__':
    try:
        csv_publisher()
    except rospy.ROSInterruptException:
        pass
