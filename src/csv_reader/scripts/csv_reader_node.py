#!/usr/bin/env python

import rospy
import csv
from std_msgs.msg import String

def csv_reader():
    rospy.init_node('csv_reader', anonymous=True)
    pub = rospy.Publisher('csv_data', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Adjust the path to your CSV file
    csv_file_path = '/media/lundyn/USB_Black/sixDOF.csv'

    try:
        with open(csv_file_path, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            for row in csvreader:
                if rospy.is_shutdown():
                    break
                # Convert row data into a single string
                data_str = ', '.join(row)
                rospy.loginfo(f"Publishing: {data_str}")
                pub.publish(data_str)
                rate.sleep()
    except IOError:
        rospy.logerr("Failed to read the CSV file.")

if __name__ == '__main__':
    try:
        csv_reader()
    except rospy.ROSInterruptException:
        pass

