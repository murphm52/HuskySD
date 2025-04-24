#!/usr/bin/env python3

## NODE DESCRIPTION ##
# Simulates actuator feedback and publishes adjusted duty cycle commands to test the exec_node.

import rospy
from std_msgs.msg import Float32
import random
import time

def simulate_feedback():
    rospy.init_node('test_feedback_node', anonymous=True)
    
    # Publishers for actuator feedback topics
    feedback_pubs = [rospy.Publisher(f"/potent_{i+1}", Float32, queue_size=10) for i in range(6)]
    
    # Publishers for adjusted duty cycle topics
    duty_cycle_pubs = [rospy.Publisher(f"/adj_duty_cycle_{i+1}", Float32, queue_size=10) for i in range(6)]
    
    rate = rospy.Rate(2)  # Publish at 2 Hz
    
    while not rospy.is_shutdown():
        # Simulate actuator feedback (random lengths between 0 and 10 units)
        feedback_values = [random.uniform(0, 10) for _ in range(6)]
        
        # Simulate adjusted duty cycles (random values between 0 and 1)
        duty_cycle_values = [random.uniform(0, 1) for _ in range(6)]
        
        # Publish feedback values
        for i in range(6):
            feedback_pubs[i].publish(feedback_values[i])
            rospy.loginfo(f"Published feedback for actuator {i+1}: {feedback_values[i]}")

        # Publish adjusted duty cycles
        for i in range(6):
            duty_cycle_pubs[i].publish(duty_cycle_values[i])
            rospy.loginfo(f"Published adjusted duty cycle for actuator {i+1}: {duty_cycle_values[i]}")

        rate.sleep()

if __name__ == '__main__':
    try:
        simulate_feedback()
    except rospy.ROSInterruptException:
        pass
