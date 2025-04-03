import rospy
from std_msgs.msg
import String
import time
import board
import neopixel

#LED strip configuration
LED_Count=30 #Number of LEDs
LED_Pin=board.D18 #GPI018
Brightness=0.5 #Brightness Level

#Initialize the LED strip
pixels=neopixel.NeoPixel(LED_Pin, LED_Count, brightness=Brightness, auto_write=False)

def emergency_lights():
    """Blink the LED strip red"""
    while not rospy.is_shutdown():
        pixels.fill((255, 0, 0)) #Red color
        pixels.show()
        time.sleep(1)
        pixels.fill((0, 0, 0))
        pixels.show()
        time.sleep(1)

def emergency_lights_node():
    """Initialize the ROS node to blink red"""
    rospy.init_node('emergency_lights', anonymous=True)
    rospy.loginfo("Emergency Lights Node Started")
    blink_red()

if __name__ == "__main__":
    emergency_lights_node()
