import rospy
import time
import board
import neopixel

#LED strip configuration
LED_Count=30 #Number of LEDs
LED_Pin=board.D18 #GPI018 (PWM)
Brightness=0.5 #Brightness Level

#Initialize the LED strip
pixels=neopixel.NeoPixel(LED_Pin, LED_Count, brightness=Brightness, auto_write=False)

def tesing_lights():
    """Make the LED strip solid blue"""
    pixels.fill((0, 0, 255)) #Blue color
    pixels.show()

def testing_lights_node():
    """Initialize the ROS node for solid blue"""
    rospy.init_node('testing_lights', anonymous=True)
    rospy.loginfo("Testing Lights Node Started")
    solid_blue()
    rospy.spin()

if __name__ == "__main__":
    testing_lights_node()
