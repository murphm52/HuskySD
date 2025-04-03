import rospy
import time
import board
import neopixel

#LED strip configuration
LED_Count=30 #Number of LEDs
LED_Pin=board.D18 #GPI018 (PWM)
Brightness=0.5 #Brightness level

#Initialize the LED strip
pixels=neopixel.NeoPixel(LED_Pin, LED_Count, brightness=Brightness, auto_write=False)

def moving_lights():
    """Make the LED strip solid red"""
    pixels.fill((255, 0, 0)) #Red color
    pixels.show()

def moving_lights_node():
    """Initialize the ROS node for solid red"""
    rospy.init_node('moving_lights', anonymous=True)
    rospy.loginfo("Moving Lights Node Started")
    solid_red()
    rospy.spin()

if __name__ == "__main__":
    moving_lights_node()
