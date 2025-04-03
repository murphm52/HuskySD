import  rospy
import time
import board
import neopixel

#LED strip configuration
LED_Count=30 #Number of LEDs
LED_Pin=board.D18 #GPI018 (PMW)
Brightness=0.5 #Brightness Level

#Initialize LED strip
pixels=neopixel.NeoPixel(LED_Pin, LED_Count, brightness=Brightness, auto_write=False)

def safe_lights():
    """MAke the LED strip solid green"""
    pixels.fill((0, 255, 0)) #Green color
    pixels.show()

def safe_lights_node():
    """Initialize the ROS node for solid green lights"""
    rospy.init_node('safe_lights', anonymous=True)
    rospy.loginfo("Safe Lights Node Started")
    solid_green()
    rospy.spin() #Keeps node running

if __name__ == "__main__":
    safe_lights_node()
