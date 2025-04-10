import time
import board
import neopixel

#LED strip configuration
LED_COUNT=30 #Adjust based on setup
LED_PIN=board.D18 #GPI018 (PWM)
BRIGHTNESS=0.5 #Adjust brightness (0.0 to 1.0)

#Initialize the LED Strip
pixels=neopixel.NeoPixel(LED_PIN, LED_COUNT, brightness=BRIGHTNESS, auto_write=FALSE)

def color_wipe(color, wait=0.1):
    """Fill the LED strip with a single color."""
    pixels.fill(color)
    pixels.show()
    time.sleep(wait)

def rainbow_cycle(wait=0.02):
    """Create a rainbow animation across LEDs."""
    for j in range (255):
        for i in range(LED_COUNT):
            pixel_index=(i*256 // LED_COUNT) + j
            pixels[i]=wheel(pixel_index & 255)
        pixels.show()
        time.sleep(wait)

def wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos<85:
        return (pos*3, 255-pos*3, 0)
    elif pos<170:
        pos-=85
        return (255-pos*3, 0, pos*3)
    else:
        pos-=170
        return (0, pos*3, 255-pos*3)

#Run test patterns
color_wipe((255, 0, 0)) #Red
time.sleep(1)
color_wipe((0, 255, 0)) #Green
time.sleep(1)
color_wipe((0, 0, 255)) #Blue
time.sleep(1)
color_wipe((255, 255, 255)) #White
time.sleep(1)
color_wipe((0, 0, 0)) #Turn off
time.sleep(1)
rainbow_cycle() #Run rainbow cycle
