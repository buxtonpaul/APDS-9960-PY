
#!/usr/bin/python
# Example using a character LCD connected to a Raspberry Pi or BeagleBone Black.
import time

import apds9960 as GestureSensor
from apds9960_constants import *
import RPi.GPIO as GPIO  



SENSOR_INTERRUPT = 4

def intcallback(channel):
    #print "Callback"
    while sensor.isGestureAvailable():
        motion=sensor.readGesture()
        if motion == Directions.DIR_NONE:
            print "None"
        if motion == Directions.DIR_LEFT:
            print "Left"
        if motion == Directions.DIR_RIGHT:
            print "Right"
        if motion == Directions.DIR_UP:
            print "Up"
        if motion == Directions.DIR_DOWN:
            print "Down"
        if motion == Directions.DIR_NEAR:
            print "Near"
        if motion == Directions.DIR_FAR:
            print "Far"
        sensor.enableGestureSensor(True)


sensor = GestureSensor.APDS9960(bus=1)

sensor.initDevice()
sensor.resetGestureParameters()


# These values work better for me than the defaults, perhaps because I am not using the Adafruit one...
sensor.setGestureGain(GGAIN_2X)
sensor.setGestureLEDDrive(LED_DRIVE_25MA)
  
sensor.enableGestureSensor(True)

#sensor.clearGestureInt()
time.sleep(0.5)

raw_input("Press Enter when ready\n>")

GPIO.setmode(GPIO.BCM)

# GPIO 4 set up as an input, pulled up, pulled down on interrupt
GPIO.setup(SENSOR_INTERRUPT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(SENSOR_INTERRUPT, GPIO.FALLING, callback=intcallback)

if sensor.isGestureAvailable():
    sensor.readGesture()


try:
    while True:
        time.sleep(1)
        #light = sensor.readAmbientLight()
        #print "light={}".format(light)
        #red = sensor.readRedLight()
        #green = sensor.readGreenLight()
        #blue = sensor.readBlueLight()
        #print "Light Colors({},{},{})".format(red,green,blue)

except KeyboardInterrupt:
    GPIO.cleanup()       # clean up GPIO on CTRL+C exit

sensor.resetGestureParameters()
print "Done"
