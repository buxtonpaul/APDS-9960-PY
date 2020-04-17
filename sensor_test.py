#!/usr/bin/python
# Example using a character LCD connected to a Raspberry Pi or BeagleBone Black.
import time

import apds9960 as GestureSensor
from apds9960_constants import *
import RPi.GPIO as GPIO  

SENSOR_INTERRUPT = 4

def intcallback(channel):
    #global sensor
    print("Interrupt handler called!")
    prox=sensor.readProximity()
    print("Proximity = {}".format(prox))
    sensor.clearProximityInt()

sensor = GestureSensor.APDS9960(bus=1)

sensor.initDevice()
sensor.setProximityGain(PGAIN_2X)
sensor.setProximityIntLowThreshold(0)
sensor.setProximityIntHighThreshold(100)
sensor.enableProximitySensor(1)
sensor.enableLightSensor(0)

sensor.clearProximityInt()
time.sleep(0.5)
prox = sensor.readProximity()
print("Proximity = {}".format(prox))

input("Press Enter when ready\n>")

GPIO.setmode(GPIO.BCM)

# GPIO 4 set up as an input, pulled up, pulled down on interrupt
GPIO.setup(SENSOR_INTERRUPT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(SENSOR_INTERRUPT, GPIO.FALLING, callback=intcallback)

try:
    while True:
        time.sleep(1)
        light = sensor.readAmbientLight()
        print("light={}".format(light))
        red = sensor.readRedLight()
        green = sensor.readGreenLight()
        blue = sensor.readBlueLight()
        print("Light Colors({},{},{})".format(red,green,blue))

except KeyboardInterrupt:
    GPIO.cleanup()       # clean up GPIO on CTRL+C exit

print("Done")
