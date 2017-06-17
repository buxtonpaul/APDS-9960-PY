#/**
# * @file    SparkFun_APDS-9960.cpp
# * @brief   Library for the SparkFun APDS-9960 breakout board
# * @author  Shawn Hymel (SparkFun Electronics)
# *
# * @copyright	This code is public domain but you buy me a beer if you use
# * this and we meet someday (Beerware license).
# *
# * This library interfaces the Avago APDS-9960 to Arduino over I2C. The library
# * relies on the Arduino Wire (I2C) library. to use the library, instantiate an
# * APDS9960 object, call init(), and call the appropriate functions.
# *
# * APDS-9960 current draw tests (default parameters):
# *   Off:                   1mA
# *   Waiting for gesture:   14mA
# *   Gesture in progress:   35mA
# */
# 
# #include <Arduino.h>
# #include <Wire.h>
# 
# #include "SparkFun_APDS9960.h"
# 

from apds9960_constants import *
import time





class APDS9960:
    def __init__(self, bus=123, devaddr=APDS9960_I2C_ADDR):
        # init the bus and store the devardd
        self._devaddr = devaddr
        self._bus = bus

        self._gesture_ud_delta_ = 0
        self._gesture_lr_delta_ = 0
        self._gesture_ud_count_ = 0
        self._gesture_lr_count_ = 0
        self._gesture_near_count_ = 0
        self._gesture_far_count_ = 0
        self._gesture_state_ = States.NA_STATE
        self._gesture_motion_ = Directions.DIR_NONE
        self._device = 0
        self._devid = 0

    def initDevice(self):
        if self._bus == 123:
            print "Pretending to do something here"
            return OK

        from smbus import SMBus
        self._device = SMBus(self._bus) # 0 indicates /dev/i2c-0
        self._devid = self._device.read_byte_data(self._devaddr, APDS9960_ID)
        if  self._devid not in (APDS9960_ID_1, APDS9960_ID_2):
            print "Unknown device ID {}".format(self._devid)
            raise
        print "Device found ok"
        if self.setMode(ALL, OFF) != OK:
            print "Failed to set all oiff"
            return ERROR

        self._device.write_byte_data(self._devaddr, APDS9960_ATIME, DEFAULT_ATIME)
        self._device.write_byte_data(self._devaddr, APDS9960_WTIME, DEFAULT_WTIME)
        self._device.write_byte_data(self._devaddr, APDS9960_PPULSE, DEFAULT_PROX_PPULSE)
        self._device.write_byte_data(self._devaddr, APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR)
        self._device.write_byte_data(self._devaddr, APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL)
        self._device.write_byte_data(self._devaddr, APDS9960_CONFIG1, DEFAULT_CONFIG1)

        self.setLEDDrive(DEFAULT_LDRIVE)
        self.setProximityGain(DEFAULT_PGAIN)
        self.setAmbientLightGain(DEFAULT_AGAIN)
        self.setProxIntLowThresh(DEFAULT_PILT)
        self.setProxIntHighThresh(DEFAULT_PIHT)
        self.setLightIntLowThreshold(DEFAULT_AILT)
        self.setLightIntHighThreshold(DEFAULT_AIHT)
        self._device.write_byte_data(self._devaddr, APDS9960_PERS, DEFAULT_PERS)
        self._device.write_byte_data(self._devaddr, APDS9960_CONFIG2, DEFAULT_CONFIG2)
        self._device.write_byte_data(self._devaddr, APDS9960_CONFIG3, DEFAULT_CONFIG3)
        # Set default values for gesture sense registers
        self.setGestureEnterThresh(DEFAULT_GPENTH)
        self.setGestureExitThresh(DEFAULT_GEXTH)
        self._device.write_byte_data(self._devaddr, APDS9960_GCONF1, DEFAULT_GCONF1)
        self.setGestureGain(DEFAULT_GGAIN)
        self.setGestureLEDDrive(DEFAULT_GLDRIVE)
        self.setGestureWaitTime(DEFAULT_GWTIME)

        self._device.write_byte_data(self._devaddr, APDS9960_GOFFSET_U, DEFAULT_GOFFSET)
        self._device.write_byte_data(self._devaddr, APDS9960_GOFFSET_D, DEFAULT_GOFFSET)
        self._device.write_byte_data(self._devaddr, APDS9960_GOFFSET_L, DEFAULT_GOFFSET)
        self._device.write_byte_data(self._devaddr, APDS9960_GOFFSET_R, DEFAULT_GOFFSET)
        self._device.write_byte_data(self._devaddr, APDS9960_GPULSE, DEFAULT_GPULSE)
        self._device.write_byte_data(self._devaddr, APDS9960_GCONF3, DEFAULT_GCONF3)
        self.setGestureIntEnable(DEFAULT_GIEN)
        return OK

    def setMode(self, mode, enable):
    # sets/clears bits in the mode register for the mode requested.
        val = self.getMode()
        if val == 0xff:
            print "unable to set mode bits as mode reg = 0xff"
            return
        enable = enable & 1
        if mode >= 0 and mode <= 6:
            if enable:
                val |= 1<<mode
            else:
                val &= ~(1<<mode)
        elif mode == ALL:
            if enable:
                val = 0x7f
            else:
                val = 0
        self._device.write_byte_data(self._devaddr, APDS9960_ENABLE, val)
        return OK

    def getMode(self):
        val = self._device.read_byte_data(self._devaddr, APDS9960_ENABLE)
        return val

    def enableLightSensor(self, interrupts):
        self.setAmbientLightGain(DEFAULT_AGAIN)
        if interrupts:
            self.setAmbientLightIntEnable(1)
        else:
            self.setAmbientLightIntEnable(0)
        self.enablePower()
        self.setMode(AMBIENT_LIGHT, 1)


    def disableLightSensor(self):
        self.setAmbientLightIntEnable(0)
        self.setMode(AMBIENT_LIGHT, 0)

    def enableProximitySensor(self, interrupts):
        self.setProximityGain(DEFAULT_PGAIN)
        self.setLEDDrive(DEFAULT_LDRIVE)
        if interrupts:
            self.setProximityIntEnable(1)
        else:
            self.setProximityIntEnable(0)
        self.enablePower()
        self.setMode(PROXIMITY, 1)

    def disableProximitySensor(self):
        self.setProximityIntEnable(0)
        self.setMode(PROXIMITY, 0)

    def enableGestureSensor(self, interrupts):
        self.resetGestureParameters()
        self._device.write_byte_data(self._devaddr, APDS9960_WTIME, 0xFF)
        self._device.write_byte_data(self._devaddr, APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE)
        self.setLEDBoost(LED_BOOST_300)
        if interrupts:
            self.setGestureIntEnable(1)
        else:
            self.setGestureIntEnable(0)
        self.setGestureMode(1)
        self.enablePower()
        self.setMode(WAIT, 1)
        self.setMode(PROXIMITY, 1)
        self.setMode(GESTURE, 1)

    def resetGestureParameters(self):
        self.setGestureIntEnable(0)
        self.setGestureMode(0)
        self.setMode(GESTURE, 0)
    
        self._gesture_ud_delta_ = 0
        self._gesture_lr_delta_ = 0
        
        self._gesture_ud_count_ = 0
        self._gesture_lr_count_ = 0
        
        self._gesture_near_count_ = 0
        self._gesture_far_count_ = 0
        
        self._gesture_state_ = 0
        self._gesture_motion_ = Directions.DIR_NONE






    def isGestureAvailable(self):
        val = self._device.read_byte_data(self._devaddr, APDS9960_GSTATUS)
        val &= APDS9960_GVALID
        if val == 1:
            return True
        else:
            return False

    def readGesture(self):

        fifo_data=[]
        if  (self.isGestureAvailable() == False) or ((self.getMode() & 0b01000001) == 0):
            return Directions.DIR_NONE
        while True:
            time.sleep(FIFO_PAUSE_TIME/1000.0) # assuming the define is in ms
            gstatus = self._device.read_byte_data(self._devaddr, APDS9960_GSTATUS)
            if (gstatus & APDS9960_GVALID) == APDS9960_GVALID:
                fifo_level = self._device.read_byte_data(self._devaddr, APDS9960_GFLVL)
                #print "fifo_level = {}".format(fifo_level)
                if fifo_level > 0:
                    while fifo_level > 0:
                        fifu=self._device.read_byte_data(self._devaddr, APDS9960_GFIFO_U)
                        fifd=self._device.read_byte_data(self._devaddr, APDS9960_GFIFO_D)
                        fifl=self._device.read_byte_data(self._devaddr, APDS9960_GFIFO_L)
                        fifr=self._device.read_byte_data(self._devaddr, APDS9960_GFIFO_R)
                        fifo_entry=(fifu,fifd,fifl,fifr)
                        fifo_data.append (fifo_entry)
                        fifo_level = self._device.read_byte_data(self._devaddr, APDS9960_GFLVL)
                        #print "fifo_level = {}".format(fifo_level)
                    #print fifo_data
                    # /* Filter and process gesture data. Decode near/far state */
                    if self.processGestureData(fifo_data):
                        if self.decodeGesture():
                            pass
                            #print "not sure how we get here"
                    
                    #/* Reset data */
                    del fifo_data[:]
                else:
                    pass #return Directions.DIR_NONE
            else:
                 #        /* Determine best guessed gesture and clean up */
                time.sleep(FIFO_PAUSE_TIME/1000.0)
                self.decodeGesture()
                motion = self._gesture_motion_
                self.resetGestureParameters()
                return motion
            


    def processGestureData(self,gesture_data):
        u_first = 0
        d_first = 0
        l_first = 0
        r_first = 0
        u_last = 0
        d_last = 0
        l_last = 0
        r_last = 0

        # If we have less than 4 total gestures, that's not enough */
        if len(gesture_data) < 4:
            #print("Not enough gestures")
            return False
        
        
        #Find the first value in U/D/L/R above the threshold */
        for i in gesture_data:
           # print i
            if ((i[0] > GESTURE_THRESHOLD_OUT) and (i[1] > GESTURE_THRESHOLD_OUT)) or ( (i[2] > GESTURE_THRESHOLD_OUT) and (i[3] > GESTURE_THRESHOLD_OUT)):
                
                if u_first==0:
                    u_first = i[0]
                    d_first = i[1]
                    l_first = i[2]
                    r_first = i[3]
                else:
                   u_last = i[0]
                   d_last = i[1]
                   l_last = i[2]
                   r_last = i[3]                   

        
        if u_first==0 or u_last==0:
            #print "No gestures found"
            return False
        # Calculate the first vs. last ratio of up/down and left/right */
        ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first)
        lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first)
        ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last)
        lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last)
        
        
        # Determine the difference between the first and last ratios */
        ud_delta = ud_ratio_last - ud_ratio_first
        lr_delta = lr_ratio_last - lr_ratio_first
        
    
        #/* Accumulate the UD and LR delta values */
        self._gesture_ud_delta_ += ud_delta
        self._gesture_lr_delta_ += lr_delta

        #/* Determine U/D gesture */
        if self._gesture_ud_delta_ >= GESTURE_SENSITIVITY_1:
            self._gesture_ud_count_ = 1
        elif self._gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1:
            self._gesture_ud_count_ = -1
        else:
            self._gesture_ud_count_ = 0
        
        #/* Determine L/R gesture */
        if self._gesture_lr_delta_ >= GESTURE_SENSITIVITY_1:
            self._gesture_lr_count_ = 1
        elif self._gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1:
            self._gesture_lr_count_ = -1
        else:
            self._gesture_lr_count_ = 0
        
        
        #/* Determine Near/Far gesture */
        if self._gesture_ud_count_ == 0 and self._gesture_lr_count_ == 0 :
            
            if abs(ud_delta) < GESTURE_SENSITIVITY_2 and abs(lr_delta) < GESTURE_SENSITIVITY_2:            
                if (ud_delta == 0) and (lr_delta == 0):
                    self._gesture_near_count_+=1
                elif (ud_delta != 0) or (lr_delta != 0):
                    self._gesture_far_count_+=1             
                
                if (self._gesture_near_count_ >= 10) and (self._gesture_far_count_ >= 2):
                    #print "ud delta {},lr delta {}".format(ud_delta,lr_delta)            
                    #print "Near count{}, far count={}".format(self._gesture_near_count_,self._gesture_far_count_)
                    if (ud_delta == 0) and (lr_delta == 0):
                        self._gesture_state_ = States.NEAR_STATE
                        #print "Near State"
                    elif (ud_delta != 0) and (lr_delta != 0):
                        self._gesture_state_ = States.FAR_STATE
                        #print "Far State"
                    
                    return True
        else:
            if (abs(ud_delta) < GESTURE_SENSITIVITY_2) and (abs(lr_delta) < GESTURE_SENSITIVITY_2):
                if (ud_delta == 0) and (lr_delta == 0):
                    self._gesture_near_count_+=1
                if self._gesture_near_count_ >= 10:
                    self._gesture_ud_count_ = 0
                    self._gesture_lr_count_ = 0
                    self._gesture_ud_delta_ = 0
                    self._gesture_lr_delta_ = 0
        return False

    #/**
    #* @brief Determines swipe direction or near/far state
    #*
    #* @return True if near/far event. False otherwise.
    #*/
    def decodeGesture(self):
        #print "Decode Gesture"
        #/* Return if near or far event is detected */
        if  self._gesture_state_ == States.NEAR_STATE:
            self._gesture_motion_ = Directions.DIR_NEAR
            #print "Near"
            return True
        elif self._gesture_state_ == States.FAR_STATE:
            self._gesture_motion_ = Directions.DIR_FAR
            #print "Far"
            return True
        
        #print "UP/DOWN {} LEFT/RIGHT {}".format(self._gesture_ud_count_,self._gesture_lr_count_)
        #/* Determine swipe direction */
        if (self._gesture_ud_count_ == -1) and (self._gesture_lr_count_ == 0):
            self._gesture_motion_ = Directions.DIR_UP
        elif (self._gesture_ud_count_ == 1) and (self._gesture_lr_count_ == 0):
            self._gesture_motion_ = Directions.DIR_DOWN
        elif (self._gesture_ud_count_ == 0) and (self._gesture_lr_count_ == 1):
            self._gesture_motion_ = Directions.DIR_RIGHT
        elif (self._gesture_ud_count_ == 0) and (self._gesture_lr_count_ == -1):
            self._gesture_motion_ = Directions.DIR_LEFT
        elif (self._gesture_ud_count_ == -1) and (self._gesture_lr_count_ == 1):
            if abs(self._gesture_ud_delta_) > abs(self._gesture_lr_delta_):
                self._gesture_motion_ = Directions.DIR_UP
            else:
                self._gesture_motion_ = Directions.DIR_RIGHT
            
        elif (self._gesture_ud_count_ == 1) and (self._gesture_lr_count_ == -1):
            if abs(self._gesture_ud_delta_) > abs(self._gesture_lr_delta_):
                self._gesture_motion_ = Directions.DIR_DOWN
            else:
                self._gesture_motion_ = Directions.DIR_LEFT
        elif (self._gesture_ud_count_ == -1) and (self._gesture_lr_count_ == -1):
            if abs(self._gesture_ud_delta_) > abs(self._gesture_lr_delta_):
                self._gesture_motion_ = Directions.DIR_UP
            else:
                self._gesture_motion_ = Directions.DIR_LEFT
        elif (self._gesture_ud_count_ == 1) and (self._gesture_lr_count_ == 1):
            if abs(self._gesture_ud_delta_) > abs(self._gesture_lr_delta_):
                self._gesture_motion_ = Directions.DIR_DOWN
            else:
                self._gesture_motion_ = Directions.DIR_RIGHT
        else:
            return False
        
        
        return True
    


    def enablePower(self):
        self.setMode(POWER,1)
    
    def disablePower(self):
        self.setmode(POWER,0)



#/*******************************************************************************
# * Ambient light and color sensor controls
# ******************************************************************************/
#

    def readAmbientLight(self):
        vallow = self._device.read_byte_data(self._devaddr, APDS9960_CDATAL)
        valhi = self._device.read_byte_data(self._devaddr, APDS9960_CDATAH)
        return vallow + valhi<<8

    def readRedLight(self):
        vallow = self._device.read_byte_data(self._devaddr, APDS9960_RDATAL)
        valhi = self._device.read_byte_data(self._devaddr, APDS9960_RDATAH)
        return vallow + valhi<<8

    def readGreenLight(self):
        vallow = self._device.read_byte_data(self._devaddr, APDS9960_GDATAL)
        valhi = self._device.read_byte_data(self._devaddr, APDS9960_GDATAH)
        return vallow + valhi<<8

    def readBlueLight(self):
        vallow = self._device.read_byte_data(self._devaddr, APDS9960_BDATAL)
        valhi = self._device.read_byte_data(self._devaddr, APDS9960_BDATAH)
        return vallow + valhi<<8



    def readProximity(self):
        prox=self._device.read_byte_data(self._devaddr, APDS9960_PDATA)
        return prox


    # get/set fields within a register
    def getVal(self,reg,mask,shift):
        val = self._device.read_byte_data(self._devaddr, reg)
        return (val>>shift)&mask

    def setVal(self,reg,mask,shift,setvalue):
        val = self._device.read_byte_data(self._devaddr, reg)
        setvalue &= mask
        val &= ~(mask<<shift)
        val |= (setvalue<<shift)
        self._device.write_byte_data(self._devaddr, reg, val)


#
#/*******************************************************************************
# * Getters and setters for register values

    def getProxIntLowThresh(self):
        return self._device.read_byte_data(self._devaddr, APDS9960_PILT)

    def setProxIntLowThresh(self,val):
        self._device.write_byte_data(self._devaddr, APDS9960_PILT, val)

    def getProxIntHighThresh(self):
        return self._device.read_byte_data(self._devaddr, APDS9960_PIHT)

    def setProxIntHighThresh(self,val):
        self._device.write_byte_data(self._devaddr, APDS9960_PIHT, val)



    def getLEDDrive(self):
        # * @brief Returns LED drive strength for proximity and ALS
        # * Value    LED Current
        # *   0        100 mA
        # *   1         50 mA
        # *   2         25 mA
        # *   3         12.5 mA
        # *
        val = self.getVal(APDS9960_CONTROL,0x3,6)
        return val 


    def setLEDDrive(self,drive):
        self.setVal(APDS9960_CONTROL,0x3,6,drive)
 

    def getProximityGain(self):
        # * @brief Returns LED drive strength for proximity and ALS
        # * Value    Gain
        # *   0       1x
        # *   1       2x
        # *   2       4x
        # *   3       8x
        # *
        val = self.getVal(APDS9960_CONTROL,0x3,2)
        return val 

    def setProximityGain(self,gain):
        self.setVal(APDS9960_CONTROL,0x3,2,gain)


    def getAmbientLightGain(self):
        # * @brief Returns LED drive strength for proximity and ALS
        # * Value    Gain
        # *   0       1x
        # *   1       4x
        # *   2       16x
        # *   3       64x
        # *
        val = self.getVal(APDS9960_CONTROL,0x3,0)
        return val 

    def setAmbientLightGain(self,gain):
        self.setVal(APDS9960_CONTROL,0x3,0,gain)

    def getLEDBoost(self):
        # * Value  Boost Current
        # *   0        100%
        # *   1        150%
        # *   2        200%
        # *   3        300%
        # *
        return self.getVal(APDS9960_CONFIG2,0x3,4) 

    def setLEDBoost(self,drive):
        self.setVal(APDS9960_CONFIG2,0x3,4,drive)

    def getProxGainCompEnable(self):
        return self.getVal(APDS9960_CONFIG3,0x1,5)

    def setProxGainCompEnable(self,val):
        self.setVal(APDS9960_CONFIG3,0x1,5,val)



    def getProxPhotoMask(self):
        # * 1 = disabled, 0 = enabled
        # * Bit    Photodiode
        # *  3       UP
        # *  2       DOWN
        # *  1       LEFT
        # *  0       RIGHT
        return self.getVal(APDS9960_CONFIG3,0xf,0)

    def setProxPhotoMask(self,val):
        self.setVal(APDS9960_CONFIG3,0xf,0,val)

    def getGestureEnterThresh(self):
        return self._device.read_byte_data(self._devaddr, APDS9960_GPENTH)

    def setGestureEnterThresh(self,val):
        self._device.write_byte_data(self._devaddr, APDS9960_GPENTH, val)


    def getGestureExitThresh(self):
        return self._device.read_byte_data(self._devaddr, APDS9960_GEXTH)

    def setGestureExitThresh(self,val):
        self._device.write_byte_data(self._devaddr, APDS9960_GEXTH, val)


    def getGestureGain(self):
        # * @brief Gets the gain of the photodiode during gesture mode
        # *
        # * Value    Gain
        # *   0       1x
        # *   1       2x
        # *   2       4x
        # *   3       8x
        # *
        return self.getVal(APDS9960_GCONF2,0x3,5)

    def setGestureGain(self,val):
        self.setVal(APDS9960_GCONF2,0x3,5,val)




#
#/**
# * @brief Gets the drive current of the LED during gesture mode
# *
# * Value    LED Current
# *   0        100 mA
# *   1         50 mA
# *   2         25 mA
# *   3         12.5 mA
# *
# * @return the LED drive current value. 0xFF on error.
# */
    def getGestureLEDDrive(self):
         return self.getVal(APDS9960_GCONF2,0x3,3)
    def setGestureLEDDrive(self, val):
        self.setVal(APDS9960_GCONF2,0x3,3,val)

#
#/**
# * @brief Gets the time in low power mode between gesture detections
# *
# * Value    Wait time
# *   0          0 ms
# *   1          2.8 ms
# *   2          5.6 ms
# *   3          8.4 ms
# *   4         14.0 ms
# *   5         22.4 ms
# *   6         30.8 ms
# *   7         39.2 ms
# *
# * @return the current wait time between gestures. 0xFF on error.
# */
    def getGestureWaitTime(self):
        return self.getVal(APDS9960_GCONF2,0x7,0)

#
#/**
# * @brief Sets the time in low power mode between gesture detections
# *
# * Value    Wait time
# *   0          0 ms
# *   1          2.8 ms
# *   2          5.6 ms
# *   3          8.4 ms
# *   4         14.0 ms
# *   5         22.4 ms
# *   6         30.8 ms
# *   7         39.2 ms
# *
# * @param[in] the value for the wait time
# * @return True if operation successful. False otherwise.
# */
    def setGestureWaitTime(self, time):
        self.setVal(APDS9960_GCONF2,0x7,0,time)
#
#/**
# * @brief Gets the low threshold for ambient light interrupts
# *
# * @param[out] threshold current low threshold stored on the APDS-9960
# * @return True if operation successful. False otherwise.
# */


    def getLightIntLowThreshold(self, threshold):
        vallow = self._device.read_byte_data(self._devaddr, APDS9960_AILTL)
        valhi = self._device.read_byte_data(self._devaddr, APDS9960_AILTH)
        return vallow + valhi<<8

    def setLightIntLowThreshold(self, threshold):
        vallow = threshold & 0x00ff
        valhi = (threshold >>8) & 0x00ff 
        self._device.write_byte_data(self._devaddr, APDS9960_AILTL, vallow)
        self._device.write_byte_data(self._devaddr, APDS9960_AILTH, valhi)
  
    def getLightIntHighThreshold(self, threshold):
        vallow = self._device.read_byte_data(self._devaddr, APDS9960_AIHTL)
        valhi = self._device.read_byte_data(self._devaddr, APDS9960_AIHTL)
        return vallow + valhi<<8

    def setLightIntHighThreshold(self, threshold):
        vallow = threshold & 0x00ff
        valhi = (threshold >>8) & 0x00ff 
        self._device.write_byte_data(self._devaddr, APDS9960_AIHTL, vallow)
        self._device.write_byte_data(self._devaddr, APDS9960_AIHTL, valhi)


    def getProximityIntLowThreshold(self):
        return self._device.read_byte_data(self._devaddr, APDS9960_PILT)

    def setProximityIntLowThreshold(self,val):
        self._device.write_byte_data(self._devaddr, APDS9960_PILT, val)

    def getProximityIntHighThreshold(self):
        return self._device.read_byte_data(self._devaddr, APDS9960_PIHT)

    def setProximityIntHighThreshold(self,val):
        self._device.write_byte_data(self._devaddr, APDS9960_PIHT, val)

    def getAmbientLightIntEnable(self):
        return self.getVal(APDS9960_ENABLE,0x1,4)

    def setAmbientLightIntEnable(self, val):
        self.setVal(APDS9960_ENABLE,0x1,4,val)

    def getProximityIntEnable(self):
        return self.getVal(APDS9960_ENABLE,0x1,5)

    def setProximityIntEnable(self, val):
        self.setVal(APDS9960_ENABLE,0x1,5,val)

    def getGestureIntEnable(self):
        return self.getVal(APDS9960_GCONF4,0x1,1)

    def setGestureIntEnable(self, val):
        self.setVal(APDS9960_GCONF4,0x1,1,val)

    def clearAmbientLightInt(self):
        self._device.read_byte_data(self._devaddr, APDS9960_AICLEAR)

    def clearProximityInt(self):
        self._device.read_byte_data(self._devaddr, APDS9960_PICLEAR)

    def getGestureMode(self):
        return self.getVal(APDS9960_GCONF4,0x1,0)

    def setGestureMode(self, val):
        self.setVal(APDS9960_GCONF4,0x1,0,val)

