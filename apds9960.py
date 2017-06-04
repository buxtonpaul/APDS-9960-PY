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

    def resetGestureParameters(self, interrupts):
        self.setGestureIntEnable(0)
        self.setGestureMode(0)
        self.setMode(GESTURE, 0)

    def isGestureAvailable(self):
        val = self._device.read_byte_data(self._devaddr, APDS9960_GSTATUS)
        val &= APDS9960_GVALID
        if val == 1:
            return True
        else:
            return False

    def readGesture(self):

        if  (self.isGestureAvailable() == False) or ((self.getMode() & 0b01000001) == 0):
            return Directions.DIR_NONE
        while True:
            time.sleep(FIFO_PAUSE_TIME/1000.0) # assuming the define is in ms
            gstatus = self._device.read_byte_data(self._devaddr, APDS9960_GSTATUS)
            if (gstatus & APDS9960_GVALID) == APDS9960_GVALID:
                fifo_level = self._device.read_byte_data(self._devaddr, APDS9960_GFLVL)
                print "fifo_level = {}".format(fifo_level)
                if fifo_level > 0:
                    fifo_data = self._device.read_i2c_block_data(self._devaddr, 
                                                                APDS9960_GFIFO_U, fifo_level*4)
                    print fifo_data
                else:
                    return Directions.DIR_NONE
            else:
                return Directions.DIR_NONE
            

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

#
#/**
# * @brief Processes a gesture event and returns best guessed gesture
# *
# * @return Number corresponding to gesture. -1 on error.
# */
#int SparkFun_APDS9960::readGesture()
#{
#    uint8_t fifo_level = 0;
#    uint8_t bytes_read = 0;
#    uint8_t fifo_data[128];
#    uint8_t gstatus;
#    int motion;
#    int i;
#    
#    /* Make sure that power and gesture is on and data is valid */
#    if( !isGestureAvailable() || !(getMode() & 0b01000001) ) {
#        return DIR_NONE;
#    }
#    
#    /* Keep looping as long as gesture data is valid */
#    while(1) {
#    
#        /* Wait some time to collect next batch of FIFO data */
#        delay(FIFO_PAUSE_TIME);
#        
#        /* Get the contents of the STATUS register. Is data still valid? */
#        if( !wireReadDataByte(APDS9960_GSTATUS, gstatus) ) {
#            return ERROR;
#        }
#        
#        /* If we have valid data, read in FIFO */
#        if( (gstatus & APDS9960_GVALID) == APDS9960_GVALID ) {
#        
#            /* Read the current FIFO level */
#            if( !wireReadDataByte(APDS9960_GFLVL, fifo_level) ) {
#                return ERROR;
#            }
#
##if DEBUG
#            Serial.print("FIFO Level: ");
#            Serial.println(fifo_level);
##endif
#
#            /* If there's stuff in the FIFO, read it into our data block */
#            if( fifo_level > 0) {
#                bytes_read = wireReadDataBlock(  APDS9960_GFIFO_U, 
#                                                (uint8_t*)fifo_data, 
#                                                (fifo_level * 4) );
#                if( bytes_read == -1 ) {
#                    return ERROR;
#                }
##if DEBUG
#                Serial.print("FIFO Dump: ");
#                for ( i = 0; i < bytes_read; i++ ) {
#                    Serial.print(fifo_data[i]);
#                    Serial.print(" ");
#                }
#                Serial.println();
##endif
#
#                /* If at least 1 set of data, sort the data into U/D/L/R */
#                if( bytes_read >= 4 ) {
#                    for( i = 0; i < bytes_read; i += 4 ) {
#                        gesture_data_.u_data[gesture_data_.index] = \
#                                                            fifo_data[i + 0];
#                        gesture_data_.d_data[gesture_data_.index] = \
#                                                            fifo_data[i + 1];
#                        gesture_data_.l_data[gesture_data_.index] = \
#                                                            fifo_data[i + 2];
#                        gesture_data_.r_data[gesture_data_.index] = \
#                                                            fifo_data[i + 3];
#                        gesture_data_.index++;
#                        gesture_data_.total_gestures++;
#                    }
#                    
##if DEBUG
#                Serial.print("Up Data: ");
#                for ( i = 0; i < gesture_data_.total_gestures; i++ ) {
#                    Serial.print(gesture_data_.u_data[i]);
#                    Serial.print(" ");
#                }
#                Serial.println();
##endif
#
#                    /* Filter and process gesture data. Decode near/far state */
#                    if( processGestureData() ) {
#                        if( decodeGesture() ) {
#                            //***TODO: U-Turn Gestures
##if DEBUG
#                            //Serial.println(gesture_motion_);
##endif
#                        }
#                    }
#                    
#                    /* Reset data */
#                    gesture_data_.index = 0;
#                    gesture_data_.total_gestures = 0;
#                }
#            }
#        } else {
#    
#            /* Determine best guessed gesture and clean up */
#            delay(FIFO_PAUSE_TIME);
#            decodeGesture();
#            motion = gesture_motion_;
##if DEBUG
#            Serial.print("END: ");
#            Serial.println(gesture_motion_);
##endif
#            resetGestureParameters();
#            return motion;
#        }
#    }
#}
#
#
#/*******************************************************************************
# * High-level gesture controls
# ******************************************************************************/
#
#/**
# * @brief Resets all the parameters in the gesture data member
# */
#void SparkFun_APDS9960::resetGestureParameters()
#{
#    gesture_data_.index = 0;
#    gesture_data_.total_gestures = 0;
#    
#    gesture_ud_delta_ = 0;
#    gesture_lr_delta_ = 0;
#    
#    gesture_ud_count_ = 0;
#    gesture_lr_count_ = 0;
#    
#    gesture_near_count_ = 0;
#    gesture_far_count_ = 0;
#    
#    gesture_state_ = 0;
#    gesture_motion_ = DIR_NONE;
#}
#
#/**
# * @brief Processes the raw gesture data to determine swipe direction
# *
# * @return True if near or far state seen. False otherwise.
# */
#bool SparkFun_APDS9960::processGestureData()
#{
#    uint8_t u_first = 0;
#    uint8_t d_first = 0;
#    uint8_t l_first = 0;
#    uint8_t r_first = 0;
#    uint8_t u_last = 0;
#    uint8_t d_last = 0;
#    uint8_t l_last = 0;
#    uint8_t r_last = 0;
#    int ud_ratio_first;
#    int lr_ratio_first;
#    int ud_ratio_last;
#    int lr_ratio_last;
#    int ud_delta;
#    int lr_delta;
#    int i;
#
#    /* If we have less than 4 total gestures, that's not enough */
#    if( gesture_data_.total_gestures <= 4 ) {
#        return false;
#    }
#    
#    /* Check to make sure our data isn't out of bounds */
#    if( (gesture_data_.total_gestures <= 32) && \
#        (gesture_data_.total_gestures > 0) ) {
#        
#        /* Find the first value in U/D/L/R above the threshold */
#        for( i = 0; i < gesture_data_.total_gestures; i++ ) {
#            if( (gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
#                (gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
#                (gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
#                (gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {
#                
#                u_first = gesture_data_.u_data[i];
#                d_first = gesture_data_.d_data[i];
#                l_first = gesture_data_.l_data[i];
#                r_first = gesture_data_.r_data[i];
#                break;
#            }
#        }
#        
#        /* If one of the _first values is 0, then there is no good data */
#        if( (u_first == 0) || (d_first == 0) || \
#            (l_first == 0) || (r_first == 0) ) {
#            
#            return false;
#        }
#        /* Find the last value in U/D/L/R above the threshold */
#        for( i = gesture_data_.total_gestures - 1; i >= 0; i-- ) {
##if DEBUG
#            Serial.print(F("Finding last: "));
#            Serial.print(F("U:"));
#            Serial.print(gesture_data_.u_data[i]);
#            Serial.print(F(" D:"));
#            Serial.print(gesture_data_.d_data[i]);
#            Serial.print(F(" L:"));
#            Serial.print(gesture_data_.l_data[i]);
#            Serial.print(F(" R:"));
#            Serial.println(gesture_data_.r_data[i]);
##endif
#            if( (gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
#                (gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
#                (gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
#                (gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {
#                
#                u_last = gesture_data_.u_data[i];
#                d_last = gesture_data_.d_data[i];
#                l_last = gesture_data_.l_data[i];
#                r_last = gesture_data_.r_data[i];
#                break;
#            }
#        }
#    }
#    
#    /* Calculate the first vs. last ratio of up/down and left/right */
#    ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
#    lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
#    ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
#    lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);
#       
##if DEBUG
#    Serial.print(F("Last Values: "));
#    Serial.print(F("U:"));
#    Serial.print(u_last);
#    Serial.print(F(" D:"));
#    Serial.print(d_last);
#    Serial.print(F(" L:"));
#    Serial.print(l_last);
#    Serial.print(F(" R:"));
#    Serial.println(r_last);
#
#    Serial.print(F("Ratios: "));
#    Serial.print(F("UD Fi: "));
#    Serial.print(ud_ratio_first);
#    Serial.print(F(" UD La: "));
#    Serial.print(ud_ratio_last);
#    Serial.print(F(" LR Fi: "));
#    Serial.print(lr_ratio_first);
#    Serial.print(F(" LR La: "));
#    Serial.println(lr_ratio_last);
##endif
#       
#    /* Determine the difference between the first and last ratios */
#    ud_delta = ud_ratio_last - ud_ratio_first;
#    lr_delta = lr_ratio_last - lr_ratio_first;
#    
##if DEBUG
#    Serial.print("Deltas: ");
#    Serial.print("UD: ");
#    Serial.print(ud_delta);
#    Serial.print(" LR: ");
#    Serial.println(lr_delta);
##endif
#
#    /* Accumulate the UD and LR delta values */
#    gesture_ud_delta_ += ud_delta;
#    gesture_lr_delta_ += lr_delta;
#    
##if DEBUG
#    Serial.print("Accumulations: ");
#    Serial.print("UD: ");
#    Serial.print(gesture_ud_delta_);
#    Serial.print(" LR: ");
#    Serial.println(gesture_lr_delta_);
##endif
#    
#    /* Determine U/D gesture */
#    if( gesture_ud_delta_ >= GESTURE_SENSITIVITY_1 ) {
#        gesture_ud_count_ = 1;
#    } else if( gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1 ) {
#        gesture_ud_count_ = -1;
#    } else {
#        gesture_ud_count_ = 0;
#    }
#    
#    /* Determine L/R gesture */
#    if( gesture_lr_delta_ >= GESTURE_SENSITIVITY_1 ) {
#        gesture_lr_count_ = 1;
#    } else if( gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1 ) {
#        gesture_lr_count_ = -1;
#    } else {
#        gesture_lr_count_ = 0;
#    }
#    
#    /* Determine Near/Far gesture */
#    if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 0) ) {
#        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
#            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {
#            
#            if( (ud_delta == 0) && (lr_delta == 0) ) {
#                gesture_near_count_++;
#            } else if( (ud_delta != 0) || (lr_delta != 0) ) {
#                gesture_far_count_++;
#            }
#            
#            if( (gesture_near_count_ >= 10) && (gesture_far_count_ >= 2) ) {
#                if( (ud_delta == 0) && (lr_delta == 0) ) {
#                    gesture_state_ = NEAR_STATE;
#                } else if( (ud_delta != 0) && (lr_delta != 0) ) {
#                    gesture_state_ = FAR_STATE;
#                }
#                return true;
#            }
#        }
#    } else {
#        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
#            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {
#                
#            if( (ud_delta == 0) && (lr_delta == 0) ) {
#                gesture_near_count_++;
#            }
#            
#            if( gesture_near_count_ >= 10 ) {
#                gesture_ud_count_ = 0;
#                gesture_lr_count_ = 0;
#                gesture_ud_delta_ = 0;
#                gesture_lr_delta_ = 0;
#            }
#        }
#    }
#    
##if DEBUG
#    Serial.print("UD_CT: ");
#    Serial.print(gesture_ud_count_);
#    Serial.print(" LR_CT: ");
#    Serial.print(gesture_lr_count_);
#    Serial.print(" NEAR_CT: ");
#    Serial.print(gesture_near_count_);
#    Serial.print(" FAR_CT: ");
#    Serial.println(gesture_far_count_);
#    Serial.println("----------");
##endif
#    
#    return false;
#}
#
#/**
# * @brief Determines swipe direction or near/far state
# *
# * @return True if near/far event. False otherwise.
# */
#bool SparkFun_APDS9960::decodeGesture()
#{
#    /* Return if near or far event is detected */
#    if( gesture_state_ == NEAR_STATE ) {
#        gesture_motion_ = DIR_NEAR;
#        return true;
#    } else if ( gesture_state_ == FAR_STATE ) {
#        gesture_motion_ = DIR_FAR;
#        return true;
#    }
#    
#    /* Determine swipe direction */
#    if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 0) ) {
#        gesture_motion_ = DIR_UP;
#    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 0) ) {
#        gesture_motion_ = DIR_DOWN;
#    } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 1) ) {
#        gesture_motion_ = DIR_RIGHT;
#    } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == -1) ) {
#        gesture_motion_ = DIR_LEFT;
#    } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 1) ) {
#        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
#            gesture_motion_ = DIR_UP;
#        } else {
#            gesture_motion_ = DIR_RIGHT;
#        }
#    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == -1) ) {
#        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
#            gesture_motion_ = DIR_DOWN;
#        } else {
#            gesture_motion_ = DIR_LEFT;
#        }
#    } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == -1) ) {
#        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
#            gesture_motion_ = DIR_UP;
#        } else {
#            gesture_motion_ = DIR_LEFT;
#        }
#    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 1) ) {
#        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
#            gesture_motion_ = DIR_DOWN;
#        } else {
#            gesture_motion_ = DIR_RIGHT;
#        }
#    } else {
#        return false;
#    }
#    
#    return true;
#}
