# original header
#*
# * @file    SparkFun_APDS-9960.h
# * @brief   Library for the SparkFun APDS-9960 breakout board
# * @author  Shawn Hymel (SparkFun Electronics)
# *
# * @copyright	This code is public domain but you buy me a beer if you use
# * this and we meet someday (Beerware license).
# *
# * This library interfaces the Avago APDS-9960 to Arduino over I2C. The library
# * relies on the Arduino Wire (I2C) library. to use the library, instantiate an
# * APDS9960 object, call init(), and call the appropriate functions.
#
#
# APDS-9960 I2C address
APDS9960_I2C_ADDR = 0x39

# Gesture parameters
GESTURE_THRESHOLD_OUT = 10
GESTURE_SENSITIVITY_1 = 50
GESTURE_SENSITIVITY_2 = 20


# Acceptable device IDs
APDS9960_ID_1 = 0xAB
APDS9960_ID_2 = 0x9C

# Misc parameters
FIFO_PAUSE_TIME = 30      # Wait period (ms) between FIFO reads

# APDS-9960 register addresses
APDS9960_ENABLE = 0x80
APDS9960_ATIME = 0x81
APDS9960_WTIME = 0x83
APDS9960_AILTL = 0x84
APDS9960_AILTH = 0x85
APDS9960_AIHTL = 0x86
APDS9960_AIHTH = 0x87
APDS9960_PILT = 0x89
APDS9960_PIHT = 0x8B
APDS9960_PERS = 0x8C
APDS9960_CONFIG1 = 0x8D
APDS9960_PPULSE = 0x8E
APDS9960_CONTROL = 0x8F
APDS9960_CONFIG2 = 0x90
APDS9960_ID = 0x92
APDS9960_STATUS = 0x93
APDS9960_CDATAL = 0x94
APDS9960_CDATAH = 0x95
APDS9960_RDATAL = 0x96
APDS9960_RDATAH = 0x97
APDS9960_GDATAL = 0x98
APDS9960_GDATAH = 0x99
APDS9960_BDATAL = 0x9A
APDS9960_BDATAH = 0x9B
APDS9960_PDATA = 0x9C
APDS9960_POFFSET_UR = 0x9D
APDS9960_POFFSET_DL = 0x9E
APDS9960_CONFIG3 = 0x9F
APDS9960_GPENTH = 0xA0
APDS9960_GEXTH = 0xA1
APDS9960_GCONF1 = 0xA2
APDS9960_GCONF2 = 0xA3
APDS9960_GOFFSET_U = 0xA4
APDS9960_GOFFSET_D = 0xA5
APDS9960_GOFFSET_L = 0xA7
APDS9960_GOFFSET_R = 0xA9
APDS9960_GPULSE = 0xA6
APDS9960_GCONF3 = 0xAA
APDS9960_GCONF4 = 0xAB
APDS9960_GFLVL = 0xAE
APDS9960_GSTATUS = 0xAF
APDS9960_IFORCE = 0xE4
APDS9960_PICLEAR = 0xE5
APDS9960_CICLEAR = 0xE6
APDS9960_AICLEAR = 0xE7
APDS9960_GFIFO_U = 0xFC
APDS9960_GFIFO_D = 0xFD
APDS9960_GFIFO_L = 0xFE
APDS9960_GFIFO_R = 0xFF

# Bit fields
APDS9960_PON = 0b00000001
APDS9960_AEN = 0b00000010
APDS9960_PEN = 0b00000100
APDS9960_WEN = 0b00001000
APSD9960_AIEN = 0b00010000
APDS9960_PIEN = 0b00100000
APDS9960_GEN = 0b01000000
APDS9960_GVALID = 0b00000001

# On/Off definitions
OFF = 0
ON = 1

OK = 0
ERROR =0xff 


# Acceptable parameters for setMode
POWER = 0
AMBIENT_LIGHT = 1
PROXIMITY = 2
WAIT = 3
AMBIENT_LIGHT_INT = 4
PROXIMITY_INT = 5
GESTURE = 6
ALL = 7

# LED Drive values
LED_DRIVE_100MA = 0
LED_DRIVE_50MA = 1
LED_DRIVE_25MA = 2
LED_DRIVE_12_5MA = 3

# Proximity Gain (PGAIN) values
PGAIN_1X = 0
PGAIN_2X = 1
PGAIN_4X = 2
PGAIN_8X = 3

# ALS Gain (AGAIN) values
AGAIN_1X = 0
AGAIN_4X = 1
AGAIN_16X = 2
AGAIN_64X = 3

# Gesture Gain (GGAIN) values
GGAIN_1X = 0
GGAIN_2X = 1
GGAIN_4X = 2
GGAIN_8X = 3

# LED Boost values
LED_BOOST_100 = 0
LED_BOOST_150 = 1
LED_BOOST_200 = 2
LED_BOOST_300 = 3

# Gesture wait time values
GWTIME_0MS = 0
GWTIME_2_8MS = 1
GWTIME_5_6MS = 2
GWTIME_8_4MS = 3
GWTIME_14_0MS = 4
GWTIME_22_4MS = 5
GWTIME_30_8MS = 6
GWTIME_39_2MS = 7

# Default values
DEFAULT_ATIME = 219     # 103ms
DEFAULT_WTIME = 246     # 27ms
DEFAULT_PROX_PPULSE = 0x87    # 16us, 8 pulses
DEFAULT_GESTURE_PPULSE = 0x89    # 16us, 10 pulses
DEFAULT_POFFSET_UR = 0       # 0 offset
DEFAULT_POFFSET_DL = 0       # 0 offset
DEFAULT_CONFIG1 = 0x60    # No 12x wait (WTIME) factor
DEFAULT_LDRIVE = LED_DRIVE_100MA
DEFAULT_PGAIN = PGAIN_4X
DEFAULT_AGAIN = AGAIN_4X
DEFAULT_PILT = 0       # Low proximity threshold
DEFAULT_PIHT = 50      # High proximity threshold
DEFAULT_AILT = 0xFFFF  # Force interrupt for calibration
DEFAULT_AIHT = 0
DEFAULT_PERS = 0x11    # 2 consecutive prox or ALS for int.
DEFAULT_CONFIG2 = 0x01    # No saturation interrupts or LED boost
DEFAULT_CONFIG3 = 0       # Enable all photodiodes, no SAI
DEFAULT_GPENTH = 40      # Threshold for entering gesture mode
DEFAULT_GEXTH = 30      # Threshold for exiting gesture mode
DEFAULT_GCONF1 = 0x40    # 4 gesture events for int., 1 for exit
DEFAULT_GGAIN = GGAIN_4X
DEFAULT_GLDRIVE = LED_DRIVE_100MA
DEFAULT_GWTIME = GWTIME_2_8MS
DEFAULT_GOFFSET = 0       # No offset scaling for gesture mode
DEFAULT_GPULSE = 0xC9    # 32us, 10 pulses
DEFAULT_GCONF3 = 0       # All photodiodes active during gesture
DEFAULT_GIEN = 0       # Disable gesture interrupts


# Direction definitions
class Directions:
    DIR_NONE, DIR_LEFT, DIR_RIGHT, DIR_UP, DIR_DOWN, DIR_NEAR, DIR_FAR, DIR_ALL = range(8)



# State definitions
class States:
    NA_STATE, NEAR_STATE, FAR_STATE, ALL_STATEGWTIME_0MS = range(4)


# Container for gesture data
#typedef struct gesture_data_type {
#    uint8_t u_data[32];
#    uint8_t d_data[32];
#    uint8_t l_data[32];
#    uint8_t r_data[32];
#    uint8_t index;
#    uint8_t total_gestures;
#    uint8_t in_threshold;
#    uint8_t out_threshold;
#} gesture_data_type;

# APDS9960 Class
#class SparkFun_APDS9960 {
#public:

 #    Initialization methods
 #   SparkFun_APDS9960();
 #   ~SparkFun_APDS9960();
 #   bool init();
 #   uint8_t getMode();
 #   bool setMode(uint8_t mode, uint8_t enable);
 #
 #    Turn the APDS-9960 on and off
 #   bool enablePower();
 #   bool disablePower();

 #    Enable or disable specific sensors
 #   bool enableLightSensor(bool interrupts = false);
 #   bool disableLightSensor();
 #   bool enableProximitySensor(bool interrupts = false);
 #   bool disableProximitySensor();
 #   bool enableGestureSensor(bool interrupts = true);
 #  bool disableGestureSensor();

 #    LED drive strength control
 #   uint8_t getLEDDrive();
 #   bool setLEDDrive(uint8_t drive);
 #   uint8_t getGestureLEDDrive();
 #   bool setGestureLEDDrive(uint8_t drive);

 #    Gain control
 #   uint8_t getAmbientLightGain();
 #   bool setAmbientLightGain(uint8_t gain);
 #   uint8_t getProximityGain();
 #   bool setProximityGain(uint8_t gain);
 #   uint8_t getGestureGain();
 #   bool setGestureGain(uint8_t gain);
 #   Get and set light interrupt thresholds
 #   bool getLightIntLowThreshold(uint16_t &threshold);
 #   bool setLightIntLowThreshold(uint16_t threshold);
 #   bool getLightIntHighThreshold(uint16_t &threshold);
 #   bool setLightIntHighThreshold(uint16_t threshold);

 #    Get and set proximity interrupt thresholds
 #   bool getProximityIntLowThreshold(uint8_t &threshold);
 #   bool setProximityIntLowThreshold(uint8_t threshold);
 #   bool getProximityIntHighThreshold(uint8_t &threshold);
 #   bool setProximityIntHighThreshold(uint8_t threshold);

 #    Get and set interrupt enables
 #   uint8_t getAmbientLightIntEnable();
 #   bool setAmbientLightIntEnable(uint8_t enable);
 #   uint8_t getProximityIntEnable();
 #   bool setProximityIntEnable(uint8_t enable);
 #   uint8_t getGestureIntEnable();
 #   bool setGestureIntEnable(uint8_t enable);
 #   Clear interrupts
 #   bool clearAmbientLightInt();
 #   bool clearProximityInt();

 #    Ambient light methods
 #   bool readAmbientLight(uint16_t &val);
 #   bool readRedLight(uint16_t &val);
 #   bool readGreenLight(uint16_t &val);
 #   bool readBlueLight(uint16_t &val);

 #    Proximity methods
 #   bool readProximity(uint8_t &val);
 #    Gesture methods
 #   bool isGestureAvailable();
 #   int readGesture();
#private:

#     Gesture processing
#    void resetGestureParameters();
#    bool processGestureData();
#    bool decodeGesture();

#     Proximity Interrupt Threshold
#    uint8_t getProxIntLowThresh();
#    bool setProxIntLowThresh(uint8_t threshold);
#    uint8_t getProxIntHighThresh();
#    bool setProxIntHighThresh(uint8_t threshold);

#     LED Boost Control
#    uint8_t getLEDBoost();
#    bool setLEDBoost(uint8_t boost);

#     Proximity photodiode select
#    uint8_t getProxGainCompEnable();
#    bool setProxGainCompEnable(uint8_t enable);
#    uint8_t getProxPhotoMask();
#    bool setProxPhotoMask(uint8_t mask);

#     Gesture threshold control
#    uint8_t getGestureEnterThresh();
#    bool setGestureEnterThresh(uint8_t threshold);
#    uint8_t getGestureExitThresh();
#    bool setGestureExitThresh(uint8_t threshold);

#     Gesture LED, gain, and time control
#    uint8_t getGestureWaitTime();
#    bool setGestureWaitTime(uint8_t time);

#     Gesture mode
#    uint8_t getGestureMode();
#    bool setGestureMode(uint8_t mode);

#     Raw I2C Commands
#    bool wireWriteByte(uint8_t val);
#    bool wireWriteDataByte(uint8_t reg, uint8_t val);
#    bool wireWriteDataBlock(uint8_t reg, uint8_t *val, unsigned int len);
#    bool wireReadDataByte(uint8_t reg, uint8_t &val);
#    int wireReadDataBlock(uint8_t reg, uint8_t *val, unsigned int len);

#     Members
#    gesture_data_type gesture_data_;
#    int gesture_ud_delta_;
#    int gesture_lr_delta_;
#    int gesture_ud_count_;
#    int gesture_lr_count_;
#    int gesture_near_count_;
#    int gesture_far_count_;
#    int gesture_state_;
#    int gesture_motion_;
#};

