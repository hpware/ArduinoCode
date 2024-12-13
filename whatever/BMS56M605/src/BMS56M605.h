/*****************************************************************
File:             BMS56M605.h
Author:           Weng, BESTMODULE
Description:      Define classes and required variables
History：         
V1.0.1   -- initial version；2021-06-29；Arduino IDE :v1.8.15
******************************************************************/

#ifndef _BMS56M605_H_
#define _BMS56M605_H_

#include <Wire.h>
#include <Arduino.h>
/*IIC ADDRESS*/
const uint8_t BMS56M605_IICADDR = 0x68; //pin AD0 is logic low
//const uint8_t BMS56M605_IICADDR = 0x69;  //pin AD0 is logic high

class BMS56M605
{
   public:
       BMS56M605(uint8_t intPin = 8, TwoWire *theWire = &Wire);
       void begin(uint8_t addr = BMS56M605_IICADDR);

       /*** Functional function  ***/
       //read Data
       void getEvent();
       float readTemperature();
       float readAccelerationX();
       float readAccelerationY();
       float readAccelerationZ();
       float readGyroscopeX();
       float readGyroscopeY();
       float readGyroscopeZ();
       // INT pin
       void setInterruptPinPolarity(uint8_t active_level);
       void setINT(uint8_t mode,uint8_t isEnable = false);
       uint8_t getINT();
       // other
       void writeReg(uint8_t addr, uint8_t data);
       uint8_t readReg(uint8_t addr);
       void readReg(uint8_t addr, uint8_t rBuf[], uint8_t rLen);
       void enableSleep(bool isEnable = false);
       void enableCycle(bool isEnable = false);
       void reset();

       /*** Parameter configuration and reading  ***/
       uint8_t getAccelerometerRange();
       uint8_t getGyroRange();
       uint8_t getFreefallThreshold();
       uint8_t getFreefallDuration();
       uint8_t getMotionThreshold();
       uint8_t getMotionDuration();
       uint8_t getZeroMotionThreshold();
       uint8_t getZeroMotionDuration();
       uint8_t getClock();
       uint8_t getFilterBandwidth();
       uint8_t getSampleRateDivisor();
       uint8_t getCycleRate();
       
       void setAccelerometerRange(uint8_t range);
       void setGyroRange(uint8_t range);
       void setFreefallThreshold(uint8_t threshold);
       void setFreefallDuration(uint8_t duration);
       void setMotionThreshold(uint8_t threshold);
       void setMotionDuration(uint8_t duration);
       void setZeroMotionThreshold(uint8_t threshold);
       void setZeroMotionDuration(uint8_t duration);
       void setClock(uint8_t clock);
       void setFilterBandwidth(uint8_t band);
       void setSampleRateDivisor(uint8_t divisor);
       void setCycleRate(uint8_t rate);
       
       float temperature;
       float accX, accY, accZ;
       float gyroX, gyroY, gyroZ;

   private:
       uint16_t readRawTemperature();
       void readAcceleration(float &accX, float &accY, float &accZ);
       void readGyroscope(float &gyroX, float &gyroY, float &gyroZ);
       void readRawAcceleration(int &rawAccX, int &rawAccY, int &rawAccZ);
       void readRawGyroscope(int &rawGyroX, int &rawGyroY, int &rawGyroZ);
       void clearBuf();

       void writeBytes(uint8_t wbuf[], uint8_t wlen);
       void writeRegBit(uint8_t addr,uint8_t bitNum, uint8_t bitValue);
       void readBytes(uint8_t rbuf[], uint8_t rlen);
       
       uint8_t dataBuff[10];   //Array for storing data
       TwoWire *_wire;
       uint8_t _i2caddr;
       uint8_t _intpin;
       int _lsbAcc;
       float _lsbGyro;
       uint8_t _accrange;
       uint8_t _gyrorange;
};


#define ENABLE                    1   
#define DISABLE                   0

/*set Accelerometer Range*/
#define ACC_RANGE_2G              0
#define ACC_RANGE_4G              1
#define ACC_RANGE_8G              2
#define ACC_RANGE_16G             3
/*set Gyro Range*/
#define GYRO_RANGE_250            0
#define GYRO_RANGE_500            1
#define GYRO_RANGE_1000           2
#define GYRO_RANGE_2000           3
/*set Interrupt Pin Polarity*/
#define ACTIVE_HIGH               0
#define ACTIVE_LOW                1
/*set Interrupt Pin Open*/
#define PUSH_PULL                 0
#define OPEN_DRAIN                1
/*set Clock Source*/
#define INTERAL_8MHZ              0
#define PLL_X_GYRO                1
#define PLL_Y_GYRO                2
#define PLL_Z_GYRO                3 
#define PLL_EXTERNAL_32_768KHZ    4
#define PLL_EXTERNAL_19_2MHZ      5
//#define RESERVED                  6
#define STOP_CLOCK                7
/*set Filter Bandwidth*/
#define ACC_260HZ_GYRO_256HZ      0
#define ACC_184HZ_GYRO_188HZ      1
#define ACC_96HZ_GYRO_98HZ        2
#define ACC_44HZ_GYRO_42HZ        3
#define ACC_21HZ_GYRO_20HZ        4
#define ACC_10HZ_GYRO_10HZ        5
#define ACC_5HZ_GYRO_5HZ          6
//#define RESERVED                  7

/*set Cycle Rate*/
#define F_1_25HZ                  0
#define F_2_5HZ                   1
#define F_5HZ                     2
#define F_10HZ                    3

/*INT mode*/
#define FREE_FALL_MODE            7
#define MOTION_MODE               6
#define ZERO_MOTION_MODE          5

//BMS56M605 Register
#define REG_XG_OFFS_TC           0x00 
#define REG_YG_OFFS_TC           0x01 
#define REG_ZG_OFFS_TC           0x02
#define REG_X_FINE_GAIN          0x03
#define REG_Y_FINE_GAIN          0x04
#define REG_Z_FINE_GAIN          0x05 
#define REG_XA_OFFS_H            0x06 
#define REG_XA_OFFS_L_TC         0x07
#define REG_YA_OFFS_H            0x08 
#define REG_YA_OFFS_L_TC         0x09
#define REG_ZA_OFFS_H            0x0A
#define REG_ZA_OFFS_L_TC         0x0B
#define REG_XG_OFFS_USRH         0x13 
#define REG_XG_OFFS_USRL         0x14
#define REG_YG_OFFS_USRH         0x15 
#define REG_YG_OFFS_USRL         0x16
#define REG_ZG_OFFS_USRH         0x17 
#define REG_ZG_OFFS_USRL         0x18
#define REG_SMPLRT_DIV           0x19
#define REG_CONFIG               0x1A
#define REG_GYRO_CONFIG          0x1B
#define REG_ACCEL_CONFIG         0x1C
#define REG_FF_THR               0x1D
#define REG_FF_DUR               0x1E
#define REG_MOT_THR              0x1F
#define REG_MOT_DUR              0x20
#define REG_ZRMOT_THR            0x21
#define REG_ZRMOT_DUR            0x22
#define REG_FIFO_EN              0x23
#define REG_I2C_MST_CTRL         0x24
#define REG_I2C_SLV0_ADDR        0x25
#define REG_I2C_SLV0_REG         0x26
#define REG_I2C_SLV0_CTRL        0x27
#define REG_I2C_SLV1_ADDR        0x28
#define REG_I2C_SLV1_REG         0x29
#define REG_I2C_SLV1_CTRL        0x2A
#define REG_I2C_SLV2_ADDR        0x2B
#define REG_I2C_SLV2_REG         0x2C
#define REG_I2C_SLV2_CTRL        0x2D
#define REG_I2C_SLV3_ADDR        0x2E
#define REG_I2C_SLV3_REG         0x2F
#define REG_I2C_SLV3_CTRL        0x30
#define REG_I2C_SLV4_ADDR        0x31
#define REG_I2C_SLV4_REG         0x32
#define REG_I2C_SLV4_DO          0x33
#define REG_I2C_SLV4_CTRL        0x34
#define REG_I2C_SLV4_DI          0x35
#define REG_I2C_MST_STATUS       0x36
#define REG_INT_PIN_CFG          0x37
#define REG_INT_ENABLE           0x38
#define REG_DMP_INT_STATUS       0x39
#define REG_INT_STATUS           0x3A
#define REG_ACCEL_XOUT_H         0x3B
#define REG_ACCEL_XOUT_L         0x3C
#define REG_ACCEL_YOUT_H         0x3D
#define REG_ACCEL_YOUT_L         0x3E
#define REG_ACCEL_ZOUT_H         0x3F
#define REG_ACCEL_ZOUT_L         0x40
#define REG_TEMP_OUT_H           0x41
#define REG_TEMP_OUT_L           0x42
#define REG_GYRO_XOUT_H          0x43
#define REG_GYRO_XOUT_L          0x44
#define REG_GYRO_YOUT_H          0x45
#define REG_GYRO_YOUT_L          0x46
#define REG_GYRO_ZOUT_H          0x47
#define REG_GYRO_ZOUT_L          0x48
#define REG_EXT_SENS_DATA_00     0x49
#define REG_EXT_SENS_DATA_01     0x4A
#define REG_EXT_SENS_DATA_02     0x4B
#define REG_EXT_SENS_DATA_03     0x4C
#define REG_EXT_SENS_DATA_04     0x4D
#define REG_EXT_SENS_DATA_05     0x4E
#define REG_EXT_SENS_DATA_06     0x4F
#define REG_EXT_SENS_DATA_07     0x50
#define REG_EXT_SENS_DATA_08     0x51
#define REG_EXT_SENS_DATA_09     0x52
#define REG_EXT_SENS_DATA_10     0x53
#define REG_EXT_SENS_DATA_11     0x54
#define REG_EXT_SENS_DATA_12     0x55
#define REG_EXT_SENS_DATA_13     0x56
#define REG_EXT_SENS_DATA_14     0x57
#define REG_EXT_SENS_DATA_15     0x58
#define REG_EXT_SENS_DATA_16     0x59
#define REG_EXT_SENS_DATA_17     0x5A
#define REG_EXT_SENS_DATA_18     0x5B
#define REG_EXT_SENS_DATA_19     0x5C
#define REG_EXT_SENS_DATA_20     0x5D
#define REG_EXT_SENS_DATA_21     0x5E
#define REG_EXT_SENS_DATA_22     0x5F
#define REG_EXT_SENS_DATA_23     0x60
#define REG_MOT_DETECT_STATUS    0x61
#define REG_I2C_SLV0_DO          0x63
#define REG_I2C_SLV1_DO          0x64
#define REG_I2C_SLV2_DO          0x65
#define REG_I2C_SLV3_DO          0x66
#define REG_I2C_MST_DELAY_CTRL   0x67
#define REG_SIGNAL_PATH_RESET    0x68
#define REG_MOT_DETECT_CTRL      0x69
#define REG_USER_CTRL            0x6A
#define REG_PWR_MGMT_1           0x6B
#define REG_PWR_MGMT_2           0x6C
#define REG_BANK_SEL             0x6D
#define REG_MEM_START_ADDR       0x6E
#define REG_MEM_R_W              0x6F
#define REG_DMP_CFG_1            0x70
#define REG_DMP_CFG_2            0x71
#define REG_FIFO_COUNTH          0x72
#define REG_FIFO_COUNTL          0x73
#define REG_FIFO_R_W             0x74
#define REG_WHO_AM_I             0x75



#endif
