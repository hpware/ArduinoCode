/*****************************************************************
File:             BMS56M605.cpp
Author:           Weng, BESTMODULE
Description:      IIC communication with the sensor and obtain the corresponding value  
History：         
V1.0.1   -- initial version；2021-06-29；Arduino IDE :v1.8.15
******************************************************************/
#include "BMS56M605.h"


/**********************************************************
Description: Constructor
Parameters:  intPin:INT Output pin connection with Arduino         
Return:      none    
Others:      none
**********************************************************/
BMS56M605::BMS56M605(uint8_t intPin, TwoWire *theWire)
{
   _intpin = intPin;
   _wire = theWire;
}
/**********************************************************
Description: Module Initial
Parameters:  addr :Module IIC address
             theWire : Wire object if your board has more than one I2C interface 
Return:      void    
Others:      none
**********************************************************/
void BMS56M605::begin(uint8_t addr)
{
     pinMode(_intpin,INPUT);
     _i2caddr = addr;
     _wire->begin();
     enableSleep(false);  //make sure that the device is not in sleep mode
     setClock(PLL_X_GYRO);//PLL with X axis gyroscope reference
     setAccelerometerRange(ACC_RANGE_2G);//the full scale range of the accelerometer:± 2g
     setGyroRange(GYRO_RANGE_250);//the full scale range of the gyroscope:± 250 °/s
     writeRegBit(REG_INT_PIN_CFG, 6, PUSH_PULL & 0x01); //the INT pin is configured as push-pull.
     setInterruptPinPolarity(ACTIVE_LOW);
}

/**********************************************************
Description: read data include temperature/acc and gyro 3 axis data
Parameters:  none
Return:      void    
Others:      none
**********************************************************/
void BMS56M605::getEvent()
{
    temperature = readTemperature();   delay(10);
    readAcceleration(accX, accY, accZ);delay(10);
    readGyroscope(gyroX, gyroY, gyroZ);delay(10);
}
/**********************************************************
Description: read Temperature
Parameters:  none
Return:      temperature data(unit: ℃)    
Others:      none
**********************************************************/
float BMS56M605::readTemperature()
{
      float tempValue = 0;
      readReg(REG_TEMP_OUT_H,dataBuff,2);
      tempValue = (uint16_t)dataBuff[0]*256 + dataBuff[1];
      if(tempValue != 0) tempValue = tempValue/340 + 36.53;
      tempValue /= 10;
      return tempValue;
}

/**********************************************************
Description: read X-axis Acceleration 
Parameters:  none
Return:      data of X-axis Acceleration (unit: g)   
Others:      none
**********************************************************/
float BMS56M605::readAccelerationX()
{
      int16_t x = 0;
      readReg(REG_ACCEL_XOUT_H,dataBuff,2);
      x = dataBuff[0]*256 + dataBuff[1];
      return (float)x/_lsbAcc;
      
}
/**********************************************************
Description: read Y-axis Acceleration 
Parameters:  none
Return:      data of Y-axis Acceleration (unit: g)   
Others:      none
**********************************************************/
float BMS56M605::readAccelerationY()
{
      int16_t y = 0;
      readReg(REG_ACCEL_YOUT_H,dataBuff,2);
      y = dataBuff[0]*256 + dataBuff[1];
      return (float)y/_lsbAcc;      
}
/**********************************************************
Description: read Z-axis Acceleration 
Parameters:  none
Return:      data of Z-axis Acceleration (unit: g)   
Others:      none
**********************************************************/
float BMS56M605::readAccelerationZ()
{
      int16_t z = 0;
      readReg(REG_ACCEL_ZOUT_H,dataBuff,2);
      z = dataBuff[0]*256 + dataBuff[1];
      return (float)z/_lsbAcc;      
}
/**********************************************************
Description: read X-axis Gyroscope 
Parameters:  none
Return:      data of X-axis Gyroscope (unit: °/s)   
Others:      none
**********************************************************/
float BMS56M605::readGyroscopeX()
{
      int16_t x = 0;
      readReg(REG_GYRO_XOUT_H,dataBuff,2);
      x = dataBuff[0]*256 + dataBuff[1];
      return (float)x/_lsbGyro;
}
/**********************************************************
Description: read Y-axis Gyroscope 
Parameters:  none
Return:      data of Y-axis Gyroscope (unit: °/s)   
Others:      none
**********************************************************/
float BMS56M605::readGyroscopeY()
{
      int16_t y = 0;
      readReg(REG_GYRO_YOUT_H,dataBuff,2); 
      y = dataBuff[0]*256 + dataBuff[1];
      return (float)y/_lsbGyro;  
}
/**********************************************************
Description: read Z-axis Gyroscope 
Parameters:  none
Return:      data of Z-axis Gyroscope (unit: °/s)   
Others:      none
**********************************************************/
float BMS56M605::readGyroscopeZ()
{
      int16_t z = 0;
      readReg(REG_GYRO_ZOUT_H,dataBuff,2); 
      z = dataBuff[0]*256 + dataBuff[1];
      return (float)z/_lsbGyro;  
}

/**********************************************************
Description: set Interrupt Pin Polarity 
Parameters:  active_level:Optional:
              ACTIVE_HIGH 
              ACTIVE_LOW 
Return:      void    
Others:   When this bit is equal to 0, the logic level for the INT pin is active high.
          When this bit is equal to 1, the logic level for the INT pin is active low.
**********************************************************/
void BMS56M605::setInterruptPinPolarity(uint8_t active_level)
{
       writeRegBit(REG_INT_PIN_CFG, 7, active_level & 0x01); 
}

/**********************************************************
Description:  set INT Mode
Parameters:   mode:
                FREE_FALL_MODE            
                MOTION_MODE               
                ZERO_MOTION_MODE
              isEnable:
                true:enable
                false:disable
Return:       void    
Others:       none
**********************************************************/
void BMS56M605::setINT(uint8_t mode,uint8_t isEnable)
{
     if(isEnable == true)
     {
          writeRegBit(REG_INT_ENABLE, mode, ENABLE);
     }
     if(isEnable == false)
     {
          writeRegBit(REG_INT_ENABLE, mode, DISABLE);
     }
}
/**********************************************************
Description: get INT
Parameters:  none
Return:      INT status  
Others:      1:INT status is HIGH
             0:INT status is LOW
**********************************************************/
uint8_t BMS56M605::getINT()
{
      uint8_t statusValue = 0;
      statusValue = digitalRead(_intpin);
      return statusValue;
}

/**********************************************************
Description: writeReg
Parameters:  addr :Register to be written
             data:Value to be written  
Return:      void   
Others:      none
**********************************************************/
void BMS56M605::writeReg(uint8_t addr, uint8_t data)
{
    uint8_t sendBuf[2] = {addr,data};
    writeBytes(sendBuf,2);
    delay(1);
}
/**********************************************************
Description: read Register data
Parameters:  addr :Register to be written    
Return:      8-bit data of Register
Others:      user can use this function to read any register  
             including something are not mentioned.
**********************************************************/
uint8_t  BMS56M605::readReg(uint8_t addr)
{
    clearBuf();
    uint8_t sendBuf[1] = {addr};
    writeBytes(sendBuf,1);
    delay(1);
    readBytes(dataBuff,1);
    delay(1);
    return dataBuff[0];
}
/**********************************************************
Description: Read register value continuously
Parameters:  addr:Register to be written
             rBuf:Variables for storing Data to be obtained
             rLen:the byte of the data   
Return:      void    
Others:      none
**********************************************************/
void BMS56M605::readReg(uint8_t addr, uint8_t rBuf[], uint8_t rLen)
{
    clearBuf();
    uint8_t sendBuf[1] = {addr};
    writeBytes(sendBuf,1);
    delay(1);
    readBytes(rBuf,rLen);
    delay(1);
}
/**********************************************************
Description: enable Sleep Mode
Parameters:  isEnable =  ture,  enable
             isEnable =  false, disable(default)  
Return:      void    
Others:   When enable, puts the DEVICE into sleep mode.
**********************************************************/
void BMS56M605::enableSleep(bool isEnable)
{
  if(isEnable == true)
  {
      writeRegBit(REG_PWR_MGMT_1, 6, ENABLE);
  }
  if(isEnable == false)
  {
      writeRegBit(REG_PWR_MGMT_1, 6, DISABLE);
  }
}
/**********************************************************
Description: enable Cycle Mode
Parameters:  isEnable =  ture,  enable 
             isEnable =  false, disable(default) 
Return:      void    
Others:   When this bit is set to 1 and SLEEP is disabled, 
          the MPU-60X0 will cycle between sleep mode and waking up 
          to take a single sample of data from active sensors 
          at a rate determined by LP_WAKE_CTRL (register 108).
**********************************************************/
void BMS56M605::enableCycle(bool isEnable)
{
   if(isEnable == true)
  {
      writeRegBit(REG_PWR_MGMT_1, 5, ENABLE);
  }
  if(isEnable == false)
  {
      writeRegBit(REG_PWR_MGMT_1, 5, DISABLE);
  }  
}
/**********************************************************
Description: reset
Parameters:       
Return:      void   
Others:   When set to 1, this bit resets all internal registers 
          to their default values.The bit automatically clears 
          to 0 once the reset is done.
**********************************************************/
void BMS56M605::reset()
{
     writeRegBit(REG_PWR_MGMT_1, 7, 1);
}
/**********************************************************
Description: get Accelerometer Range
Parameters:  none
Return:      Accelerometer Range data(2 bit)              
Others:      none
**********************************************************/
uint8_t BMS56M605::getAccelerometerRange()
{
      uint8_t rangeValue = 0;
      readReg(REG_ACCEL_CONFIG,dataBuff,1);
      rangeValue = (dataBuff[0]>>3)&0x03; //get bit4 and bit3   
      return rangeValue;
}
/**********************************************************
Description: get Gyroscope Range
Parameters:  none
Return:      Gyroscope Range data(2 bit)   
Others:      none
**********************************************************/
uint8_t BMS56M605::getGyroRange()
{
      uint8_t rangeValue = 0;
      readReg(REG_GYRO_CONFIG,dataBuff,1);
      rangeValue = (dataBuff[0]>>3)&0x03; //get bit4 and bit3   
      return rangeValue;
}
/**********************************************************
Description: get Free fall Threshold
Parameters:  none
Return:      Threshold data(1 byte) ,unit:mg
Others:      when FREE_FALL_Mode is set
**********************************************************/
uint8_t BMS56M605::getFreefallThreshold()
{
      uint8_t thresholdValue = 0;
      readReg(REG_FF_THR,dataBuff,1); 
      thresholdValue = dataBuff[0];
      return thresholdValue;
}
/**********************************************************
Description: get Motion Threshold
Parameters:  none
Return:      Threshold data(1 byte)  ,unit:mg   
Others:      when Motion_Mode is set
**********************************************************/
uint8_t BMS56M605::getMotionThreshold()
{
      uint8_t thresholdValue = 0;
      readReg(REG_MOT_THR,dataBuff,1);   
      thresholdValue = dataBuff[0];  
      return thresholdValue;
}
/**********************************************************
Description: get Free fall Duration
Parameters:  none
Return:      Duration data(1 byte)   ,unit:ms
Others:      when FREE_FALL_Mode is set
**********************************************************/
uint8_t BMS56M605::getFreefallDuration()
{
      uint8_t durationValue = 0;
      readReg(REG_FF_DUR,dataBuff,1); 
      durationValue = dataBuff[0]; 
      return durationValue;
}
/**********************************************************
Description: get Motion Duration
Parameters:  none
Return:      Duration data(1 byte),unit:ms    
Others:      when Motion_Mode is set
**********************************************************/
uint8_t BMS56M605::getMotionDuration()
{
      uint8_t durationValue = 0;
      readReg(REG_MOT_DUR,dataBuff,1);
      durationValue = dataBuff[0]; 
      return durationValue;
}
/**********************************************************
Description: get Zero Motion Threshold
Parameters:  none
Return:      Threshold data(1 byte)   ,unit:mg   
Others:      when Zero_Motion_Mode is set
**********************************************************/
uint8_t BMS56M605::getZeroMotionThreshold()
{
      uint8_t thresholdValue = 0;
      readReg(REG_ZRMOT_THR,dataBuff,1);
      thresholdValue = dataBuff[0]; 
      return thresholdValue;
}
/**********************************************************
Description: get Zero Motion Duration
Parameters:  none
Return:      Duration data(1 byte)  ,unit:ms 
Others:      when Zero_Motion_Mode is set
**********************************************************/
uint8_t BMS56M605::getZeroMotionDuration()
{
      uint8_t durationValue = 0;
      readReg(REG_ZRMOT_DUR,dataBuff,1);
      durationValue = dataBuff[0]; 
      return durationValue;
}
/**********************************************************
Description: get Clock Source
Parameters:  none
Return:      Clock Source data(3 bit)   
Others:      none
**********************************************************/
uint8_t BMS56M605::getClock()
{
      uint8_t clockValue = 0;
      readReg(REG_PWR_MGMT_1,dataBuff,1);
      clockValue = dataBuff[0]&0x07; 
      return clockValue;
}
/**********************************************************
Description: get Filter Band width
Parameters:  bandWidthValue:Variables for storing Filter Band width data
Return:      Filter Band width data(3 bit)   
Others:      none
**********************************************************/
uint8_t BMS56M605::getFilterBandwidth()
{
      uint8_t bandWidthValue  = 0;
      readReg(REG_CONFIG,dataBuff,1);
      bandWidthValue = dataBuff[0]&0x07; 
      return bandWidthValue;
}
/**********************************************************
Description: get Sample Rate Divisor
Parameters:  none
Return:      Rate divisor data(1 Byte)   
Others:      none
**********************************************************/
uint8_t BMS56M605::getSampleRateDivisor()
{
      uint8_t divisorValue = 0;
      readReg(REG_SMPLRT_DIV,dataBuff,1);
      divisorValue = dataBuff[0];
      return divisorValue;
}
/**********************************************************
Description: get Cycle Rate
Parameters:  none
Return:      Cycle Rate data(2 bit)   
Others:      none
**********************************************************/
uint8_t BMS56M605::getCycleRate()
{
      uint8_t rateValue = 0; 
      readReg(REG_PWR_MGMT_2,dataBuff,1);
      rateValue = (dataBuff[0]>>6)&0x03; 
      return rateValue;
}


/**********************************************************
Description: set Accelerometer Range
Parameters:  range:Optional:
              ACC_RANGE_2G              
              ACC_RANGE_4G              
              ACC_RANGE_8G              
              ACC_RANGE_16G 
Return:      void    
Others:      none
**********************************************************/
void BMS56M605::setAccelerometerRange(uint8_t range)
{
      _accrange = range;
            
      switch(_accrange)
      {
          case ACC_RANGE_2G: _lsbAcc = 16384;break;
          case ACC_RANGE_4G: _lsbAcc = 8192; break;
          case ACC_RANGE_8G: _lsbAcc = 4096; break;
          case ACC_RANGE_16G:_lsbAcc = 2048; break;
          default:           _lsbAcc = 16384;break;
      }
      writeRegBit(REG_ACCEL_CONFIG, 3, range & 0x01); 
      writeRegBit(REG_ACCEL_CONFIG, 4, range & 0x02);      
}
/**********************************************************
Description: set Gyroscope Range
Parameters:  range:Optional:
              GYRO_RANGE_250            
              GYRO_RANGE_500            
              GYRO_RANGE_1000           
              GYRO_RANGE_2000  
Return:      void    
Others:      none
**********************************************************/
void BMS56M605::setGyroRange(uint8_t range)
{
      _gyrorange = range;
      switch(_gyrorange)
      {
          case GYRO_RANGE_250:  _lsbGyro = 131 ;break;
          case GYRO_RANGE_500:  _lsbGyro = 65.5;break;
          case GYRO_RANGE_1000: _lsbGyro = 32.8;break;
          case GYRO_RANGE_2000: _lsbGyro = 16.4;break;
          default:              _lsbGyro = 131 ;break;
      }
      writeRegBit(REG_GYRO_CONFIG, 3, range & 0x01); 
      writeRegBit(REG_GYRO_CONFIG, 4, range & 0x02);      
}
/**********************************************************
Description: set Free fall Threshold
Parameters:  threshold:Variables for storing Threshold data,unit:mg
Return:      void   
Others:      when FREE_FALL_Mode is set
	           1 LSB = 1mg
**********************************************************/
void  BMS56M605::setFreefallThreshold(uint8_t threshold)
{
      writeReg(REG_FF_THR, threshold);
}
/**********************************************************
Description: set Free fall Duration
Parameters:  duration:Variables for storing Duration data,unit:ms
Return:      void    
Others:      when FREE_FALL_Mode is set,unit:ms
**********************************************************/
void  BMS56M605::setFreefallDuration(uint8_t duration)
{
      writeReg(REG_FF_DUR, duration);
}
/**********************************************************
Description: set Motion Threshold
Parameters:  threshold:Variables for storing Threshold data ,unit:mg
Return:      void   
Others:      when Motion_Mode is set
             1 LSB = 1mg
**********************************************************/
void  BMS56M605::setMotionThreshold(uint8_t threshold)
{
      writeReg(REG_MOT_THR, threshold);
}
/**********************************************************
Description: set Motion Duration
Parameters:  duration:Variables for storing Duration data ,unit:ms
Return:      void    
Others:      when Motion_Mode is set,unit:ms
**********************************************************/
void  BMS56M605::setMotionDuration(uint8_t duration)
{
      writeReg(REG_MOT_DUR, duration);
}
/**********************************************************
Description: set Zero Motion Threshold
Parameters:  threshold:Variables for storing Threshold data,unit:mg
Return:      void   
Others:      when Zero_Motion_Mode is set
             1 LSB = 1mg
**********************************************************/
void  BMS56M605::setZeroMotionThreshold(uint8_t threshold)
{
      writeReg(REG_ZRMOT_THR, threshold);
}
/**********************************************************
Description: set Zero Motion Duration
Parameters:  duration:Variables for storing Duration data ,unit:ms
Return:      void  
Others:      when Zero_Motion_Mode is set,unit:ms
**********************************************************/
void  BMS56M605::setZeroMotionDuration(uint8_t duration)
{
      writeReg(REG_ZRMOT_DUR, duration);
}
/**********************************************************
Description: set Clock Source
Parameters:  clock:Optional:
              INTERAL_8MHZ              
              PLL_X_GYRO                
              PLL_Y_GYRO                
              PLL_Z_GYRO                
              PLL_EXTERNAL_32_768KHZ    
              PLL_EXTERNAL_19_2MHZ                
              STOP_CLOCK 
Return:      void    
Others:   It is highly recommended that the device be configured 
          to use one of the gyroscopes (or an external clock source) 
          as the clock reference for improved stability.
**********************************************************/
void BMS56M605::setClock(uint8_t clock)
{
      writeRegBit(REG_PWR_MGMT_1, 0, clock & 0x01);
      writeRegBit(REG_PWR_MGMT_1, 1, clock & 0x02);
      writeRegBit(REG_PWR_MGMT_1, 2, clock & 0x04);
}
/**********************************************************
Description: set Filter Band width
Parameters:  band:Optional:
              ACC_260HZ_GYRO_256HZ      
              ACC_184HZ_GYRO_188HZ      
              ACC_96HZ_GYRO_98HZ        
              ACC_44HZ_GYRO_42HZ        
              ACC_21HZ_GYRO_20HZ        
              ACC_10HZ_GYRO_10HZ        
              ACC_5HZ_GYRO_5HZ 
Return:      void    
Others:      none
**********************************************************/
void BMS56M605::setFilterBandwidth(uint8_t band)
{
      writeRegBit(REG_CONFIG, 0, band & 0x01);
      writeRegBit(REG_CONFIG, 1, band & 0x02);
      writeRegBit(REG_CONFIG, 2, band & 0x04);        
}
/**********************************************************
Description: set Sample Rate Divisor
Input:       divisor: Sample Rate Divisor 
Output:      none                          
Return:      void    
Others:   Sample Rate = Gyroscope Output Rate / (1 + divisor).
          where Gyroscope Output Rate = 8kHz when the DLPF 
          is disabled (DLPF_CFG = 0 or 7), and 1kHz when the 
          DLPF is enabled (see Register 26).
**********************************************************/
void BMS56M605::setSampleRateDivisor(uint8_t divisor)
{
     writeReg(REG_SMPLRT_DIV, divisor); 
}
/**********************************************************
Description: set Cycle Mode
Parameters:  rate:Optional:
              F_1_25HZ                  
              F_2_5HZ                   
              F_5HZ                     
              F_10HZ
Return:      void   
Others:      none
**********************************************************/
void BMS56M605::setCycleRate(uint8_t rate)
{
     writeRegBit(REG_PWR_MGMT_2, 6, rate & 0x01);
     writeRegBit(REG_PWR_MGMT_2, 7, rate & 0x02);
}

/**********************************************************
Description: read Raw Temperature
Parameters:  none
Return:      temperature A/D raw data(2 byte)    
Others:      none
**********************************************************/
uint16_t BMS56M605::readRawTemperature()
{
      uint16_t rawTempValue = 0;
      readReg(REG_TEMP_OUT_H,dataBuff,2); 
      rawTempValue = dataBuff[0]*256 + dataBuff[1];
      return rawTempValue;
}
/**********************************************************
Description: read Acceleration
Parameters:  accX/accY/accZ :Variables for storing Acceleration 3 axis data
             Acceleration 3 axis data(unit: g)  
Return:      void   
Others:      none
**********************************************************/
void BMS56M605::readAcceleration(float &accX, float &accY, float &accZ)
{

      int16_t x,y,z;
      readReg(REG_ACCEL_XOUT_H,dataBuff,6); 
      x = dataBuff[0]*256 + dataBuff[1];
      accX = (float)x/_lsbAcc;
      y = dataBuff[2]*256 + dataBuff[3];
      accY = (float)y/_lsbAcc;
      z = dataBuff[4]*256 + dataBuff[5];
      accZ = (float)z/_lsbAcc;
}
/**********************************************************
Description: read Gyroscope
Parameters:  gyroX/gyroY/gyroZ :Variables for storing Gyroscope 3 axis data
             Gyroscope 3 axis data(unit: °/s)  
Return:      void    
Others:      none
**********************************************************/
void BMS56M605::readGyroscope(float &gyroX, float &gyroY, float &gyroZ)
{
      int16_t x,y,z;
      readReg(REG_GYRO_XOUT_H,dataBuff,6);
      x = dataBuff[0]*256 + dataBuff[1];
      gyroX = (float)x/_lsbGyro;
      y = dataBuff[2]*256 + dataBuff[3];
      gyroY = (float)y/_lsbGyro;
      z = dataBuff[4]*256 + dataBuff[5];
      gyroZ = (float)z/_lsbGyro;
}
/**********************************************************
Description: read Raw Acceleration
Parameters:  rawAccX/rawAccY/rawAccZ :Variables for storing Acceleration 3 axis raw data
Output:      Acceleration 3 axis raw data(2 byte) 
Return:      void   
Others:      none
**********************************************************/
void BMS56M605::readRawAcceleration(int &rawAccX, int &rawAccY, int &rawAccZ)
{
      readReg(REG_ACCEL_XOUT_H,dataBuff,6); 
      rawAccX = dataBuff[0]*256 + dataBuff[1];
      rawAccY = dataBuff[2]*256 + dataBuff[3];
      rawAccZ = dataBuff[4]*256 + dataBuff[5];
}
/**********************************************************
Description: read Raw Gyroscope
Input:       rawGyroX/rawGyroY/rawGyroZ :Variables for storing Gyroscope 3 axis raw data
Output:      Gyroscope 3 axis raw data(2 byte)   
Return:      void    
Others:      none
**********************************************************/
void BMS56M605::readRawGyroscope(int &rawGyroX, int &rawGyroY, int &rawGyroZ)
{
      readReg(REG_GYRO_XOUT_H,dataBuff,6);
      rawGyroX = dataBuff[0]*256 + dataBuff[1];
      rawGyroY = dataBuff[2]*256 + dataBuff[3];
      rawGyroZ = dataBuff[4]*256 + dataBuff[5];
}




/**********************************************************
Description: clearBuf()
Parameters:  clear Buff
Return:   void
Others:
**********************************************************/
void BMS56M605::clearBuf()
{
  for(int a = 0; a < 10; a++)
  {
    dataBuff[a] = 0;
  }
}
/**********************************************************
Description: writeBytes
Parameters:  wbuf[]:Variables for storing Data to be sent
             wlen:Length of data sent  
Return:   void
Others:
**********************************************************/
void BMS56M605::writeBytes(uint8_t wbuf[], uint8_t wlen)
{
    while(_wire->available() > 0)
    {
      _wire->read();
    }
    _wire->beginTransmission(_i2caddr); //IIC start with 7bit addr
    _wire->write(wbuf, wlen);
    _wire->endTransmission();
}


/**********************************************************
Description: write a bit data
Parameters:  bitNum :Number of bits(bit7-bit0)
             bitValue :Value written 
Return:      void  
Others:      none
**********************************************************/
void BMS56M605::writeRegBit(uint8_t addr,uint8_t bitNum, uint8_t bitValue)
{
      uint8_t data;
      readReg(addr,dataBuff,1);
      data = dataBuff[0];
      data = (bitValue != 0)? (data|(1<<bitNum)) : (data & ~(1 << bitNum));
      writeReg(addr, data);
}
/**********************************************************
Description: readBytes
Parameters:  rbuf[]:Variables for storing Data to be obtained
             rlen:Length of data to be obtained
Return:   void
Others:
**********************************************************/
void BMS56M605::readBytes(uint8_t rbuf[], uint8_t rlen)
{
    _wire->requestFrom(_i2caddr, rlen);
    if(_wire->available()==rlen)
    {
      for(uint8_t i = 0; i < rlen; i++)
      {
         rbuf[i] = _wire->read();
      }
    }
}
