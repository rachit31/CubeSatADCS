/** 
 *  @file    i2c.hpp
 *  @author  Rachit Bhatia
 *  @date    02/23/2018
 *  @version 1.0.3
 *  
 *  @brief Declares the I2C Class
 *
 *  @section DESCRIPTION
 *  
 *  This class is a wrapper for using i2c on linux
 *
 */
#ifndef I2C_HPP_
#define I2C_HPP_

#include <stdint.h>
#include "mutex.hpp"

/**
*  @brief Class that sets up a clock_nanosleep
*/
class I2C
{
public:

  /** 
  *   @brief  Constructor
  *
  */
  I2C();

  /** 
  *   @brief  Destructor
  *
  */
  ~I2C();

  /** 
  *   @brief  Try to open I2C bus
  *
  */
  const char* initialize();

  /** 
  *   @brief  Write a byte to a specific address
  *
  */
  const char* writeByte(uint8_t busAddr, uint8_t dataAddr, uint8_t data);

  /** 
  *   @brief  Read a byte from a specific address
  *
  */
  uint8_t readByte(uint8_t busAddr, uint8_t dataAddr);

  /** 
  *   @brief  Read a buffer of data from I2C bus
  *
  */
  const char* readBuffer(uint8_t busAddr, uint8_t dataAddr, uint8_t* buffer, int number);

private:
  int m_busFd;
  Mutex m_mutex;
};

#endif