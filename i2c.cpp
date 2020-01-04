/** 
 *  @file    i2c.cpp
 *  @author  Rachit Bhatia
 *  @date    02/23/2018
 *  @version 1.0.3
 *  
 *  @brief This class is a wrapper for using i2c on linux
 *
 */

#include "i2c.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

I2C::I2C()
{
}

I2C::~I2C()
{
  close(m_busFd);
}

const char* I2C::initialize()
{
  std::stringstream error;

  // Open the I2C Bus
  m_busFd = open("/dev/i2c-1", O_RDWR);
  if (m_busFd == -1)
  {
    error << "open(): " << std::strerror(errno);
    return error.str().c_str();
  }

  return NULL;
}

const char* I2C::writeByte(uint8_t busAddr, uint8_t dataAddr, uint8_t data)
{
  std::stringstream error;

  union i2c_smbus_data smbus_data;
  smbus_data.byte = data;

  struct i2c_smbus_ioctl_data args;

  args.read_write = 0;
  args.command = dataAddr;
  args.size = 2;
  args.data = &smbus_data;

  m_mutex.lock();
  if (ioctl(m_busFd, I2C_SLAVE, busAddr) == -1)
  {
    m_mutex.unlock();
    error << "ioctl(): " << std::strerror(errno);
    return error.str().c_str();
  }

  if (ioctl(m_busFd,I2C_SMBUS,&args) == -1)
  {
    m_mutex.unlock();
    error << "ioctl(): " << std::strerror(errno);
    return error.str().c_str();
  }
  m_mutex.unlock();

  return NULL;
}

uint8_t I2C::readByte(uint8_t busAddr, uint8_t dataAddr)
{
  std::stringstream error;

  union i2c_smbus_data smbus_data;

  struct i2c_smbus_ioctl_data args;

  args.read_write = 1;
  args.command = dataAddr;
  args.size = 2;
  args.data = &smbus_data;

  m_mutex.lock();
  if (ioctl(m_busFd, I2C_SLAVE, busAddr) == -1)
  {
    m_mutex.unlock();
    return -1;
  }

  if (ioctl(m_busFd,I2C_SMBUS,&args) == -1)
  {
    m_mutex.unlock();
    return -1;
  }
  m_mutex.unlock();

  return 0xFF & smbus_data.byte;
}

const char* I2C::readBuffer(uint8_t busAddr, uint8_t dataAddr, uint8_t* buffer, int number)
{
  std::stringstream error;
  int retVal;

  struct i2c_smbus_ioctl_data args;

  args.read_write = 0;
  args.command = dataAddr;
  args.size = 1;
  args.data = NULL;

  m_mutex.lock();
  if (ioctl(m_busFd, I2C_SLAVE, busAddr) == -1)
  {
    m_mutex.unlock();
    error << "ioctl(): " << std::strerror(errno);
    return error.str().c_str();
  }

  if (ioctl(m_busFd,I2C_SMBUS,&args) == -1)
  {
    m_mutex.unlock();
    error << "ioctl(): " << std::strerror(errno);
    return error.str().c_str();
  }

  retVal = read(m_busFd, buffer, number);
  m_mutex.unlock();

  if (retVal != number)
  {
    if (retVal == -1)
    { 
      error << "ioctl(): " << std::strerror(errno);
      return error.str().c_str();
    }
    else
    {
      error << "read() did not read " << number << " bytes";
      return error.str().c_str();
    }
  }

  return NULL;
}



