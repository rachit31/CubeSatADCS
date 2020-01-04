/** 
 *  @file    mutex.cpp
 *  @author  Rachit Bhatia
 *  @date    02/23/2018
 *  @version 1.0.3
 *  
 *  @brief Initializes a mutex that can locked and unlocked
 *
 */

#include "mutex.hpp"
#include <sstream>
#include <cstring>

Mutex::Mutex() :
m_locked(false)
{
}

Mutex::~Mutex()
{
  if (m_locked)
  {
    // Lock the thread Mutex (TLPI pg 636)
    pthread_mutex_unlock(&m_pthread_mutex);
  }
}

const bool Mutex::isLocked() const
{
  return m_locked;
}

const char* Mutex::lock()
{
  std::stringstream error;
  int retVal;

  // Lock the thread Mutex (TLPI pg 636)
  retVal = pthread_mutex_lock(&m_pthread_mutex);
  if (retVal != 0)
  {
    error << "pthread_mutex_lock(): " << std::strerror(retVal);
    return error.str().c_str();
  }
  m_locked = true;

  return error.str().c_str();
}

const char* Mutex::unlock()
{
  std::stringstream error;
  int retVal;

  // Unlock the thread Mutex (TLPI pg 636)
  retVal = pthread_mutex_unlock(&m_pthread_mutex);
  if (retVal != 0)
  {
    error << "pthread_mutex_unlock(): " << std::strerror(retVal);
    return error.str().c_str();
  }
  m_locked = false;

  return error.str().c_str();
}

const char* Mutex::wait()
{
  std::stringstream error;
  int retVal;

  // Lock the thread Mutex (TLPI pg 636)
  retVal = pthread_mutex_lock(&m_pthread_mutex);
  if (retVal != 0)
  {
    error << "pthread_mutex_lock(): " << std::strerror(retVal);
    return error.str().c_str();
  }
  m_locked = true;

  // If the cond_wait is successful the mutex will be unlocked
  m_locked = false;

  // Wait for Thread Conditional Variable (TLPI pg 644)
  retVal = pthread_cond_wait(&m_pthread_cond, &m_pthread_mutex);
  if (retVal != 0)
  {
    m_locked = true;
    error << "pthread_cond_wait(): " << std::strerror(retVal);
    return error.str().c_str();
  }

  // The mutex is now relocked
  m_locked = true;

  return error.str().c_str();
}

const char* Mutex::signal()
{
  std::stringstream error;
  int retVal;

  // Signal the conditional variable (TLPI pg 644)
  retVal = pthread_cond_signal(&m_pthread_cond);
  if (retVal != 0)
  {
    error << "pthread_cond_signal(): " << std::strerror(retVal);
    return error.str().c_str();
  }
  return error.str().c_str();
}