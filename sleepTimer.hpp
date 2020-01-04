/** 
 *  @file    sleepTimer.hpp
 *  @author  Rachit Bhatia
 *  @date    02/23/2018
 *  @version 1.0.3
 *  
 *  @brief Declares the SleepTimer Class
 *
 *  @section DESCRIPTION
 *  
 *  This class is a wrapper for the Linux clock_nanosleep function
 *
 */

#ifndef SLEEPTIMER_HPP_
#define SLEEPTIMER_HPP_

#include <time.h>

/**
*  @brief Class that sets up a clock_nanosleep
*/
class SleepTimer
{
public:

  /** 
  *   @brief  Constructor pass in sleep duration
  *
  */
  SleepTimer(double duration);

  /** 
  *   @brief  Increment timer and sleep until it expires
  *
  */
  void sleep();

  /** 
  *   @brief  Set initial timer to expire now
  *
  */
  void initialize();

  /** 
  *   @brief  Get the current time
  *
  */
  static double getCurrentTime();

private:
  struct timespec m_wakeTime;
  unsigned long m_durSec;
  unsigned long m_durNan;
};

#endif