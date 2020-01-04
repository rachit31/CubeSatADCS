/** 
 *  @file    SleepTimer.cpp
 *  @author  Rachit Bhatia
 *  @date    02/23/2018
 *  @version 1.0.3
 *  
 *  @brief Sets up a clock_nanosleep function
 *
 */

#include "sleepTimer.hpp"

SleepTimer::SleepTimer(double duration)
{
  m_durSec = duration;
  m_durNan = (duration - m_durSec)*1000000000;
}

void SleepTimer::initialize()
{
  clock_gettime(CLOCK_MONOTONIC, &m_wakeTime);
}

void SleepTimer::sleep()
{
  m_wakeTime.tv_sec += m_durSec;
  m_wakeTime.tv_nsec += m_durNan;

  if (m_wakeTime.tv_nsec >= 1000000000)
  {
    m_wakeTime.tv_sec++;
    m_wakeTime.tv_nsec -= 1000000000;
  }

  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &m_wakeTime, NULL);
}

double SleepTimer::getCurrentTime()
{
  struct timespec currentTime;
  clock_gettime(CLOCK_MONOTONIC, &currentTime);

  double time = (1.0*currentTime.tv_sec) + (1.0*currentTime.tv_nsec)/1000000000;

  return time;
}
