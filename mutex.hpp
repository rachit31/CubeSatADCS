/** 
 *  @file    mutex.hpp
 *  @author  Rachit Bhatia
 *  @date    02/23/2018
 *  @version 1.0.3
 *  
 *  @brief Declares the Mutex Class
 *
 *  @section DESCRIPTION
 *  
 *  This class is a wrapper for Linux pthread_mutexs.  It creates a mutex that
 *  will be locked when object is destroyed.
 *
 */

#ifndef MUTEX_HPP_
#define MUTEX_HPP_

#include <pthread.h>

/**
*  @brief Class that creates a mutex and perform actions with the mutex
*/
class Mutex
{
public:

  /** 
  *   @brief  Constructor which sets mutex lock status to unlocked
  *
  */
  Mutex();

  /** 
  *   @brief  Destructor which unlocks the mutex if locked
  *
  */
  ~Mutex();

  /** 
  *   @brief  Check if mutex is locked
  *  
  *   @return bool is true if locked
  */
  const bool isLocked() const;

  /** 
  *   @brief  Locks the mutex
  *  
  *   @return const char* return error message on failure
  */
  const char* lock();

  /** 
  *   @brief  Unlocks the mutex
  *  
  *   @return const char* return error message on failure
  */
  const char* unlock();

  /** 
  *   @brief  Waits until the conditional variable is signaled
  *  
  *   @return const char* return error message on failure
  */
  const char* wait();

  /** 
  *   @brief  Signals the conditional variable
  *  
  *   @return const char* return error message on failure
  */
  const char* signal();

private:
  bool m_locked;                                               ///< lock status
  pthread_mutex_t m_pthread_mutex = PTHREAD_MUTEX_INITIALIZER; ///< mutex
  pthread_cond_t m_pthread_cond = PTHREAD_COND_INITIALIZER;    ///< conditional variable
};

#endif