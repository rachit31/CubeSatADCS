/** 
 *  @file    thread.hpp
 *  @author  Rachit Bhatia
 *  @date    02/23/2018
 *  @version 1.0.3
 *  
 *  @brief Declares the Thread Class
 *
 *  @section DESCRIPTION
 *  
 *  This class is a wrapper for Linux pthreads and is designed to be used by
 *  the ThreadManager class.
 *
 */

#ifndef THREAD_HPP_
#define THREAD_HPP_

#include <pthread.h>
#include "threadManager.hpp"

class ThreadManager; ///< Forward declaration since each class contains the other

/**
*  @brief Class that creates a pthread and maintains members related to it
*/
class Thread
{
public:

  /** 
  *   @brief  Constructor
  *  
  *   @param  function is the thread function to start 
  *   @param  pManaager is the pointer to threadManager in charge of thread
  *   @param  id is a simple id used to identify the thread
  */
  Thread(void *(*function)(void *),
         ThreadManager* pManager,
         const unsigned int id);

  /** 
  *   @brief  Destructor does nothing
  */
  ~Thread();

  /** 
  *   @brief  Check if thread is alive
  *  
  *   @return boolean that is true when alive
  */
  const bool isAlive() const;

  /** 
  *   @brief  Return the underlying pthread_t of the thread
  *  
  *   @return pthread_t of thread
  */
  const pthread_t getThread() const;

  /** 
  *   @brief  Return the thread id
  *  
  *   @return the id of the thread
  */
  const unsigned int getId() const;

  /** 
  *   @brief  A function that can be called by a pthread cleanup handler
  *  
  *   @param  pArg is a pointer to the thread class to cleanup 
  *   @return void
  */
  static void cleanup(void* pArg);

private:

  /** 
  *   @brief  Actual cleanup thread that is called by static cleanup function 
  *  
  *   @return void
  */
  void threadCleanup();

  const unsigned int m_id;   ///< thread id
  pthread_t m_pthread;       ///< pthread_t of underlying thread
  bool m_alive;              ///< true when alive
  ThreadManager* m_pManager; ///< pointer to threadManager controlling thread
};

#endif