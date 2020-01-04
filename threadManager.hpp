/** 
 *  @file    threadManager.hpp
 *  @author  Rachit Bhatia
 *  @date    02/23/2018
 *  @version 1.0.3
 *  
 *  @brief Used to manager the thread class
 *
 *  @section DESCRIPTION
 *  
 *  This class is used to create and manage instances of the thread class
 *
 */

#ifndef THREADMANAGER_HPP_
#define THREADMANAGER_HPP_

#include <pthread.h>
#include <map>
#include "thread.hpp"
#include "mutex.hpp"

class Thread;  ///< Forward declaration since each class contains the other

/**
*  @brief Class that creates a pthread and maintains members related to it
*/
class ThreadManager
{
public:

  /** 
  *   @brief  Constructor which sets the number of unjoined threads to 0
  *
  */
  ThreadManager();

  /** 
  *   @brief  Frees all the memory allocated for created threads
  */
  ~ThreadManager();

  /** 
  *   @brief  Creates a thread which starts the function.  A thread id is assigned
  *  
  *   @param  function is the the name of the function to start 
  *   @return const char* returns the thread id on success or error message on failure
  */
  const char* create(void *(*function)(void *));

  /** 
  *   @brief  Creates a thread with a specific id that starts the function.
  *  
  *   @param  function is the the name of the function to start
  *   @param  id is the desired thread id
  *   @return const char* returns error message on failure
  */
  const char* create(void *(*function)(void *),
                     const unsigned int id);

  /** 
  *   @brief  Used the cancel a previously created thread
  *  
  *   @param  id is the id of the thread to cancel
  *   @return const char* returns error message on failure
  */
  const char* cancel(const unsigned int id);

  /** 
  *   @brief  Waits for all threads to finish, and then cleansup
  *  
  *   @return const char* return error message on failure or accumulated status on success
  */
  const char* wait();

  /** 
  *   @brief  Called by a dead thread to tell the manager how many have died
  *  
  *   @return void
  */
  void incrementJoin();

  /** 
  *   @brief  Returns the thread manager mutex
  *  
  *   @return Mutex* pointer to mutex held by this class
  */
  Mutex* getMutex();

private:

  /** 
  *   @brief  Internal function called by both create functions
  *  
  *   @param  function is the function to start in the thread
  *   @param  id will be the thread id
  *   @return const char* error message on failure
  */
  const char* threadCreate(void *(*function)(void *), const unsigned int id);

  int m_numUnjoined;                           ///< number of unjoined thread
  std::map<unsigned int, Thread*> m_threadMap; ///< map of currently created threads
  Mutex m_mutex;                               ///< mutex used to synchronous across threads
};

#endif