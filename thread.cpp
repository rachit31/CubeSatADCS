/** 
 *  @file    thread.cpp
 *  @author  Rachit Bhatia
 *  @date    02/23/2018
 *  @version 1.0.3
 *  
 *  @brief Creates a pthread and cleans up when done
 *
 *  @section DESCRIPTION
 *  
 *  This class is a wrapper for Linux pthreads and is designed to be used by
 *  the ThreadManager class.
 *
 */

#include "thread.hpp"
#include <sstream>
#include <cstring>

Thread::Thread(void *(*function)(void *),
               ThreadManager* pManager,
               const unsigned int id) :
m_alive(true), m_pManager(pManager), m_id(id)
{
  int retVal;

  // Create pthread (TLPI pg 622)
  retVal = pthread_create(&m_pthread, NULL, function, this);
  if (retVal != 0)
  {
    std::stringstream error;
    error << "pthread_create(): " << std::strerror(retVal);
    throw error.str().c_str();
  }
}

Thread::~Thread()
{
}

const bool Thread::isAlive() const
{
  return m_alive;
}

const pthread_t Thread::getThread() const
{
 return m_pthread;
}

const unsigned int Thread::getId() const
{
 return m_id;
}

void Thread::threadCleanup()
{
  // Get the mutex from the threadManager
  Mutex* pMutex = m_pManager->getMutex();

  // Lock the mutex
  pMutex->lock();

  // Mark this thread as dead
  m_alive = false;

  // Increment the managers join variable
  m_pManager->incrementJoin();

  // Unlock the mutex
  pMutex->unlock();

  // Signal the thread conditional variable
  pMutex->signal();
}

// Static method to be called by cleanup handler which invokes actually object method
void Thread::cleanup(void* pArg)
{
  static_cast<Thread*>(pArg)->threadCleanup();
}