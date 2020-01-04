/** 
 *  @file    threadManager.cpp
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

#include "threadManager.hpp"
#include <sstream>
#include <cstring>
#include <vector>

ThreadManager::ThreadManager() : 
m_numUnjoined(0)
{
}

ThreadManager::~ThreadManager()
{
  // Iterate over map and free memory
  std::map<unsigned int, Thread*>::iterator it = m_threadMap.begin();
  for (it = m_threadMap.begin(); it != m_threadMap.end(); ++it)
  {
    delete it->second;
  }
}

const char* ThreadManager::create(void *(*function)(void *))
{
  // Look for lowest available thread id
  unsigned int id = 0;
  bool found = true;
  while(found)
  {
    if (m_threadMap.count(id))
    {
      id++;
    }
    else
    {
      found = false;
    }
  }

  std::string error = threadCreate(function,id);
  if (!error.empty())
  {
    return error.c_str();
  }

  std::stringstream idStream;
  idStream << id;

  return idStream.str().c_str();
}

const char* ThreadManager::create(void *(*function)(void *), const unsigned int id)
{
  std::stringstream error;

  // Look for existing id
  if (m_threadMap.count(id))
  {
    error << "thread " << id << " already exists";
    return error.str().c_str();
  }

  return threadCreate(function,id);
}

const char* ThreadManager::threadCreate(void *(*function)(void *), const unsigned int id)
{
  std::stringstream error;

  Thread* pThread;

  try
  {
    pThread = new Thread(function, this, id);
  }
  catch (const char* e)
  {
    error << "Thread(): " << e;
    return error.str().c_str();
  }

  m_threadMap[id] = pThread;

  return error.str().c_str();
}

// Compare Listing 30-4 in TLPI
const char* ThreadManager::wait()
{
  std::stringstream message;

  // Join with terminated threads
  while (!m_threadMap.empty())
  {
    std::string error;

    // Lock the thread Mutex (TLPI pg 636)
    error = m_mutex.lock();
    if (!error.empty())
    {
      return error.c_str();
    }

    // Set the Conditional Variable to wait for a thread to die
    if (m_numUnjoined == 0)
    {
      error = m_mutex.wait();
      if (!error.empty())
      {
        return error.c_str();
      }
    }

    // Loop through the threads to find the dead ones
    std::vector<unsigned int> eraseVec;
    std::map<unsigned int, Thread*>::iterator it;
    for (it = m_threadMap.begin(); it != m_threadMap.end(); ++it)
    {
      if (!it->second->isAlive())
      {
        void* pthreadRetVal;

        // Join with terminated thread (TLPI pg 625)
        int retVal = pthread_join(it->second->getThread(), &pthreadRetVal);
        if (retVal != 0)
        {
          message << "pthread_join(): " << std::strerror(retVal);
          return message.str().c_str();
        }

        if (pthreadRetVal == PTHREAD_CANCELED)
        {
          message << "thread " << it->first << " was canceled" << std::endl;
        }
        else
        {
          message << "thread " << it->first << " joined successfully" << std::endl;
        }

        // Release the dynamically allocated memory
        delete it->second;

        // Add the map key to the delete vector
        eraseVec.push_back(it->first);

        // Decrement the number of unjoined threads
        m_numUnjoined--;
      }
    }

    std::vector<unsigned int>::iterator iter;
    for (iter = eraseVec.begin(); iter != eraseVec.end(); ++iter)
    {
      if (!m_threadMap.erase(*iter))
      {
        error = "could not remove thread from threadManager threadMap";
        return error.c_str();
      }
    }

    // Lock the thread Mutex (TLPI pg 636)
    error = m_mutex.unlock();
    if (!error.empty())
    {
      return error.c_str();
    }
  }

  return message.str().c_str();
}

void ThreadManager::incrementJoin()
{
  m_numUnjoined++;
}

Mutex* ThreadManager::getMutex()
{
  return &m_mutex;
}

const char* ThreadManager::cancel(const unsigned int id)
{
  std::stringstream error;
  int retVal;

  if (!m_threadMap.count(id))
  {
    error << "thread id " << id << " does not exist";
    return error.str().c_str();
  }

  // Cancel the thread (TLPI pg 671)
  retVal = pthread_cancel(m_threadMap[id]->getThread());
  if (retVal != 0)
  {
    error << "pthread_cancel(): " << std::strerror(retVal);
    return error.str().c_str();
  }

  return error.str().c_str();
}