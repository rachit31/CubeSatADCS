/** 
 *  @file    sendFunction.cpp
 *  @author  Rachit Bhatia
 *  @date    03/16/2018
 *  @version 1.0.0
 *  
 *  @brief Send data thread to call sendUDP.cpp function and copy the data safely in sendBuffer 
 *
 */

#include "thread.hpp"
#include "sendUdp.hpp"
#include "KalmanPi.hpp"
#include <cstring>

void* sendFunction(void *pArg)
{
  // Push Cleanup Handler to cleanup thread
  pthread_cleanup_push(&Thread::cleanup,pArg);

  // Create a SendUdp Class
  SendUdp sendUdp;
  
  // Initialize the SendUdp Class
  sendUdp.initialize(LOCALIP, REMOTEIP, SNDPORT);

  // Create a buffer to dump the data in  
  double localBuffer[SNDNUM];
  
  // Loop and send data
  while(true)  
	  {
		  sendMutex.wait(); // Wait until gets access to mutex
	  
          std::memcpy(localBuffer, sendBuffer, sizeof(localBuffer)); // (Copy to, Copy from, Copy Number of Bytes)
	  
          sendMutex.unlock(); // Now anyone can access send buffer
	  
          sendUdp.sendDoubles(localBuffer, SNDNUM); // Send the specified number of doubles from the localBuffer to Simulink
	  }

  // Run clean up handler to cleanup thread (TLPI pg 676)
  pthread_cleanup_pop(1);
}