/** 
 *  @file    readAccel.cpp
 *  @author  Rachit Bhatia
 *  @date    05/01/2018
 *  @version 1.0.0
 *  
 *  @brief A thread to read Accelerometer data
 *
 */
 
 #include "thread.hpp"
 #include "KalmanPi.hpp"
 #include "sleepTimer.hpp"
 #include <cstring>
 #include <iostream>
 
 void* readAccel(void *pArg)
 {
	 // Push Cleanup Handler to cleanup thread
	 pthread_cleanup_push(&Thread::cleanup,pArg);
	 
	 // Code goes here...
	
	 uint8_t buffer[6];
	 int16_t data;
	 
	 const double toAccel = 2.0/32767.0; // gs/count
	 /* const double toAccel = 2.0/(pow(2,15)-1); // gs/count */
	 
	 double a_B[3];
	 
	 SleepTimer myTimer(0.1);
	 
	 myTimer.initialize();
	 
	 while (true)
	{
		i2c.readBuffer(0x6b, 0x28, buffer, 6);
		
		data = (buffer[1] << 8) | buffer[0]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		a_B[0] = -1.0*toAccel*data; // converting to engineering units (g)
		
		data = (buffer[3] << 8) | buffer[2]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		a_B[1] = toAccel*data; // converting to engineering units (g)
		
		data = (buffer[5] << 8) | buffer[4]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		a_B[2] = toAccel*data; // converting to engineering units (g)
		
		accelMutex.lock();
		
		memcpy(accelBuffer, a_B, sizeof(a_B));
		
		accelMutex.unlock();
		
		// std::cout << a_B[0] << ", " << a_B[1] << ", " << a_B[2] << std::endl;
		
		myTimer.sleep();
		
		/*
		accelMutex.lock();
		
		memcpy(accelBuffer, a_B, sizeof(a_B));
		
		accelMutex.unlock();
		
		accelMutex.signal();
		*/
		
		/*
		a_B[0] = -1.0*toAccel*data - Accel_Bw_B[0]; // converting to engineering units (g)
		
		data = (buffer[3] << 8) | buffer[2]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		a_B[1] = toAccel*data - Accel_Bw_B[1]; // converting to engineering units (g)
		
		data = (buffer[5] << 8) | buffer[4]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		a_B[2] = toAccel*data - Accel_Bw_B[2]; // converting to engineering units (g)
		*/
	}
	
     // Run clean up handler to cleanup thread (TLPI pg 676)
     pthread_cleanup_pop(1);
 }