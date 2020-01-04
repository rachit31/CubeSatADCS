/** 
 *  @file    readMag.cpp
 *  @author  Rachit Bhatia
 *  @date    05/01/2018
 *  @version 1.0.0
 *  
 *  @brief A thread to read Magnetometer data
 *
 */
 
 #include "thread.hpp"
 #include "KalmanPi.hpp"
 #include "sleepTimer.hpp"
 #include <cstring>
 #include <iostream>
 
 void* readMag(void *pArg)
 {
	 // Push Cleanup Handler to cleanup thread
	 pthread_cleanup_push(&Thread::cleanup,pArg);
	 
	 // Code goes here...
	
	 uint8_t buffer[6];
	 int16_t data;
	 
	 const double toMag = 4.0/32767.0; // gauss/count
	 /* const double toMag = 4.0/(pow(2,15)-1); // gauss/count */
	 
	 double B_B[3];
	 
	 SleepTimer myTimer(1.0/1.25);
	 
	 myTimer.initialize();
	 
	 while (true)
	{
		i2c.readBuffer(0x1E, 0x28, buffer, 6);
		
		data = (buffer[1] << 8) | buffer[0]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		B_B[0] = 1.0*toMag*data; // converting to engineering units (gauss)
		
		data = (buffer[3] << 8) | buffer[2]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		B_B[1] = toMag*data; // converting to engineering units (gauss)
		
		data = (buffer[5] << 8) | buffer[4]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		B_B[2] = toMag*data; // converting to engineering units (gauss)
		
		magMutex.lock();
		
		memcpy(magBuffer, B_B, sizeof(B_B));
		
		magMutex.unlock();
		
		// std::cout << B_B[0] << ", " << B_B[1] << ", " << B_B[2] << std::endl;
		
		myTimer.sleep();
		
		/*
		magMutex.lock();
		
		memcpy(magBuffer, B_B, sizeof(B_B));
		
		magMutex.unlock();
		
		magMutex.signal();
		*/
		
		/*
		B_B[0] = -1.0*toMag*data - Mag_Bw_B[0]; // converting to engineering units (gauss)
		
		data = (buffer[3] << 8) | buffer[2]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		B_B[1] = toMag*data - Mag_Bw_B[1]; // converting to engineering units (gauss)
		
		data = (buffer[5] << 8) | buffer[4]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		B_B[2] = toMag*data - Mag_Bw_B[2]; // converting to engineering units (gauss)
		*/
	}
	
     // Run clean up handler to cleanup thread (TLPI pg 676)
     pthread_cleanup_pop(1);
 }