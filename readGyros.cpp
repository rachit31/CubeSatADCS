/** 
 *  @file    readGyro.cpp
 *  @author  Rachit Bhatia
 *  @date    05/01/2018
 *  @version 1.0.0
 *  
 *  @brief A thread to read Gyro data
 *
 */
 
 #include "thread.hpp"
 #include "KalmanPi.hpp"
 #include "sleepTimer.hpp"
 #include <cstring>
 #include <iostream>
 
 void* readGyros(void *pArg)
 {
	 // Push Cleanup Handler to cleanup thread
	 pthread_cleanup_push(&Thread::cleanup,pArg);
	 
	 // Code goes here...
	
	 const double pi = 3.141592653589793;
	 const double gyroToW = 500.0/32767.0*pi/180.0; // radians/second/count
	 /* const double gyroToW = 500/(pow(2,15)-1)*pi/180.0; // radians/second/count */
	 
	 uint8_t buffer[6];
	 int16_t data;
	 
	 double wbi_B[3];
	 
	 SleepTimer myTimer(1.0/119.0);
	 
	 myTimer.initialize();
	 
	 while (true)
	{
		i2c.readBuffer(0x6b, 0x18, buffer, 6);
		
		data = (buffer[1] << 8) | buffer[0]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		wbi_B[0] = -1.0*gyroToW*data; // converting to engineering units (radians/second)
		
		data = (buffer[3] << 8) | buffer[2]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		wbi_B[1] = gyroToW*data; // converting to engineering units (radians/second)
		
		data = (buffer[5] << 8) | buffer[4]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		wbi_B[2] = gyroToW*data; // converting to engineering units (radians/second)
		
		gyroMutex.lock();
		
		memcpy(gyroBuffer, wbi_B, sizeof(wbi_B));
		
		gyroMutex.unlock();
		
		gyroMutex.signal();
		
		myTimer.sleep();
		
		// std::cout << wbi_B[0] << ", " << wbi_B[1] << ", " << wbi_B[2] << std::endl;
		
		/*
		wbi_B[0] = -1.0*gyroToW*data - Bw_B[0]; // converting to engineering units (radians/second)
		
		data = (buffer[3] << 8) | buffer[2]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		wbi_B[1] = gyroToW*data - Bw_B[1]; // converting to engineering units (radians/second)
		
		data = (buffer[5] << 8) | buffer[4]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		wbi_B[2] = gyroToW*data - Bw_B[2]; // converting to engineering units (radians/second)
		*/
	}
	
     // Run clean up handler to cleanup thread (TLPI pg 676)
     pthread_cleanup_pop(1);
 }