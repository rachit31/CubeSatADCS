/** 
 *  @file    KalmanPi.cpp
 *  @author  Rachit Bhatia
 *  @date    02/23/2018
 *  @version 1.0.3
 */
 #include "KalmanPi.hpp"
 #include "threadManager.hpp"
 #include <iostream>
 #include <cmath>
 #include "sleepTimer.hpp"

// Instantiate Global Variables 
 double gyroBuffer[3];
 Mutex gyroMutex;
 
 double accelBuffer[3];
 Mutex accelMutex;
 
 double magBuffer[3];
 Mutex magMutex;
 
 double sendBuffer[SNDNUM];
 Mutex sendMutex;
 
 double Bw_B[3] = {0, 0, 0};
 
 double a_I[3] = {0, 0, 0};
 
 double B_I[3] = {0, 0, 0};
 
 I2C i2c;
 
 const double dtGyro = 1.0/119.0;
 
 int main(int argc, char* argv[])
 {
	 i2c.initialize();
	 
	 // Initialize Gyro
	 initializeGyros();
	 
	 // Initialize Accelerometer
	 initializeAccel();
	 
	 // Initialize Magnetometer
	 initializeMag();
	 
	 ThreadManager threadManager;
	 threadManager.create(readGyros);
	 threadManager.create(readAccel);
	 threadManager.create(readMag);
	 threadManager.create(sendFunction);
	 threadManager.create(propagateANDupdate);
	 
	 threadManager.wait();
	 
	 return EXIT_SUCCESS;
 }
	 
void initializeGyros()
{
     // 119 Hz Sampling, 500 Degrees/second, 31 Hz LPF
	 i2c.writeByte(0x6b, 0x10, 0b01101011);
	 
	 const double pi = 3.141592653589793;
	 const double gyroToW = 500.0/32767.0*pi/180.0; // radians/second/count
	 /* const double gyroToW = 500/(pow(2,15)-1)*pi/180.0; // radians/second/count */
	 
	 uint8_t buffer[6];
	 int16_t data;
	 
	 SleepTimer myTimer(dtGyro);
	 
	 std::cout << "Hold still to initialize Gyro bias" << std::endl;
	
	 myTimer.initialize();
	
	 for (int i = 0; i < 2*119; i++)
	 {
		 i2c.readBuffer(0x6b, 0x18, buffer, 6);
		
		 data = (buffer[1] << 8) | buffer[0]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		 Bw_B[0] += -1.0*gyroToW*data; // computing initial bias, converting to engineering units (radians/second)
		
		 data = (buffer[3] << 8) | buffer[2]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		 Bw_B[1] += gyroToW*data; // computing initial bias, converting to engineering units (radians/second)
		
		 data = (buffer[5] << 8) | buffer[4]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		 Bw_B[2] += gyroToW*data; // computing initial bias, converting to engineering units (radians/second)
		
		 myTimer.sleep();
	 }
	
	 Bw_B[0] = Bw_B[0]/(2.0*119.0);
	 Bw_B[1] = Bw_B[1]/(2.0*119.0);
	 Bw_B[2] = Bw_B[2]/(2.0*119.0);
	
	 std::cout << "Bw_B[0]: " << Bw_B[0] << std::endl;
	 std::cout << "Bw_B[1]: " << Bw_B[1] << std::endl;
	 std::cout << "Bw_B[2]: " << Bw_B[2] << std::endl;
	 
	 std::cout << "Gyro bias initialized" << std::endl;
}
	 
void initializeAccel()
{	 
	 // 10 Hz Sampling, 2 gs
	 i2c.writeByte(0x6b, 0x20, 0b001000000);
	 
	 const double toAccel = 2.0/32767.0; // gs/count
	 /* const double toAccel = 2.0/(pow(2,15)-1); // gs/count */
	 
	 uint8_t buffer[6];
	 int16_t data;
	 
	 SleepTimer myTimer(0.1);
	 
	 std::cout << "Hold still to initialize Accelerometer" << std::endl;
	
	 myTimer.initialize();
	
	 for (int i = 0; i < 20; i++)
	 {
		 i2c.readBuffer(0x6b, 0x28, buffer, 6);
		
		 data = (buffer[1] << 8) | buffer[0]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		 a_I[0] += -1.0*toAccel*data; // converting to engineering units (g)
		
		 data = (buffer[3] << 8) | buffer[2]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		 a_I[1] += toAccel*data; // converting to engineering units (g)
		
		 data = (buffer[5] << 8) | buffer[4]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		 a_I[2] += toAccel*data; // converting to engineering units (g)
		
		 myTimer.sleep();
	 }
	
	 a_I[0] /= 20.0;
	 a_I[1] /= 20.0;
	 a_I[2] /= 20.0;
	
	 std::cout << "a_I[0]: " << a_I[0] << std::endl;
	 std::cout << "a_I[1]: " << a_I[1] << std::endl;
	 std::cout << "a_I[2]: " << a_I[2] << std::endl;
	 
	 std::cout << "Accelerometer initialized" << std::endl;
}	 

void initializeMag()
{
	 // TEMP_COMP Enabled, high Performance, 1.25 Hz Sampling
	 i2c.writeByte(0x1E, 0x20, 0b11100100);
	 
	 // Continuous COnversion Mode
	 i2c.writeByte(0x1E, 0x22, 0b00000000);
	 
	 const double toMag = 4.0/32767.0; // gauss/count
	 /* const double toMag = 4.0/(pow(2,15)-1); // gauss/count */
	 
	 uint8_t buffer[6];
	 int16_t data;
	 
	 SleepTimer myTimer(1.0/1.25);
	 
	 std::cout << "Hold still to initialize Magnetometer" << std::endl;
	
	 myTimer.initialize();
	
	 for (int i = 0; i < 10; i++)
	 {
		 i2c.readBuffer(0x1E, 0x28, buffer, 6);
		
		 data = (buffer[1] << 8) | buffer[0]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		 B_I[0] += 1.0*toMag*data; // converting to engineering units (gauss)
		
		 data = (buffer[3] << 8) | buffer[2]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		 B_I[1] += toMag*data; // converting to engineering units (gauss)
		
		 data = (buffer[5] << 8) | buffer[4]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		 B_I[2] += toMag*data; // converting to engineering units (gauss)
		
		 myTimer.sleep();
	 }
	
	 B_I[0] /= 10.0;
	 B_I[1] /= 10.0;
	 B_I[2] /= 10.0;
	
	 std::cout << "B_I[0]: " << B_I[0] << std::endl;
	 std::cout << "B_I[1]: " << B_I[1] << std::endl;
	 std::cout << "B_I[2]: " << B_I[2] << std::endl;
	 
	 std::cout << "Magnetometer initialized" << std::endl;
}	 
	 