#include <iostream>
#include <bitset>
#include "i2c.hpp"
#include "sleepTimer.hpp"
#include <cmath>
#include "sendUdp.hpp"

int main(int argc, char* argv[])
{
	I2C i2c;
	
	i2c.initialize();
	
	SleepTimer myTimer(1.0/119.0);
	
	std::cout.precision(16);
	
	// 119 Hz Sampling, 500 Degrees/second, 31 Hz LPF
	i2c.writeByte(0x6b, 0x10, 0b01101011);
	
	uint8_t buffer[6];
	int16_t data;
	
	const double pi = 3.141592653589793;
	const double gyroToW = 500/(pow(2,15)-1)*pi/180.0; // radians/second/count
	const double dtGyros = 1.0/119.0;
 	
	double wbi_B[3];
	double Bw_B[3] = {0, 0, 0};
	
	SendUdp sendUdp;
	sendUdp.initialize("10.128.1.9", "10.128.1.10", 12345);
	
	double q_BI[4] = {1, 0, 0, 0};
	
	std::cout << "Wait to initialize Gyro bias" << std::endl;
	
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
	
	std::cout << "Gyro bias initialized" << std::endl;
	
	while (true)
	{
		i2c.readBuffer(0x6b, 0x18, buffer, 6);
		
		data = (buffer[1] << 8) | buffer[0]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		
		wbi_B[0] = -1.0*gyroToW*data - Bw_B[0]; // converting to engineering units (radians/second)
		
		data = (buffer[3] << 8) | buffer[2]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		
		wbi_B[1] = gyroToW*data - Bw_B[1]; // converting to engineering units (radians/second)
		
		data = (buffer[5] << 8) | buffer[4]; // converting to signed 16 bit value (in raw counts, no units), by bit shifting
		
		wbi_B[2] = gyroToW*data - Bw_B[2]; // converting to engineering units (radians/second)
		
		double wmag = sqrt(wbi_B[0]*wbi_B[0]+wbi_B[1]*wbi_B[1]+wbi_B[2]*wbi_B[2]);
		
		double s = sin(0.5*wmag*dtGyros);
		double c = cos(0.5*wmag*dtGyros);
		
		double psi[3];
		
		psi[0] = s*wbi_B[0]/wmag;
		psi[1] = s*wbi_B[1]/wmag;
		psi[2] = s*wbi_B[2]/wmag;
		
		double qnew_BI[4];
		
		qnew_BI[0] = c*q_BI[0] - psi[0]*q_BI[1] - psi[1]*q_BI[2] - psi[2]*q_BI[3];
		qnew_BI[1] = c*q_BI[1] + psi[0]*q_BI[0] - psi[1]*q_BI[3] + psi[2]*q_BI[2];
		qnew_BI[2] = c*q_BI[2] + psi[1]*q_BI[0] + psi[0]*q_BI[3] - psi[2]*q_BI[1];
		qnew_BI[3] = c*q_BI[3] - psi[0]*q_BI[2] + psi[1]*q_BI[1] + psi[2]*q_BI[0];
		
		double qmag = sqrt(qnew_BI[0]*qnew_BI[0]+qnew_BI[1]*qnew_BI[1]+qnew_BI[2]*qnew_BI[2]+qnew_BI[3]*qnew_BI[3]);
		
		q_BI[0] = qnew_BI[0]/qmag;
		q_BI[1] = qnew_BI[1]/qmag;
		q_BI[2] = qnew_BI[2]/qmag;
		q_BI[3] = qnew_BI[3]/qmag;
		
		//std::cout << wbi_B[0] << ", " << wbi_B[1] << ", " << wbi_B[2] << std::endl;
		
		sendUdp.sendDoubles(q_BI, 4);
		
		//std::cout << "time:" << SleepTimer::getCurrentTime() << std::endl;
		myTimer.sleep();
	}
	
	/*
	uint8_t byte;
	
	byte = i2c.readByte(0x6b, 0x0F);
	
	std::cout << "The WHO_AM_I register = " << std::bitset<8>(byte) << std::endl;
	*/
	
    return 0;
}
  