/** 
 *  @file    KalmanPi.hpp
 *  @author  Rachit Bhatia
 *  @date    05/01/2018
 *  @version 1.0.0
 *  
 *  @brief Defines constants and global variables for Kalman Filter
 *
 */

 #ifndef KALMANPI_HPP_
 #define KALMANPI_HPP_
 
 #include "i2c.hpp"
 
 #define LOCALIP "10.128.1.9"
 #define REMOTEIP "10.128.1.10"
 
 #define SNDPORT 12346
 
 #define SNDNUM 4
 
 extern double gyroBuffer[3]; ///< Buffer into which Gyro data to send is stored
 extern Mutex gyroMutex;      ///< Mutex for signaling controller that the gyroBuffer is being copied or saved
 
 extern double accelBuffer[3]; ///< Buffer into which Accelerometer data to send is stored
 extern Mutex accelMutex;      ///< Mutex for signaling controller that the accelBuffer is being copied or saved
 
 extern double magBuffer[3]; ///< Buffer into which Magnetometer data to send is stored
 extern Mutex magMutex;      ///< Mutex for signaling controller that the magBuffer is being copied or saved
 
 extern double sendBuffer[SNDNUM];
 extern Mutex sendMutex;
 
 extern double Bw_B[3];
 extern double a_I[3];
 extern double B_I[3];
 
 extern I2C i2c;
 
 extern const double dtGyro;

  
 void initializeGyros();  ///< initialize Gyro Bias function
 void initializeAccel();  ///< initialize Accelerometer function
 void initializeMag();    ///< initialize Magnetometer function
 
 void* readGyros(void *pArg);     ///< read Gyro data function
 void* readAccel(void *pArg);     ///< read Accelerometer data function
 void* readMag(void *pArg);       ///< read Magnetometer data function
 void* sendFunction(void *pArg);  ///< send Gyro data function
 void* propagateANDupdate(void *pArg);     ///< State and State Covariance Propagate and Update function
 void* readAccel(void *pArg);     ///< read Accelerometer data function
 
 #endif
 
 
 