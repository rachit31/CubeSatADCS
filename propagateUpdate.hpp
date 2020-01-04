/** 
 *  @file    propagateUpdate.hpp
 *  @author  Rachit Bhatia
 *  @date    04/24/2018
 *  @version 1.0.0
 *  
 *  @brief Declares the functions that will be used in the Kalman filter propagation and update
 *
 */

#ifndef PROPAGATEUPDATE_HPP_
#define PROPAGATEUPDATE_HPP_

  /** 
  *   @brief  Propagate Covariance Matrix
  *  
  *   @param  wbi_B is the angular velocity projected to the body frame
  *   @param  dtGryo is the gyro sample time
  *   @param  sigmaw is standard deviation of the gyro noise
  *   @param  sigmaBw is standard deviation of the noise driving the gyro bias
  *   @param  P is the covariance matrix.  P will be updated in this function
  *   @return void
  */
void covarianceProp(const double (&wbi_B)[3], const double dtGyro, const double sigmaw, const double sigmaBw, double (&P)[6][6]);

  /** 
  *   @brief  Kalman Update
  *  
  *   @param  q_BI is the current estimate of the quaternion.  q_BI will be updated in this function.
  *   @param  Bw_B is the current estimate of the gyro bias.  Bw_B will be updated in this function.
  *   @param  P is the current covariance matrix.  P will be updated in this function.
  *   @param  v_B is the sensor measurement vector in the body frame
  *   @param  v_I is the expected reference vector in the inertial frame
  *   @param  sigmav is the standard deviation of the noise on the vector measurement
  *   @return void
  */
void update(double (&q_BI)[4], double (&Bw_B)[3], double (&P)[6][6], const double (&v_B)[3], const double (&v_I)[3], const double sigmav);

  /** 
  *   @brief  Forms a crossproduct matrix from a vector
  */
void X(const double (&v)[3], double (&vX)[3][3]);

  /** 
  *   @brief  Adds 2 3x3 matrices
  */
void add3x3(const double (&A)[3][3], const double (&B)[3][3], double (&C)[3][3]);

  /** 
  *   @brief  Multiplies two 3x3 matrices
  */
void mult3x3(const double (&A)[3][3], const double (&B)[3][3], double (&C)[3][3]);

  /** 
  *   @brief  Multiply a 3x3 matrix by a scalar
  */
void mult3x3Scalar(const double (&A)[3][3], const double (&b), double (&C)[3][3]);

  /** 
  *   @brief  Transposes a 3x3 matrix
  */
void trans3x3(const double (&A)[3][3], double (&B)[3][3]);

  /** 
  *   @brief  Calculates the matrix inverse of a 3x3 matrix
  */
void inv3x3(const double (&A)[3][3], double (&B)[3][3]);

  /** 
  *   @brief  Adds two 6x6 matrices
  */
void add6x6(const double (&A)[6][6], const double (&B)[6][6], double (&C)[6][6]);

  /** 
  *   @brief  Multiplies two 6x6 matrices
  */
void mult6x6(const double (&A)[6][6], const double (&B)[6][6], double (&C)[6][6]);

  /** 
  *   @brief  Multiplies a 6x6 matrix by a scalar
  */
void mult6x6Scalar(const double (&A)[6][6], const double (&b), double (&C)[6][6]);

  /** 
  *   @brief  Transposes a 6x6 matrix
  */
void trans6x6(const double (&A)[6][6], double (&B)[6][6]);

  /** 
  *   @brief  Converts a quaternion to a direction cosine matrix
  */
void q2A(const double (&q)[4], double (&A)[3][3]);

  /** 
  *   @brief  Multiplies a 3x3 matrix and a vector vout = A*b
  *  
  *   @param  A is a 3x3 matrix
  *   @param  b is a 3 element vector
  *   @param  vout is the 3 element resultant vector
  *   @return void
  */
void matMult(const double (&A)[3][3], const double (&b)[3], double (&vout)[3]);

#endif