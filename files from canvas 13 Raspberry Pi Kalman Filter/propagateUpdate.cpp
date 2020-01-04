#include "propagateUpdate.hpp"
#include <cmath>

// Propagate Covariance
void covarianceProp(const double (&wbi_B)[3], const double dtGyro, const double sigmaw, const double sigmaBw, double (&P)[6][6])
{
  // norm(wbi_b)
  double wmag = sqrt(wbi_B[0]*wbi_B[0]+wbi_B[1]*wbi_B[1]+wbi_B[2]*wbi_B[2]);
  if (wmag == 0)
  {
    return;
  }

  // wX = X(wbi_B)
  double wX[3][3];
  X(wbi_B, wX);

  const double s = sin(wmag*dtGyro);
  const double c = cos(wmag*dtGyro);

  // I3 = eye(3);
  const double I3[3][3] = { { 1, 0, 0},
                            { 0, 1, 0},
                            { 0, 0, 1} };

  // O3 = zeros(3,3);
  const double O3[3][3] = { { 0, 0, 0},
                            { 0, 0, 0},
                            { 0, 0, 0} };

  // - wX*sin(wmag*dt)/wmag
  double wXs_wmag[3][3];
  mult3x3Scalar(wX, -1.0*s/wmag, wXs_wmag);

  // wX*(1-cos(wmag*dt))/wmag^2
  double wXc_wmag2[3][3];
  mult3x3Scalar(wX,(1.0-c)/pow(wmag,2), wXc_wmag2);

  // wX*wX*(1-cos(wmag*dt))/wmag^2
  double wXwXc_wmag2[3][3];
  mult3x3(wX, wXc_wmag2, wXwXc_wmag2);

  // I3 - wX*sin(wmag*dt)/wmag
  double sum1[3][3];
  add3x3(I3, wXs_wmag, sum1);

  // I3 - wX*sin(wmag*dt)/wmag + wX*wX*(1-cos(wmag*dt))/wmag^2
  double Phi11[3][3];
  add3x3(sum1, wXwXc_wmag2, Phi11);

  // - I3*dt
  double I3dtGyros[3][3];
  mult3x3Scalar(I3,-1.0*dtGyro,I3dtGyros);

  // - wX*(wmag*dt-sin(wmag*dt))/wmag^3
  double wXs_wmag3[3][3];
  mult3x3Scalar(wX,-1.0*(wmag*dtGyro-s)/pow(wmag,3), wXs_wmag3);

  // - wX*wX*(wmag*dt-sin(wmag*dt))/wmag^3
  double wXwXs_wmag3[3][3];
  mult3x3(wX,wXs_wmag3,wXwXs_wmag3);

  //  wX*(1-cos(wmag*dt))/wmag^2 - I3*dt
  double sum2[3][3];
  add3x3(wXc_wmag2, I3dtGyros, sum2);

  // wX*(1-cos(wmag*dt))/wmag^2 - I3*dt - wX*wX*(wmag*dt-sin(wmag*dt))/wmag^3
  double Phi12[3][3];
  add3x3(sum2,wXwXs_wmag3,Phi12);

  // Form State Transition Matrix
  double Phi[6][6];

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      Phi[i][j] = Phi11[i][j];
    }
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      Phi[i][j+3] = Phi12[i][j];
    }
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      Phi[i+3][j] = O3[i][j];
    }
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      Phi[i+3][j+3] = I3[i][j];
    }
  }

  // Transpose State Transistion Matrix
  double PhiT[6][6];
  trans6x6(Phi,PhiT);

  // Form Q Matrix
  double Q[6][6] = { { 0, 0, 0, 0, 0, 0},
                     { 0, 0, 0, 0, 0, 0},
                     { 0, 0, 0, 0, 0, 0},
                     { 0, 0, 0, 0, 0, 0},
                     { 0, 0, 0, 0, 0, 0},
                     { 0, 0, 0, 0, 0, 0} };

  const double Q11 = pow(sigmaw,2)*dtGyro + 1.0/3.0*pow(sigmaBw,2)*pow(dtGyro,3);
  const double Q12 = -1.0/2.0*pow(sigmaBw,2)*pow(dtGyro,2);
  const double Q21 = Q12;
  const double Q22 = pow(sigmaBw,2)*dtGyro;

  Q[0][0] = Q11;
  Q[1][1] = Q11;
  Q[2][2] = Q11;
  Q[0][3] = Q12;
  Q[1][4] = Q12;
  Q[2][5] = Q12;
  Q[3][0] = Q21;
  Q[4][1] = Q21;
  Q[5][2] = Q21;
  Q[3][3] = Q22;
  Q[4][4] = Q22;
  Q[5][5] = Q22;

  // P*Phi'
  double PPhiT[6][6];
  mult6x6(P,PhiT,PPhiT);

  // Phi*P*Phi'
  double PhiPPhiT[6][6];
  mult6x6(Phi,PPhiT,PhiPPhiT);

  // Phi*P*Phi' + Q
  double Pnew[6][6];
  add6x6(PhiPPhiT,Q,Pnew);

  // Make Sure Covariance Matrix is Symmetric
  double PnewT[6][6];
  trans6x6(Pnew,PnewT);

  double PSymmetric[6][6];
  add6x6(Pnew,PnewT,PSymmetric);

  mult6x6Scalar(PSymmetric,0.5,P);
}

// Kalman Filter Update Equations
void update(double (&q_BI)[4], double (&Bw_B)[3], double (&P)[6][6], const double (&v_B)[3], const double (&v_I)[3], const double sigmav)
{
  // Create DCM from estimate of q_BI
  double A_BI[3][3];
  q2A(q_BI,A_BI);

  // Project Inertial Vector to Body
  double vhat_B[3];
  matMult(A_BI,v_I,vhat_B);

  // Create Cross Body Matrix from vhat_B
  double vhat_BX[3][3];
  X(vhat_B,vhat_BX);

  // Transpose the Cross Product Matrix
  double vhat_BXT[3][3];
  trans3x3(vhat_BX,vhat_BXT);

  // Extract Upper Left Covariance Sub Matrix
  double P11[3][3];
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      P11[i][j] = P[i][j];
    }
  }

  // P(1:3,1:3)*vhat_BX'
  double P11vhat_BXT[3][3];
  mult3x3(P11,vhat_BXT,P11vhat_BXT);

  // vhat_BX*P(1:3,1:3)*vhat_BX'
  double vhat_BXP11vhat_BXT[3][3];
  mult3x3(vhat_BX,P11vhat_BXT,vhat_BXP11vhat_BXT);

  // Form measurment covariance matrix
  double R[3][3] = { { 0, 0, 0},
                     { 0, 0, 0},
                     { 0, 0, 0} };
  R[0][0] = pow(sigmav,2);
  R[1][1] = pow(sigmav,2);
  R[2][2] = pow(sigmav,2);

  // H*P*H' + R
  double HPHT_R[3][3];
  add3x3(vhat_BXP11vhat_BXT,R,HPHT_R);

  // inv(H*P*H'+R)
  double invHPHT_R[3][3];
  inv3x3(HPHT_R,invHPHT_R);

  // H'*inv(H*P*H'+R)
  double vhat_BXTinvHPHT_R[3][3];
  mult3x3(vhat_BXT,invHPHT_R,vhat_BXTinvHPHT_R);

  // P(1:3,1:3)*H'*inv(H*P*H'+R)
  double K1[3][3];
  mult3x3(P11,vhat_BXTinvHPHT_R,K1);

  // Extract Bottom Left Covariance Sub Matrix
  double P21[3][3];
  for (int i = 3; i < 6; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      P21[i-3][j] = P[i][j];
    }
  }

  // P(4:6,1:3)*H'*inv(H*P*H'+R)
  double K2[3][3];
  mult3x3(P21,vhat_BXTinvHPHT_R,K2);

  // Calculate Residual
  double residual[3];
  residual[0] = v_B[0] - vhat_B[0];
  residual[1] = v_B[1] - vhat_B[1];
  residual[2] = v_B[2] - vhat_B[2];

  // Calculate Error Angles  K1*residual
  double dth[3];
  matMult(K1,residual,dth);

  // Calculate Error Bw K2*residual
  double dBw[3];
  matMult(K2,residual,dBw);

  // Update Quaternion
  double q[4];
  q[0] = q_BI[0] - (dth[0]*q_BI[1])/2.0 - (dth[1]*q_BI[2])/2.0 - (dth[2]*q_BI[3])/2.0;
  q[1] = q_BI[1] + (dth[0]*q_BI[0])/2.0 - (dth[1]*q_BI[3])/2.0 + (dth[2]*q_BI[2])/2.0;
  q[2] = q_BI[2] + (dth[1]*q_BI[0])/2.0 + (dth[0]*q_BI[3])/2.0 - (dth[2]*q_BI[1])/2.0;
  q[3] = q_BI[3] - (dth[0]*q_BI[2])/2.0 + (dth[1]*q_BI[1])/2.0 + (dth[2]*q_BI[0])/2.0;

  double qmag = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);

  q_BI[0] = q[0]/qmag;
  q_BI[1] = q[1]/qmag;
  q_BI[2] = q[2]/qmag;
  q_BI[3] = q[3]/qmag;

  // K1*vhat_BX
  double K1vhat_BX[3][3];
  mult3x3(K1,vhat_BX,K1vhat_BX);

  // K2*vhat_BX
  double K2vhat_BX[3][3];
  mult3x3(K2,vhat_BX,K2vhat_BX);

  double KH[6][6] = { { 0, 0, 0, 0, 0, 0},
                      { 0, 0, 0, 0, 0, 0},
                      { 0, 0, 0, 0, 0, 0},
                      { 0, 0, 0, 0, 0, 0},
                      { 0, 0, 0, 0, 0, 0},
                      { 0, 0, 0, 0, 0, 0} };

  // For K*H
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      KH[i][j] = K1vhat_BX[i][j];
    }
  }

  for (int i = 3; i < 6; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      KH[i][j] = K2vhat_BX[i-3][j];
    }
  }

  // -1.0*K*H
  double nKH[6][6];
  mult6x6Scalar(KH,-1.0,nKH);

  // Form eye(6)
  double I6[6][6] = { { 1, 0, 0, 0, 0, 0},
                      { 0, 1, 0, 0, 0, 0},
                      { 0, 0, 1, 0, 0, 0},
                      { 0, 0, 0, 1, 0, 0},
                      { 0, 0, 0, 0, 1, 0},
                      { 0, 0, 0, 0, 0, 1} };

  // I6 - K*H
  double I6nKH[6][6];
  add6x6(I6,nKH,I6nKH);

  // (I6 - K*H)*P
  double Ptemp[6][6];
  mult6x6(I6nKH,P,Ptemp);

  // Transpose P
  double PtempT[6][6];
  trans6x6(Ptemp,PtempT);

  // P + P'
  double PSymmetric[6][6];
  add6x6(Ptemp,PtempT,PSymmetric);

  // (P + P)/2
  mult6x6Scalar(PSymmetric,0.5,P);

  // Update Gyro Bias
  Bw_B[0] += dBw[0];
  Bw_B[1] += dBw[1];
  Bw_B[2] += dBw[2];
}

void X(const double (&v)[3], double (&vX)[3][3])
{
  vX[0][0] = 0;
  vX[0][1] = -v[2];
  vX[0][2] = v[1];
  vX[1][0] = v[2];
  vX[1][1] = 0;
  vX[1][2] = -v[0];
  vX[2][0] = -v[1];
  vX[2][1] = v[0];
  vX[2][2] = 0;
}

void add3x3(const double (&A)[3][3], const double (&B)[3][3], double (&C)[3][3])
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      C[i][j] = A[i][j] + B[i][j];
    }
  }
}

void mult3x3(const double (&A)[3][3], const double (&B)[3][3], double (&C)[3][3])
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      C[i][j] = 0;

      for (int k = 0; k < 3; k++)
      {
        C[i][j] += A[i][k]*B[k][j];
      }
    }
  }
}

void mult3x3Scalar(const double (&A)[3][3], const double (&b), double (&C)[3][3])
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      C[i][j] = A[i][j]*b;
    }
  }
}

void trans3x3(const double (&A)[3][3], double (&B)[3][3])
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      B[i][j] = A[j][i];
    }
  }
}

void add6x6(const double (&A)[6][6], const double (&B)[6][6], double (&C)[6][6])
{
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      C[i][j] = A[i][j] + B[i][j];
    }
  }
}

void mult6x6(const double (&A)[6][6], const double (&B)[6][6], double (&C)[6][6])
{
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      C[i][j] = 0;

      for (int k = 0; k < 6; k++)
      {
        C[i][j] += A[i][k]*B[k][j];
      }
    }
  }
}

void mult6x6Scalar(const double (&A)[6][6], const double (&b), double (&C)[6][6])
{
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      C[i][j] = A[i][j]*b;
    }
  }
}

void trans6x6(const double (&A)[6][6], double (&B)[6][6])
{
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      B[i][j] = A[j][i];
    }
  }
}

void q2A(const double (&q)[4], double (&A)[3][3])
{
  A[0][0] = q[0]*q[0] - q[2]*q[2] - q[3]*q[3] + q[1]*q[1];
  A[0][1] = 2.0*q[0]*q[3] + 2.0*q[1]*q[2];
  A[0][2] = 2.0*q[1]*q[3] - 2.0*q[0]*q[2];
  A[1][0] = 2.0*q[1]*q[2] - 2.0*q[0]*q[3];
  A[1][1] = q[0]*q[0] + q[2]*q[2] - q[3]*q[3] - q[1]*q[1];
  A[1][2] = 2.0*q[0]*q[1] + 2.0*q[2]*q[3];
  A[2][0] = 2.0*q[0]*q[2] + 2.0*q[1]*q[3];
  A[2][1] = 2.0*q[2]*q[3] - 2.0*q[0]*q[1];
  A[2][2] = q[0]*q[0] - q[2]*q[2] + q[3]*q[3] - q[1]*q[1];
}

void matMult(const double (&A)[3][3], const double (&b)[3], double (&vout)[3])
{
  vout[0] = A[0][0]*b[0] + A[0][1]*b[1] + A[0][2]*b[2];
  vout[1] = A[1][0]*b[0] + A[1][1]*b[1] + A[1][2]*b[2];
  vout[2] = A[2][0]*b[0] + A[2][1]*b[1] + A[2][2]*b[2];
}

void inv3x3(const double (&A)[3][3], double (&B)[3][3])
{
  const double det = A[0][0]*A[1][1]*A[2][2] - A[0][0]*A[1][2]*A[2][1] - A[0][1]*A[1][0]*A[2][2] +
                     A[0][1]*A[1][2]*A[2][0] + A[0][2]*A[1][0]*A[2][1] - A[0][2]*A[1][1]*A[2][0];

  B[0][0] = (A[1][1]*A[2][2] - A[1][2]*A[2][1])/det;
  B[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1])/det;
  B[0][2] = (A[0][1]*A[1][2] - A[0][2]*A[1][1])/det;
  B[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0])/det;
  B[1][1] = (A[0][0]*A[2][2] - A[0][2]*A[2][0])/det;
  B[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0])/det;
  B[2][0] = (A[1][0]*A[2][1] - A[1][1]*A[2][0])/det;
  B[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0])/det;
  B[2][2] = (A[0][0]*A[1][1] - A[0][1]*A[1][0])/det;
}