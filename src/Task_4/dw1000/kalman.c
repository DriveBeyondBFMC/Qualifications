/* Includes ------------------------------------------------------------------*/
#include "kalman.h"	
#include "matrix.h"				 				 		


typedef struct  _tCovariance
{
  float PNowOpt[LENGTH];
  float PPreOpt[LENGTH];
}tCovariance;







static tOptimal      tOpt;
static tCovariance   tCov;
//float         Z[LENGTH]  = {4000};          
static float         I[LENGTH]  = {1};              
static float         X[LENGTH]  = {9.8};              
static float         P[LENGTH]  = {0};              
static float         K[LENGTH]  = {0};             
static float         Temp3[LENGTH] = {0};           //  

static float         F[LENGTH]  = {1};              //  
static float         Q[LENGTH]  = {0.0001f};//0.0001f              // 
static float         R[LENGTH]  = {2};              //   

static float         H[LENGTH]  = {1};              //  
static float         Temp1[LENGTH] = {1};           //  
static float         Temp2[LENGTH] = {10000};       //  


void KalMan_PramInit(void)
{
  unsigned char   i;
  
  for (i=0; i<LENGTH; i++)
  {
    tOpt.XPreOpt[i] = Temp1[i];          
  }
  for (i=0; i<LENGTH; i++)
  {
    tCov.PPreOpt[i] = Temp2[i];           
  }
}


float KalMan_Update(double *Z)
{
	unsigned char i;  
	MatrixMul(F, tOpt.XPreOpt, X, ORDER, ORDER, ORDER);       //  X(k|k-1) = F(k,k-1)*X(k-1|k-1)

	MatrixCal(F, tCov.PPreOpt, Temp1, ORDER);
	MatrixAdd(Temp1, Q, P, ORDER, ORDER);                     //  P(k|k-1) = F(k,k-1)*P(k-1|k-1)*F(k,k-1)'+Q

	MatrixCal(H, P, Temp1, ORDER);
	MatrixAdd(Temp1, R, Temp1, ORDER, ORDER);
	Gauss_Jordan(Temp1, ORDER);
	MatrixTrans(H, Temp2, ORDER, ORDER);
	MatrixMul(P, Temp2, Temp3, ORDER, ORDER, ORDER);
	MatrixMul(Temp1, Temp3, K, ORDER, ORDER, ORDER);          //  Kg(k) = P(k|k-1)*H' / (H*P(k|k-1)*H' + R)

	MatrixMul(H, X, Temp1, ORDER, ORDER, ORDER);
	MatrixMinus(Z, Temp1, Temp1, ORDER, ORDER);
	MatrixMul(K, Temp1, Temp2, ORDER, ORDER, ORDER);
	MatrixAdd(X, Temp2, tOpt.XNowOpt, ORDER, ORDER);          //  X(k|k) = X(k|k-1)+Kg(k)*(Z(k)-H*X(k|k-1))

	MatrixMul(K, H, Temp1, ORDER, ORDER, ORDER);
	MatrixMinus((double *)I, Temp1, Temp1, ORDER, ORDER);
	MatrixMul(Temp1, P, tCov.PNowOpt, ORDER, ORDER, ORDER);   //   P(k|k) =£¨I-Kg(k)*H£©*P(k|k-1)

	for (i=0; i<LENGTH; i++)
	{
	  tOpt.XPreOpt[i] = tOpt.XNowOpt[i];
	  tCov.PPreOpt[i] = tCov.PNowOpt[i];
	}
	
	return tOpt.XNowOpt[0];
}
