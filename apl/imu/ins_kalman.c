#include "ins_kalman.h"
#include "comm/project_def.h"

#include <stdio.h>
#include <math.h>


#define MAX_VEL_CALIB_VALUE  1.25  // m/s
#define MAX_VEL_VALUE   30.0      // m/s


//static double sDt = 1.0;

static double sP0_4_4[] = { 3.0, 0, 0, 0, 
                            0, 3.0, 0, 0,
							0, 0, 0.3, 0,
                            0, 0, 0, 0.3 };

static double sQ_4_4[] = { 1.0, 0, 0, 0,
                           0, 1.0, 0, 0,
                           0, 0, 0.1, 0,
                           0, 0, 0, 0.1 };

static double sI4[] = { 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0,
						0, 0, 0, 1 };

static double sI2[] = { 1, 0,
                        0, 1 };

// 1,0,dt,0; 0,1,0,dt; 0,0,1,0; 0,0,0,1
// 根据dt填充 [0,2], [1,3]
static double sA_4_4[] = { 1, 0, 0, 0,
                           0, 1, 0, 0,
                           0, 0, 1, 0,
                           0, 0, 0, 1 };

static double sH_PosVel_4_4[] = { 1, 0, 0, 0,
                                  0, 1, 0, 0,
                                  0, 0, 1, 0,
                                  0, 0, 0, 1 };

static double sH_Pos_2_4[] = { 1, 0, 0, 0,
                               0, 1, 0, 0 };

static double sH_Vel_2_4[] = { 0, 0, 1, 0,
                               0, 0, 0, 1 };

// 根据位置/速度的方差填充：(0, 5, 10, 15)
static double sR_PosVel[] = { 0, 0, 0, 0,
                              0, 0, 0, 0, 
                              0, 0, 0, 0,
                              0, 0, 0, 0 };

// 根据位置的方差填充: (0, 3)
static double sR_Pos[] = { 0, 0,
                           0, 0 };

// 根据速度的方差填充: (0, 3)
static double sR_Vel[] = { 0, 0,
                           0, 0 };


// matrix should be stored in col major
static double m_P[4*4];
static double m_Q[4*4];
static double m_A[4*4];
static double m_B[4*4];
static double m_Pk_k1[4*4];
static double m_X[4*1];

static void ins_kalman_MeasureUpdate_Pos(double measurePos[], double std_e, double std_n);
static void ins_kalman_MeasureUpdate_Vel(double measureVel[], double std_ve, double std_vn);

static void ins_kalman_MeasureUpdate_AttiBias(double measure_dV[], double std_dve, double std_dvn);


void ins_kalman_init()
{
	insMat_Rowmjr_to_Colmjr(sP0_4_4, 4, 4, m_P);
	insMat_Rowmjr_to_Colmjr(sQ_4_4, 4, 4, m_Q);
	insMat_Rowmjr_to_Colmjr(sA_4_4, 4, 4, m_A);
	insMat_Rowmjr_to_Colmjr(sI4, 4, 4, m_B);

}

/*
void CKalman::Initiate()
{
	
	m_P = CMatrix(sP0, 4, 4);
	m_Q = CMatrix(sQ, 4, 4);
	m_R = CMatrix(sR, 2, 2);

	m_A = CMatrix(sA, 4, 4);
	m_B = CMatrix(sI4, 4, 4);
	m_H = CMatrix(sH, 2, 4);

}
*/

void ins_kalman_TimeUpdate(float dt)
{
	/*
	m_P = m_A *m_P *m_A.GetTranspose() + m_B *m_Q *m_B.GetTranspose();
	m_Pk_k1 = m_P;
	*/

	double m_AT[4*4];
	double m_BT[4*4];
	double matTemp[4*4];
	double matTemp2[4*4];

	// m_A : col_major
	m_A[8] = dt; // [0,2] : 0 + 2 *4 = 8
	m_A[13] = dt; // [1,3] : 1 + 3*4 = 13

	insMat_Colmjr_Transpose(m_A, 4, 4, m_AT);
	insMat_Colmjr_Transpose(m_B, 4, 4, m_BT);

	// matTemp2 = m_A * m_P * m_AT;
	ins_matmul("NN", 4, 4, 4, 1.0, m_P, m_AT, 0.0, matTemp);
	ins_matmul("NN", 4, 4, 4, 1.0, m_A, matTemp, 0.0, matTemp2);

	// m_A *m_P *m_A.GetTranspose() + m_B *m_Q *m_B.GetTranspose();
	ins_matmul("NN", 4, 4, 4, 1.0, m_Q, m_BT, 0.0, matTemp);
	ins_matmul("NN", 4, 4, 4, 1.0, m_B, matTemp, 1.0, matTemp2);

	ins_matcpy(m_Pk_k1, matTemp2, 4, 4);

	// 考虑多次进行时间更新的情况
	ins_matcpy(m_P, m_Pk_k1, 4, 4);

}

void ins_kalman_MeasureUpdate(const ins_geopos_t* pPos, const ins_vel_t* pVel, ins_geopos_t* pPosFus, ins_vel_t* pVelFus)
{
	double dLon, dLat;
	double dE, dN;
	double measurePos[2];
	double d_ve, d_vn;
	double measureVel[2];

	// pos fusion
	if (NULL != pPos)
	{
		dLon = pPos->lng - pPosFus->lng;
		dLat = pPos->lat - pPosFus->lat;

		dE = dLon * DEG2RAD * WGS84_a * cos(pPos->lat * DEG2RAD);
		dN = dLat * DEG2RAD * WGS84_a;

		measurePos[0] = dE;
		measurePos[1] = dN;

		ins_kalman_MeasureUpdate_Pos(measurePos, pPos->std_e, pPos->std_n);

		dE = m_X[0];
		dN = m_X[1];


		pPosFus->lng += dE / WGS84_a * RAD2DEG / cos(pPos->lat * DEG2RAD);
		pPosFus->lat += dN / WGS84_a * RAD2DEG;

		pVelFus->ve += m_X[2];
		pVelFus->vn += m_X[3];

	}

	// vel fusion
	if (NULL != pVel)
	{
		d_ve = pVel->ve - pVelFus->ve;
		d_vn = pVel->vn - pVelFus->vn;

		measureVel[0] = d_ve;
		measureVel[1] = d_vn;

		ins_kalman_MeasureUpdate_Vel(measureVel, pVel->std_ve, pVel->std_vn);

		dE = m_X[0];
		dN = m_X[1];

		pPosFus->lng += dE / WGS84_a * RAD2DEG / cos(pPosFus->lat * DEG2RAD);
		pPosFus->lat += dN / WGS84_a * RAD2DEG;

		pVelFus->ve += m_X[2];
		pVelFus->vn += m_X[3];
	}

}

static void ins_kalman_MeasureUpdate_Pos(double measurePos[], double std_e, double std_n)
{
	double Kk[4*2];
	double KkT[2*4];
	double Z[2*1];
	//CMatrix I(sI4, 4, 4);
	double I2[2*2];
	double I4[4*4];

	double m_H[2*4];
	double m_HT[4*2];
	double m_R[2*2];

	double matTemp[4*2];
	double matTemp2[2*2];
	double matTemp2_inv[2*2];

	double matI_KH[4*4];
	double matI_KH_T[4*4];

	//double m_X[4*1];

	double newP[4*4];
	double newP2[4*4];


	insMat_Rowmjr_to_Colmjr(sH_Pos_2_4, 2, 4, m_H);
	insMat_Colmjr_Transpose(m_H, 2, 4, m_HT);

	insMat_Rowmjr_to_Colmjr(sR_Pos, 2, 2, m_R);
	insMat_Rowmjr_to_Colmjr(sI2, 2, 2, I2);
	insMat_Rowmjr_to_Colmjr(sI4, 4, 4, I4);

	//
	m_R[0] = std_e*std_e;
	m_R[3] = std_n*std_n;

	Z[0] = measurePos[0];
	Z[1] = measurePos[1];

	// matTemp = m_P *m_H.GetTranspose()
	// matTemp2 = m_H *m_P *m_H.GetTranspose() + m_R
	ins_matmul("NN", 4, 2, 4, 1.0, m_P, m_HT, 0.0, matTemp);
	ins_matmul("NN", 2, 2, 4, 1.0, m_H, matTemp, 0.0, matTemp2);
	ins_matmul("NN", 2, 2, 2, 1.0, I2, m_R, 1.0, matTemp2);

	// 
	ins_matcpy(matTemp2_inv, matTemp2, 2, 2);
	ins_matinv(matTemp2_inv, 2);

	// Kk = m_P *m_H.GetTranspose() * tmpMat.GetInverse();
	ins_matmul("NN", 4, 2, 2, 1.0, matTemp, matTemp2_inv, 0.0, Kk);

	insMat_Colmjr_Transpose(Kk, 4, 2, KkT);

	// update state vector X
	// mX = Kk * Z;
	ins_matmul("NN", 4, 1, 2, 1.0, Kk, Z, 0.0, m_X);

	// matI_KH = I - Kk *m_H;
	ins_matcpy(matI_KH, I4, 4, 4);
	ins_matmul("NN", 4, 4, 2, -1.0, Kk, m_H, 1.0, matI_KH);

	// m_P = matI_KH * m_P * matI_KH.GetTranspose() + Kk * m_R *Kk.GetTranspose();
	insMat_Colmjr_Transpose(matI_KH, 4, 4, matI_KH_T);

	ins_matmul("NN", 4, 4, 4, 1.0, m_P, matI_KH_T, 0.0, newP);
	ins_matmul("NN", 4, 4, 4, 1.0, matI_KH, newP, 0.0, newP2);

	ins_matmul("NN", 4, 2, 2, 1.0, Kk, m_R, 0.0, matTemp);
	ins_matmul("NN", 4, 4, 2, 1.0, matTemp, KkT, 1.0, newP2);

	// update matrix P
	ins_matcpy(m_P, newP2, 4, 4);

}

static void ins_kalman_MeasureUpdate_Vel(double measureVel[], double std_ve, double std_vn)
{
	double Kk[4*2];
	double KkT[2*4];
	double Z[2*1];
	//CMatrix I(sI4, 4, 4);
	double I2[2*2];
	double I4[4*4];

	double m_H[2*4];
	double m_HT[4*2];
	double m_R[2*2];

	double matTemp[4*2];
	double matTemp2[2*2];
	double matTemp2_inv[2*2];

	double matI_KH[4*4];
	double matI_KH_T[4*4];

	//double m_X[4*1];

	double newP[4*4];
	double newP2[4*4];


	insMat_Rowmjr_to_Colmjr(sH_Vel_2_4, 2, 4, m_H);
	insMat_Colmjr_Transpose(m_H, 2, 4, m_HT);

	insMat_Rowmjr_to_Colmjr(sR_Vel, 2, 2, m_R);
	insMat_Rowmjr_to_Colmjr(sI2, 2, 2, I2);
	insMat_Rowmjr_to_Colmjr(sI4, 4, 4, I4);

	//
	m_R[0] = std_ve*std_ve;
	m_R[3] = std_vn*std_vn;

	Z[0] = measureVel[0];
	Z[1] = measureVel[1];

	// matTemp = m_P *m_H.GetTranspose()
	// matTemp2 = m_H *m_P *m_H.GetTranspose() + m_R
	ins_matmul("NN", 4, 2, 4, 1.0, m_P, m_HT, 0.0, matTemp);
	ins_matmul("NN", 2, 2, 4, 1.0, m_H, matTemp, 0.0, matTemp2);
	ins_matmul("NN", 2, 2, 2, 1.0, I2, m_R, 1.0, matTemp2);

	// 
	ins_matcpy(matTemp2_inv, matTemp2, 2, 2);
	ins_matinv(matTemp2_inv, 2);

	// Kk = m_P *m_H.GetTranspose() * tmpMat.GetInverse();
	ins_matmul("NN", 4, 2, 2, 1.0, matTemp, matTemp2_inv, 0.0, Kk);

	insMat_Colmjr_Transpose(Kk, 4, 2, KkT);

	// update state vector X
	// mX = Kk * Z;
	ins_matmul("NN", 4, 1, 2, 1.0, Kk, Z, 0.0, m_X);

	// matI_KH = I - Kk *m_H;
	ins_matcpy(matI_KH, I4, 4, 4);
	ins_matmul("NN", 4, 4, 2, -1.0, Kk, m_H, 1.0, matI_KH);

	// m_P = matI_KH * m_P * matI_KH.GetTranspose() + Kk * m_R *Kk.GetTranspose();
	insMat_Colmjr_Transpose(matI_KH, 4, 4, matI_KH_T);

	ins_matmul("NN", 4, 4, 4, 1.0, m_P, matI_KH_T, 0.0, newP);
	ins_matmul("NN", 4, 4, 4, 1.0, matI_KH, newP, 0.0, newP2);

	ins_matmul("NN", 4, 2, 2, 1.0, Kk, m_R, 0.0, matTemp);
	ins_matmul("NN", 4, 4, 2, 1.0, matTemp, KkT, 1.0, newP2);

	// update matrix P
	ins_matcpy(m_P, newP2, 4, 4);

}

static void ins_kalman_MeasureUpdate_AttiBias(double measure_dV[], double std_dve, double std_dvn)
{
	double Kk[4*2];
	double KkT[2*4];
	double Z[2*1];
	//CMatrix I(sI4, 4, 4);
	double I2[2*2];
	double I4[4*4];

	double m_H[2*4];
	double m_HT[4*2];
	double m_R[2*2];

	double matTemp[4*2];
	double matTemp2[2*2];
	double matTemp2_inv[2*2];

	double matI_KH[4*4];
	double matI_KH_T[4*4];

	//double m_X[4*1];

	double newP[4*4];
	double newP2[4*4];


	insMat_Rowmjr_to_Colmjr(sH_Pos_2_4, 2, 4, m_H);
	insMat_Colmjr_Transpose(m_H, 2, 4, m_HT);

	insMat_Rowmjr_to_Colmjr(sR_Pos, 2, 2, m_R);
	insMat_Rowmjr_to_Colmjr(sI2, 2, 2, I2);
	insMat_Rowmjr_to_Colmjr(sI4, 4, 4, I4);

	//
	m_R[0] = std_dve*std_dve;
	m_R[3] = std_dvn*std_dvn;

	Z[0] = measure_dV[0];
	Z[1] = measure_dV[1];

	// matTemp = m_P *m_H.GetTranspose()
	// matTemp2 = m_H *m_P *m_H.GetTranspose() + m_R
	ins_matmul("NN", 4, 2, 4, 1.0, m_P, m_HT, 0.0, matTemp);
	ins_matmul("NN", 2, 2, 4, 1.0, m_H, matTemp, 0.0, matTemp2);
	ins_matmul("NN", 2, 2, 2, 1.0, I2, m_R, 1.0, matTemp2);

	// 
	ins_matcpy(matTemp2_inv, matTemp2, 2, 2);
	ins_matinv(matTemp2_inv, 2);

	// Kk = m_P *m_H.GetTranspose() * tmpMat.GetInverse();
	ins_matmul("NN", 4, 2, 2, 1.0, matTemp, matTemp2_inv, 0.0, Kk);

	insMat_Colmjr_Transpose(Kk, 4, 2, KkT);

	// update state vector X
	// mX = Kk * Z;
	ins_matmul("NN", 4, 1, 2, 1.0, Kk, Z, 0.0, m_X);

	// matI_KH = I - Kk *m_H;
	ins_matcpy(matI_KH, I4, 4, 4);
	ins_matmul("NN", 4, 4, 2, -1.0, Kk, m_H, 1.0, matI_KH);

	// m_P = matI_KH * m_P * matI_KH.GetTranspose() + Kk * m_R *Kk.GetTranspose();
	insMat_Colmjr_Transpose(matI_KH, 4, 4, matI_KH_T);

	ins_matmul("NN", 4, 4, 4, 1.0, m_P, matI_KH_T, 0.0, newP);
	ins_matmul("NN", 4, 4, 4, 1.0, matI_KH, newP, 0.0, newP2);

	ins_matmul("NN", 4, 2, 2, 1.0, Kk, m_R, 0.0, matTemp);
	ins_matmul("NN", 4, 4, 2, 1.0, matTemp, KkT, 1.0, newP2);

	// update matrix P
	ins_matcpy(m_P, newP2, 4, 4);
}