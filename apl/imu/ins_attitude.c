#include "ins_attitude.h"
#include "comm/project_def.h"

#include <math.h>
#include <string.h>


#define Magnetic_Deviation_Angle    (-5.6)  // default value in china, in deg.

static int m_bInited;
static float m_AcceBias[3];
static float m_GyroBias[3];
static double m_Rb2L[9];
static cQuaternion m_Quat; 

static void CalAcceMagMeanValue(imu_data_t AcceData[], imu_data_t MagData[], imu_data_t *pAcceMean, imu_data_t *pMagMean);
static int GetRb2L_From_AcceMagVector(imu_data_t AcceData, imu_data_t MagData, double fRb2L[]);
//static int GetAttitude_From_AcceMag(imu_data_t AcceData[], imu_data_t MagData[], ins_atti_t* pAtti);
static void getQ_From_Rb2L(double Rb2L[], cQuaternion* Q);
static void GetRb2L_From_Q(const cQuaternion* pQuart, double Rb2L[]);
static void GetRb2L_From_Attitude(ins_atti_t Atti, double Rb2L[]);
static void GetAttitude_From_Rb2L(double Rb2L[], ins_atti_t* pAtti);
static void AttitudeCalibFromMag(int epoch, ins_atti_t* pMagAtti);


int attiInited()
{
	return m_bInited;
}

void reset_atti_status()
{
	int i;

	for (i=0; i<3; i++)
	{
		m_AcceBias[i] = 0;
		m_GyroBias[i] = 0;
	}

	m_bInited = false;
}

void set_ag_bias(float acceBias[], float gyroBias[])
{
	int i;

	for (i=0; i<3; i++)
	{
		m_AcceBias[i] = acceBias[i];
		m_GyroBias[i] = gyroBias[i];
	}
}

void get_ag_bias(float acceBias[], float gyroBias[])
{
	int i;

	for (i=0; i<3; i++)
	{
		acceBias[i] = m_AcceBias[i];
		gyroBias[i] = m_GyroBias[i];
	}
}

void ins_GetAttitude(ins_atti_t *pAtti)
{
	GetRb2L_From_Q(&m_Quat, m_Rb2L);
	GetAttitude_From_Rb2L(m_Rb2L, pAtti);
}


int ins_InitAlignment(imu_data_t AcceData[], imu_data_t MagData[])
{
	imu_data_t acceMean;
	imu_data_t magMean;
	ins_atti_t Atti;
	
	CalAcceMagMeanValue(AcceData, MagData, &acceMean, &magMean);

	if (!GetRb2L_From_AcceMagVector(acceMean, magMean, m_Rb2L))
	{
		return false;
	}

	GetAttitude_From_Rb2L(m_Rb2L, &Atti);
	Atti.y = (float)( Atti.y - Magnetic_Deviation_Angle * DEG2RAD );
	GetRb2L_From_Attitude(Atti, m_Rb2L);
	
	getQ_From_Rb2L(m_Rb2L, &m_Quat);
	//memcpy(&m_Quat_Pre, &m_Quat, sizeof(m_Quat));

	m_bInited = true;

	return true;
}

void CalAcceMagMeanValue(imu_data_t AcceData[], imu_data_t MagData[], imu_data_t *pAcceMean, imu_data_t *pMagMean)
{
	int count = SENSOR_SAMPLE_RATE;
	float tAcce[3];
	float tMag[3];
	int ti, tj;

	for (ti = 0; ti < 3; ti++) 
	{
		tAcce[ti] = 0;
		tMag[ti] = 0;
	}

	for (tj = 0; tj < count; tj++) 
	{
		tAcce[0] += AcceData[tj].x;
		tAcce[1] += AcceData[tj].y;
		tAcce[2] += AcceData[tj].z;

		tMag[0] += MagData[tj].x;
		tMag[1] += MagData[tj].y;
		tMag[2] += MagData[tj].z;
	}

	pAcceMean->x = tAcce[0] /count;
	pAcceMean->y = tAcce[1] /count;
	pAcceMean->z = tAcce[2] /count;
	pMagMean->x = tMag[0] /count;
	pMagMean->y = tMag[1] /count;
	pMagMean->z = tMag[2] /count;

}

int GetRb2L_From_AcceMagVector(imu_data_t AcceData, imu_data_t MagData, double fRb2L[])
{
	double Ax = AcceData.x - m_AcceBias[0];
	double Ay = AcceData.y - m_AcceBias[1];
	double Az = AcceData.z - m_AcceBias[2];
	double Ex = MagData.x;
	double Ey = MagData.y;
	double Ez = MagData.z;

	double Hx = Ey*Az - Ez*Ay;
	double Hy = Ez*Ax - Ex*Az;
	double Hz = Ex*Ay - Ey*Ax;
	double normH, invH, normA, invA;
	double Mx, My, Mz;

	normH = sqrt(Hx*Hx + Hy*Hy + Hz*Hz);
	if (normH < 0.1) {
		// device is close to free fall (or in space?), or close to
		// magnetic north pole. Typical values are > 100.
		return false;
	}
	invH = (1.0 / normH );
	Hx *= invH;
	Hy *= invH;
	Hz *= invH;
    
	normA = sqrt(Ax*Ax + Ay*Ay + Az*Az);
	if (normA < 0.1) {
		return false;
	}
	invA = ( 1.0 /normA );
	Ax *= invA;
	Ay *= invA;
	Az *= invA;

	Mx = Ay*Hz - Az*Hy;
	My = Az*Hx - Ax*Hz;
	Mz = Ax*Hy - Ay*Hx;

	//////////////////////////////////////////////////////////////////////////
	// R = Rb2L
	//	 R[0] = Hx;     R[1] = Hy;     R[2] = Hz;
	//	 R[3] = Mx;     R[4] = My;     R[5] = Mz;
	//	 R[6] = Ax;     R[7] = Ay;     R[8] = Az;
	//////////////////////////////////////////////////////////////////////////

	//	 	fpAtti->y = (FLT)atan2(Hy, My);
	//	 	fpAtti->p = (FLT)asin(Ay);
	//	 	fpAtti->r = (FLT)atan2(-Ax, Az);

	if (NULL != fRb2L)
	{	 
		fRb2L[0] = Hx;     
		fRb2L[1] = Hy;     
		fRb2L[2] = Hz; 
		fRb2L[3] = Mx;     
		fRb2L[4] = My;     
		fRb2L[5] = Mz;
		fRb2L[6] = Ax;     
		fRb2L[7] = Ay;     
		fRb2L[8] = Az;
	}

	return true;

}


int GetAttitude_From_AcceMag(imu_data_t AcceData[], imu_data_t MagData[], ins_atti_t* pAtti)
{
	imu_data_t acceMean;
	imu_data_t magMean;
	//double Rb2L[9];

	CalAcceMagMeanValue(AcceData, MagData, &acceMean, &magMean);

	if (!GetRb2L_From_AcceMagVector(acceMean, magMean, m_Rb2L))
	{
		return false;
	}

	GetAttitude_From_Rb2L(m_Rb2L, pAtti);
	pAtti->y = (float)( pAtti->y - Magnetic_Deviation_Angle * DEG2RAD );

	return true;
}

void getQ_From_Rb2L(double Rb2L[], cQuaternion* Q)
{
	double tF;
	double q_Norm;

	tF = 1 + Rb2L[0] + Rb2L[4] + Rb2L[8];
	if (tF > 0.0001)
	{
		Q->q0 = ( 0.5 * sqrt(tF) );
		Q->q1 = (Rb2L[7] - Rb2L[5]) / (4 * Q->q0);
		Q->q2 = (Rb2L[2] - Rb2L[6]) / (4 * Q->q0);
		Q->q3 = (Rb2L[3] - Rb2L[1]) / (4 * Q->q0);
	} 
	else if (Rb2L[0] > Rb2L[4] && Rb2L[0] > Rb2L[8])
	{
		tF = 1 + Rb2L[0] - Rb2L[4] - Rb2L[8];
		
		Q->q1 = ( 0.5 * sqrt(tF) );
		Q->q0 = (Rb2L[7] - Rb2L[5]) / (4 * Q->q1);
		Q->q2 = (Rb2L[1] + Rb2L[3]) / (4 * Q->q1);
		Q->q3 = (Rb2L[2] + Rb2L[6]) / (4 * Q->q1);
	}
	else if (Rb2L[4] > Rb2L[8])
	{
		tF = 1 + Rb2L[4] - Rb2L[0] - Rb2L[8];

		Q->q2 = ( 0.5 * sqrt(tF) );
		Q->q0 = (Rb2L[2] - Rb2L[6]) / (4 * Q->q2);
		Q->q1 = (Rb2L[1] + Rb2L[3]) / (4 * Q->q2);
		Q->q3 = (Rb2L[7] + Rb2L[5]) / (4 * Q->q2);
	}
	else
	{
		tF = 1 + Rb2L[8] - Rb2L[0] - Rb2L[4];

		Q->q3 = ( 0.5 * sqrt(tF) );
		Q->q0 = (Rb2L[3] - Rb2L[1]) / (4 * Q->q3);
		Q->q1 = (Rb2L[2] + Rb2L[6]) / (4 * Q->q3);
		Q->q2 = (Rb2L[7] + Rb2L[5]) / (4 * Q->q3);
	}

	q_Norm = sqrt(Q->q0 * Q->q0 + Q->q1 * Q->q1 + Q->q2 * Q->q2 + Q->q3 * Q->q3);
	Q->q0 = Q->q0 / q_Norm;
	Q->q1 = Q->q1 / q_Norm;
	Q->q2 = Q->q2 / q_Norm;
	Q->q3 = Q->q3 / q_Norm;

	return;
}

void GetRb2L_From_Q(const cQuaternion* pQuart, double Rb2L[])
{
	double q0 = pQuart->q0;
	double q1 = pQuart->q1;
	double q2 = pQuart->q2;
	double q3 = pQuart->q3;

	Rb2L[0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
	Rb2L[1] = 2 * (q1*q2 - q0*q3);
	Rb2L[2] = 2 * (q1*q3 + q0*q2);

	Rb2L[3] = 2 * (q1*q2 + q0*q3);
	Rb2L[4] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
	Rb2L[5] = 2 * (q2*q3 - q0*q1);

	Rb2L[6] = 2 * (q1*q3 - q0*q2);
	Rb2L[7] = 2 * (q2*q3 + q0*q1);
	Rb2L[8] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	return;
}

void ins_AttiGetRb2L(double Rb2L[])
{
	GetRb2L_From_Q(&m_Quat, Rb2L);
}

void GetRb2L_From_Attitude(ins_atti_t Atti, double Rb2L[])
{
	float cosr = (float)cos(Atti.r);
	float sinr = (float)sin(Atti.r);
	float cosp = (float)cos(Atti.p);
	float sinp = (float)sin(Atti.p);
	float cosy = (float)cos(Atti.y);
	float siny = (float)sin(Atti.y);

	Rb2L[0] = cosr*cosy - sinr*siny*sinp;
	Rb2L[1] = -siny*cosp;
	Rb2L[2] = sinr*cosy + cosr*siny*sinp;

	Rb2L[3] = cosr*siny + sinr*cosy*sinp;
	Rb2L[4] = cosy*cosp;
	Rb2L[5] = sinr*siny - cosr*cosy*sinp;

	Rb2L[6] = -sinr*cosp;
	Rb2L[7] = sinp;
	Rb2L[8] = cosr*cosp;

	return;
}

void GetAttitude_From_Rb2L(double Rb2L[], ins_atti_t* pAtti)
{
	//definition of r,p,y:
	//    from the view of positive direction of rotation axis, b-frame related to L-frame in counter-clockwise is positive.
	if (NULL != Rb2L)
	{
		pAtti->y = (float)atan2(-Rb2L[1], Rb2L[4]);
		pAtti->p = (float)asin(Rb2L[7]);
		pAtti->r = (float)atan2(-Rb2L[6], Rb2L[8]);
	}

	return;
}

void ins_AttiUpdateSec(imu_data_t GyroData[])
{
	int tCount = SENSOR_SAMPLE_RATE;
	float tDt = (float)( 1.0 /SENSOR_SAMPLE_RATE );
	float tD_Ang_x, tD_Ang_y, tD_Ang_z, tD_Ang;
	float ta, tb;
	cQuaternion tQ;
	float q_Norm;
	int ti;

	//memcpy(&m_Quat_Pre, &m_Quat, sizeof(m_Quat));
	memcpy(&tQ, &m_Quat, sizeof(m_Quat));

	for (ti = 0; ti < tCount; ti++)
	{
		tD_Ang_x = (GyroData[ti].x - m_GyroBias[0]) * tDt;
		tD_Ang_y = (GyroData[ti].y - m_GyroBias[1]) * tDt;
		tD_Ang_z = (GyroData[ti].z - m_GyroBias[2]) * tDt;
		tD_Ang = (float)sqrt(tD_Ang_x*tD_Ang_x + tD_Ang_y*tD_Ang_y + tD_Ang_z*tD_Ang_z);
		if (tD_Ang < 1.0e-6)
		{
			ta = (float)1.0;
			tb = (float)0.5;
		}
		else
		{
			ta = (float)cos(tD_Ang/2);
			tb = (float)(sin(tD_Ang/2) / tD_Ang);
		}

		m_Quat.q0 = ta* tQ.q0 - tb*tD_Ang_x*tQ.q1 - tb*tD_Ang_y*tQ.q2 - tb*tD_Ang_z*tQ.q3;
		m_Quat.q1 = ta* tQ.q1 + tb*tD_Ang_x*tQ.q0 + tb*tD_Ang_z*tQ.q2 - tb*tD_Ang_y*tQ.q3;
		m_Quat.q2 = ta* tQ.q2 + tb*tD_Ang_y*tQ.q0 - tb*tD_Ang_z*tQ.q1 + tb*tD_Ang_x*tQ.q3;
		m_Quat.q3 = ta* tQ.q3 + tb*tD_Ang_z*tQ.q0 + tb*tD_Ang_y*tQ.q1 - tb*tD_Ang_x*tQ.q2;

		memcpy(&tQ, &m_Quat, sizeof(m_Quat));
	}
	q_Norm = (float)sqrt(m_Quat.q0 * m_Quat.q0 + m_Quat.q1 * m_Quat.q1 + m_Quat.q2 * m_Quat.q2 + m_Quat.q3 * m_Quat.q3);
	m_Quat.q0 = m_Quat.q0 / q_Norm;
	m_Quat.q1 = m_Quat.q1 / q_Norm;
	m_Quat.q2 = m_Quat.q2 / q_Norm;
	m_Quat.q3 = m_Quat.q3 / q_Norm;			

	return;
}

void ins_AttitUpdate(const imu_data_t *pGyroData, float dt)
{
	float tD_Ang_x, tD_Ang_y, tD_Ang_z, tD_Ang;
	float ta, tb;
	cQuaternion tQ;
	float q_Norm;

	tD_Ang_x = (pGyroData->x - m_GyroBias[0]) * dt;
	tD_Ang_y = (pGyroData->y - m_GyroBias[1]) * dt;
	tD_Ang_z = (pGyroData->z - m_GyroBias[2]) * dt;
	tD_Ang = (float)sqrt(tD_Ang_x*tD_Ang_x + tD_Ang_y*tD_Ang_y + tD_Ang_z*tD_Ang_z);
	if (tD_Ang < 1.0e-6)
	{
		ta = (float)1.0;
		tb = (float)0.5;
	}
	else
	{
		ta = (float)cos(tD_Ang/2);
		tb = (float)(sin(tD_Ang/2) / tD_Ang);
	}

	memcpy(&tQ, &m_Quat, sizeof(m_Quat));
	m_Quat.q0 = ta* tQ.q0 - tb*tD_Ang_x*tQ.q1 - tb*tD_Ang_y*tQ.q2 - tb*tD_Ang_z*tQ.q3;
	m_Quat.q1 = ta* tQ.q1 + tb*tD_Ang_x*tQ.q0 + tb*tD_Ang_z*tQ.q2 - tb*tD_Ang_y*tQ.q3;
	m_Quat.q2 = ta* tQ.q2 + tb*tD_Ang_y*tQ.q0 - tb*tD_Ang_z*tQ.q1 + tb*tD_Ang_x*tQ.q3;
	m_Quat.q3 = ta* tQ.q3 + tb*tD_Ang_z*tQ.q0 + tb*tD_Ang_y*tQ.q1 - tb*tD_Ang_x*tQ.q2;

	q_Norm = (float)sqrt(m_Quat.q0 * m_Quat.q0 + m_Quat.q1 * m_Quat.q1 + m_Quat.q2 * m_Quat.q2 + m_Quat.q3 * m_Quat.q3);
	m_Quat.q0 = m_Quat.q0 / q_Norm;
	m_Quat.q1 = m_Quat.q1 / q_Norm;
	m_Quat.q2 = m_Quat.q2 / q_Norm;
	m_Quat.q3 = m_Quat.q3 / q_Norm;	
}

void ins_AttiClibBySnsr(int epoch, imu_data_t AcceData[], imu_data_t MagData[])
{
	ins_atti_t magAtti;
	
	GetAttitude_From_AcceMag(AcceData, MagData, &magAtti);
	AttitudeCalibFromMag(epoch, &magAtti);
}

void AttitudeCalibFromMag(int epoch, ins_atti_t* pMagAtti)
{
	//float PI = 3.14159;
	ins_atti_t GyroAtti;
	float AttiDiff[3];
	int i;
	double d_P_R;
	double d_Yaw;
	double fabs_attiDiff;

	int epoch_maxCnt = 120;

	double ratio_P_R_max = 0.1;
	double min_P_R_diff_max = 1.0 * DEG2RAD;
	double max_P_R_diff_max = 5.0 * DEG2RAD;

	double ratio_P_R_min = 0.02;
	double min_P_R_diff_min = 0.005 * DEG2RAD;
	double max_P_R_diff_min = 0.02 * DEG2RAD;

	double ratio_P_R;
	double min_P_R_diff;
	double max_P_R_diff;

	// yaw
	double ratio_Yaw = 0.01;
	double min_Yaw_diff = 0.005 * DEG2RAD;
	double max_Yaw_diff = 0.010 * DEG2RAD;

	ratio_P_R = (epoch_maxCnt - epoch) * ratio_P_R_max / epoch_maxCnt;
	if (ratio_P_R < ratio_P_R_min)
	{
		ratio_P_R = ratio_P_R_min;
	}
	min_P_R_diff = (epoch_maxCnt - epoch) * min_P_R_diff_max / epoch_maxCnt;
	if (min_P_R_diff < min_P_R_diff_min)
	{
		min_P_R_diff = min_P_R_diff_min;
	}
	max_P_R_diff = (epoch_maxCnt - epoch) * max_P_R_diff_max / epoch_maxCnt;
	if (max_P_R_diff < max_P_R_diff_min)
	{
		max_P_R_diff = max_P_R_diff_min;
	}

	GetRb2L_From_Q(&m_Quat, m_Rb2L);
	GetAttitude_From_Rb2L(m_Rb2L, &GyroAtti);

	AttiDiff[0] = pMagAtti->p - GyroAtti.p;
	AttiDiff[1] = pMagAtti->r - GyroAtti.r;
	AttiDiff[2] = pMagAtti->y - GyroAtti.y;

	for (i=0; i<3; i++)
	{
		while (AttiDiff[i] > PI) 
		{
			AttiDiff[i] -= 2* PI;
		}
		while (AttiDiff[i] < -PI)
		{
			AttiDiff[i] += 2* PI;
		}

	}
    
	// 1.1 pitch
	fabs_attiDiff = fabs(AttiDiff[0]);
    if (fabs_attiDiff < min_P_R_diff)
	{
		//
	}
	else
	{
		fabs_attiDiff = min_P_R_diff + (fabs_attiDiff - min_P_R_diff) * ratio_P_R;
	}

	if (fabs_attiDiff > max_P_R_diff)
	{
		fabs_attiDiff = max_P_R_diff;
	}

	GyroAtti.p += fabs_attiDiff * AttiDiff[0] / (fabs(AttiDiff[0]) + 1.0e-6);

	// 1.2 roll
	fabs_attiDiff = fabs(AttiDiff[1]);
	if (fabs_attiDiff < min_P_R_diff)
	{
		//
	}
	else
	{
		fabs_attiDiff = min_P_R_diff + (fabs_attiDiff - min_P_R_diff) * ratio_P_R;
	}

	if (fabs_attiDiff > max_P_R_diff)
	{
		fabs_attiDiff = max_P_R_diff;
	}

	GyroAtti.r += fabs_attiDiff * AttiDiff[1] / (fabs(AttiDiff[1]) + 1.0e-6);

	// 2. yaw
	fabs_attiDiff = fabs(AttiDiff[2]);
	if (fabs_attiDiff < min_Yaw_diff)
	{
		//
	}
	else
	{
		fabs_attiDiff = min_Yaw_diff + (fabs_attiDiff - min_Yaw_diff) * ratio_Yaw;
	}

	if (fabs_attiDiff > max_Yaw_diff)
	{
		fabs_attiDiff = max_Yaw_diff;
	}

	GyroAtti.y += fabs_attiDiff * AttiDiff[2] / (fabs(AttiDiff[2]) + 1.0e-6);

	GetRb2L_From_Attitude(GyroAtti, m_Rb2L);
	getQ_From_Rb2L(m_Rb2L, &m_Quat);

}


// heading : [0, 360), in clockwise
void ins_HeadingCalibOuter(int epoch, float heading)
{
	float yaw = -heading * DEG2RAD;

	ins_atti_t GyroAtti;
	double d_Yaw;
	double fabs_d_Yaw;

	int epoch_maxCnt = 120;

	double ratio_Yaw_max = 0.10;
	double min_Yaw_diff_max = 2.0 *DEG2RAD;
	double max_Yaw_diff_max = 10.0 * DEG2RAD;  // 0.25

	double ratio_Yaw_min = 0.025; // 0.05
	double min_Yaw_diff_min = 0.1 * DEG2RAD; // 0.25
	double max_Yaw_diff_min = 0.5 * DEG2RAD;  // 0.25

	double ratio_Yaw;
	double min_Yaw_diff;
	double max_Yaw_diff;

	ratio_Yaw = (epoch_maxCnt - epoch) * ratio_Yaw_max / epoch_maxCnt;
	if (ratio_Yaw < ratio_Yaw_min)
	{
		ratio_Yaw = ratio_Yaw_min;
	}
	min_Yaw_diff = (epoch_maxCnt - epoch) * min_Yaw_diff_max / epoch_maxCnt;
	if (min_Yaw_diff < min_Yaw_diff_min)
	{
		min_Yaw_diff = min_Yaw_diff_min;
	}
	max_Yaw_diff = (epoch_maxCnt - epoch) * max_Yaw_diff_max / epoch_maxCnt;
	if (max_Yaw_diff < max_Yaw_diff_min)
	{
		max_Yaw_diff = max_Yaw_diff_min;
	}

	GetRb2L_From_Q(&m_Quat, m_Rb2L);
	GetAttitude_From_Rb2L(m_Rb2L, &GyroAtti);

	d_Yaw = yaw - GyroAtti.y;

	while (d_Yaw > PI) 
	{
		d_Yaw -= 2*PI;									
	}
	while (d_Yaw < -PI) 
	{
		d_Yaw += 2*PI;									
	}

	fabs_d_Yaw = fabs(d_Yaw);

    if (fabs_d_Yaw < min_Yaw_diff)
	{
		//d_Yaw = d_Yaw;
	}
	else
	{
		fabs_d_Yaw = min_Yaw_diff + (fabs_d_Yaw - min_Yaw_diff) * ratio_Yaw;
	}

	if (fabs_d_Yaw > max_Yaw_diff)
	{
		fabs_d_Yaw = max_Yaw_diff;
	}

	GyroAtti.y += fabs_d_Yaw * d_Yaw / (fabs(d_Yaw) + 1.0e-6);

	GetRb2L_From_Attitude(GyroAtti, m_Rb2L);
	getQ_From_Rb2L(m_Rb2L, &m_Quat);

}