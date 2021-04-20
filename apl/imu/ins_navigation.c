#include "ins_navigation.h"
#include "ins_gravity.h"
#include "ins_attitude.h"
#include "ins_matrix.h"
#include "ins_kalman.h"
#include "comm/project_def.h"


#include <string.h>
#include <stdio.h>
#include <math.h>


//static int m_InsInited;
static int m_PosVelInited;
static int8 m_LastNaviTime;
static ins_geopos_t m_Position;
static ins_vel_t    m_Velocity;
static ins_atti_t   m_Attitude;

static void ResampleImuData(imu_rawdata_t* pImuRawData, int cnt, int8 locTime, imu_data_t veImuData[]);
static void GetMN(double latitude, double* M, double* N);
static void CalcOmega_ie_inL(double latitude, double Omega_ie_inL[]);
static void CalcOmage_eL_inL(double latitude, double H, double ve, double vn, double Omage_eL_inL[]);
static void GetAntisymmetricMatrix(double omage[], double mat[]);



void ins_reset()
{
	//m_PosInited = false;
	//m_VelInited = false;
	//m_InsInited = false;
	m_PosVelInited = false;

	m_LastNaviTime = 0;

	m_Position.h = 0;
	m_Velocity.ve = m_Velocity.vn = m_Velocity.vu = 0;

	reset_atti_status();
}

// set acce and gyro bias
void ins_set_ag_bias(float acceBias[], float gyroBias[])
{
	set_ag_bias(acceBias, gyroBias);
}


void ins_getFusionRslt(ins_geopos_t* pPos, ins_vel_t* pVel, ins_atti_t* pAtti)
{
	GLOBAL_MEMCPY(pPos, &m_Position, sizeof(ins_geopos_t));
	GLOBAL_MEMCPY(pVel, &m_Velocity, sizeof(ins_vel_t));

	ins_GetAttitude(pAtti);

}


/*
* 
*/
void insNaviUpdate(int8 locTime, imu_rawdata_t acce[], int n1, imu_rawdata_t mag[], int n2, imu_rawdata_t gyro[], int n3)
{
	static imu_data_t acceDatas[SENSOR_SAMPLE_RATE];
	static imu_data_t magDatas[SENSOR_SAMPLE_RATE];
	static imu_data_t gyroDatas[SENSOR_SAMPLE_RATE];
	imu_data_t acceData, magData, gyroData;

	float dt = 1.0/SENSOR_SAMPLE_RATE;
	float acceBias[3];
	float gyroBias[3];
	double veAcceb[3];
	double veAcceL[3];
	double veAcceL_fromb[3];
	double dVe, dVn;
	double dE, dN;

	double g_inL[3];
	double Omega_ie_inL[3];
	double Omega_eL_inL[3];

	double latitude, longitude, H;
	double ve, vn, vu;
	double vel[3];
	double vL[3];
	double M, N;
	int i;

	// matrix
	double Rb2L[9];
	double matRotate_ie_inL[9];
	double matRotate_eL_inL[9];
	//double matTemp[9];

	double Rb2LColMjr[9];
	double matRotate_ie_inLColMjr[9];
	double matRotate_eL_inLColMjr[9];	
	double matTempColMjr[9];

	double matI[] = {1,0,0, 0,1,0, 0,0,1};

	ins_atti_t atti1, atti2, atti3;

	GLOBAL_MEMSET(acceDatas,0x0,sizeof(imu_data_t)*SENSOR_SAMPLE_RATE);
	GLOBAL_MEMSET(magDatas,0x0,sizeof(imu_data_t)*SENSOR_SAMPLE_RATE);
	GLOBAL_MEMSET(gyroDatas,0x0,sizeof(imu_data_t)*SENSOR_SAMPLE_RATE);

	if (0 == locTime)
	{
		return;
	}

	if ((!attiInited()) && (n1<SENSOR_SAMPLE_RATE || n2<SENSOR_SAMPLE_RATE || n3<SENSOR_SAMPLE_RATE) )
	{
		return;
	}

	get_ag_bias(acceBias, gyroBias);

	ResampleImuData(acce, n1, locTime, acceDatas);
	ResampleImuData(mag, n2, locTime, magDatas);
	ResampleImuData(gyro, n3, locTime, gyroDatas);

	if (!attiInited())
	{
		/*
		* 姿态还没初始化，那就进行初始对准
		*/
		ins_InitAlignment(acceDatas, magDatas);

	}
	else
	{
		/*
		* 姿态已经初始化，看位置速度是否已经初始化
		*/
		if (!m_PosVelInited)
		{
			/* 
			* 位置速度还没初始化，那就更新姿态
			*/
			ins_AttiUpdateSec(gyroDatas);

			ins_AttiClibBySnsr(acceDatas, magDatas);

		}
		else
		{
			/*
			* 位置已经初始化，进行INS导航计算
			*/
			for (i=0; i<SENSOR_SAMPLE_RATE; i++)
			{
				acceData = acceDatas[i];
				magData = magDatas[i];
				gyroData = gyroDatas[i];

				latitude = m_Position.lat;
				longitude = m_Position.lng;
				H = m_Position.h;
				ve = m_Velocity.ve;
				vn = m_Velocity.vn;
				vu = m_Velocity.vu;
				vel[0] = ve;
				vel[1] = vn;
				vel[2] = vu;
				vL[0] = ve;
				vL[1] = vn;
				vL[2] = vu;

				//vL.InitVector(vel, 3);

				GetMN(latitude, &M, &N);

				ins_AttitUpdate(&gyroData, dt);
				ins_AttiGetRb2L(Rb2L);

				ins_GetGravityInL(latitude, H, g_inL);

				CalcOmega_ie_inL(latitude, Omega_ie_inL);
				GetAntisymmetricMatrix(Omega_ie_inL, matRotate_ie_inL);

				CalcOmage_eL_inL(latitude, H, ve, vn, Omega_eL_inL);
				GetAntisymmetricMatrix(Omega_eL_inL, matRotate_eL_inL);

				veAcceb[0] = acceData.x - acceBias[0];
				veAcceb[1] = acceData.y - acceBias[1];
				veAcceb[2] = acceData.z - acceBias[2];

				insMat_Rowmjr_to_Colmjr(Rb2L, 3, 3, Rb2LColMjr);
				insMat_Rowmjr_to_Colmjr(matRotate_ie_inL, 3, 3, matRotate_ie_inLColMjr);
				insMat_Rowmjr_to_Colmjr(matRotate_eL_inL, 3, 3, matRotate_eL_inLColMjr);

				// matTemp = matRotate_ie_inL * 2 + matRotate_eL_inL;
				ins_matcpy(matTempColMjr, matRotate_eL_inLColMjr, 3, 3);
				ins_matmul("NN", 3, 3, 3, 2.0, matRotate_ie_inLColMjr, matI, 1.0, matTempColMjr);

				// veAcceL_fromb = Rb2L * veAcceb;
				ins_matmul("NN", 3, 1, 3, 1.0, Rb2LColMjr, veAcceb, 0.0, veAcceL_fromb);

				//veAcceL = veAcceL_fromb - matTemp * vL + g_inL;
				ins_matmul("NN", 3, 1, 3, 1.0, matI, veAcceL_fromb, 0.0, veAcceL);
				ins_matmul("NN", 3, 1, 3, -1.0, matTempColMjr, vL, 1.0, veAcceL);
				ins_matmul("NN", 3, 1, 3, 1.0, matI, g_inL, 1.0, veAcceL);

				// 
				dVe = veAcceL[0] * dt;
				dVn = veAcceL[1] * dt;

				//printf("%.3f, %.3f\n", veAcceL(0), veAcceL(1));

				dE = (m_Velocity.ve + dVe/2) * dt;
				dN = (m_Velocity.vn + dVn/2) * dt;

				m_Velocity.ve += dVe;
				m_Velocity.vn += dVn;

				m_Position.lat += dN / (M + H) * RAD2DEG;
				m_Position.lng += dE / ((N + H) * cos(latitude * DEG2RAD)) * RAD2DEG;

			}

			ins_AttiClibBySnsr(acceDatas, magDatas);

		}

	}


	//m_pAttitudeMng.AttitudeClibrateBySensor(acceDatas, magDatas);


	//navRsltPos = m_Position;
	//navRsltVel = m_Velocity;
	//m_pAttitudeMng.GetAttitude(&navRsltAtti);

}

/*
*  pPos != NULL 表示有定位结果输入。
*  pVel != NULL 表示有速度值输入。
*  pHeading != NULL 表示有方向角输入。
*/
void insNaviFusion(int8 locTime, const ins_geopos_t* pPos, const ins_vel_t* pVel, const float* pHeading)
{
	double dt;

	ins_atti_t atti3;

	if (0 == locTime)
	{
		return;
	}

	if (!m_PosVelInited || !attiInited())
	{
		if (NULL==pPos || NULL==pVel)
		{
			// do nothing;
		}
		else
		{
			GLOBAL_MEMCPY(&m_Position, pPos, sizeof(ins_geopos_t));
			GLOBAL_MEMCPY(&m_Velocity, pVel, sizeof(ins_vel_t));

			ins_kalman_init();

			m_LastNaviTime = locTime;

			m_PosVelInited = true;
		}

	}
	else
	{
		dt = (locTime - m_LastNaviTime) / 1000.0;
		if (dt > 1.0e3)
		{
			dt = 1.0;
		}

		ins_kalman_TimeUpdate(dt);

		ins_kalman_MeasureUpdate(pPos, pVel, &m_Position, &m_Velocity);

		if (NULL != pPos)
		{
			m_Position.h = pPos->h;
		}

		if (NULL != pVel)
		{
			m_Velocity.vu = pVel->vu;
		}
		
		if (locTime != 0)
		{
			m_LastNaviTime = locTime;
		}
		
	}

	if (attiInited())
	{
		if (NULL != pHeading)
		{
			ins_HeadingCalibOuter(*pHeading);
		}
	}

}


static void ResampleImuData(imu_rawdata_t* pImuRawData, int cnt, int8 locTime, imu_data_t veImuData[])
{
	int8 interval = 1000 / SENSOR_SAMPLE_RATE;
	int dataCnt = SENSOR_SAMPLE_RATE;
	int8 startTime = locTime - 1000 + 1000 / SENSOR_SAMPLE_RATE;
	
	int rawDataCnt = cnt;
	int startIndx = 0;
	int endIndx = rawDataCnt -1;
	imu_data_t imuData;
	int indx, indx2;
	int findIndx;
	double dt;
	float dx, dy, dz;
	int i;
	
	for (i=0; i<dataCnt; )
	{
		imuData.t = startTime + i * interval;

		if (startIndx >= endIndx)
		{
			indx = endIndx;
			imuData.x = pImuRawData[indx].x;
			imuData.y = pImuRawData[indx].y;
			imuData.z = pImuRawData[indx].z;
			
			veImuData[i] = imuData;

			i++;

			continue;
		}

		indx = startIndx;
		if (imuData.t < pImuRawData[indx].t)
		{
			imuData.x = pImuRawData[indx].x;
			imuData.y = pImuRawData[indx].y;
			imuData.z = pImuRawData[indx].z;
			
			veImuData[i] = imuData;

			i++;

			continue;
		}

		findIndx = false;
		while (startIndx < endIndx)
		{
			indx2 = (startIndx + 1);
			if (imuData.t < pImuRawData[indx2].t)
			{
				findIndx = true;
				break;
			}
			else
			{
				startIndx++;
			}
		}

		if (findIndx)
		{
			indx = startIndx;
			dt = (double)(pImuRawData[indx2].t - pImuRawData[indx].t);
			if (fabs(dt) < 0.1)
			{
				imuData.x = (pImuRawData[indx2].x + pImuRawData[indx].x) / 2;
				imuData.y = (pImuRawData[indx2].y + pImuRawData[indx].y) / 2;
				imuData.z = (pImuRawData[indx2].z + pImuRawData[indx].z) / 2;
			}
			else
			{
				dx = pImuRawData[indx2].x - pImuRawData[indx].x;
				dy = pImuRawData[indx2].y - pImuRawData[indx].y;
				dz = pImuRawData[indx2].z - pImuRawData[indx].z;

				imuData.x = pImuRawData[indx].x + (double)(imuData.t - pImuRawData[indx].t) * dx / dt;
				imuData.y = pImuRawData[indx].y + (double)(imuData.t - pImuRawData[indx].t) * dy / dt;
				imuData.z = pImuRawData[indx].z + (double)(imuData.t - pImuRawData[indx].t) * dz / dt;
			}

			veImuData[i] = imuData;

			i++;
		}

	}

	//return true;
}


// get the radius of meridian and prime vertical
static void GetMN(double latitude, double* M, double* N)
{
	double latitudeRadian = latitude * DEG2RAD;
	double W = sqrt(1 - WGS84_e2 * sin(latitudeRadian) * sin(latitudeRadian));
	*M = WGS84_a * (1 - WGS84_e2) / (W * W * W); 
	*N = WGS84_a / W;	
}

static void CalcOmega_ie_inL(double latitude, double Omega_ie_inL[])
{
	Omega_ie_inL[0] = 0;
	Omega_ie_inL[1] = WGS84_AngleRate * cos(latitude * DEG2RAD);
	Omega_ie_inL[2] = WGS84_AngleRate * sin(latitude * DEG2RAD);
}

static void CalcOmage_eL_inL(double latitude, double H, double ve, double vn, double Omage_eL_inL[])
{
	double M, N;

	GetMN(latitude, &M, &N);

	Omage_eL_inL[0] = - vn / (M + H);
	Omage_eL_inL[1] = ve / (N + H);
	Omage_eL_inL[2] = ve * tan(latitude*DEG2RAD) / (N + H);
}

static void GetAntisymmetricMatrix(double omage[], double mat[])
{
	mat[0] = 0;
	mat[1] = -omage[2];
	mat[2] = omage[1];

	mat[3] = omage[2];
	mat[4] = 0;
	mat[5] = - omage[0];

	mat[6] = - omage[1];
	mat[7] = omage[0];
	mat[8] = 0;
}
