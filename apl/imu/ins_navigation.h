#ifndef __INS_NAVIGATION_H__
#define __INS_NAVIGATION_H__


#ifdef __cplusplus
extern "C" {
#endif


#include "ins_baseparams.h"


void ins_reset();

void ins_set_ag_bias(float acceBias[], float gyroBias[]);

void insNaviUpdate(int8 locTime, imu_rawdata_t acce[], int n1, imu_rawdata_t mag[], int n2, imu_rawdata_t gyro[], int n3);

void insNaviFusion(int8 locTime, const ins_geopos_t* pPos, const ins_vel_t* pVel, const float* pHeading);

void CalibVelDir();

void ins_getFusionRslt(ins_geopos_t* pPos, ins_vel_t* pVel, ins_atti_t* pAtti);

void ins_getMagAttitude(ins_atti_t* pAtti);

#ifdef __cplusplus
}
#endif

#endif