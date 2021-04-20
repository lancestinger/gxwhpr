#ifndef __INS_ATTITUDE_H__
#define __INS_ATTITUDE_H__


#ifdef __cplusplus
extern "C" {
#endif


#include "ins_baseparams.h"


int attiInited();

void reset_atti_status();

void set_ag_bias(float acceBias[], float gyroBias[]);

void get_ag_bias(float acceBias[], float gyroBias[]);

void ins_GetAttitude(ins_atti_t *pAtti);

int ins_InitAlignment(imu_data_t AcceData[], imu_data_t MagData[]);

void ins_AttiUpdateSec(imu_data_t GyroData[]);

void ins_AttitUpdate(const imu_data_t *pGyroData, float dt);

void ins_AttiGetRb2L(double Rb2L[]);

void ins_AttiClibBySnsr(imu_data_t AcceData[], imu_data_t MagData[]);

void ins_HeadingCalibOuter(float heading);



#ifdef __cplusplus
}
#endif

#endif

