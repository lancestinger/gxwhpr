#ifndef _IMU_H_
#define _IMU_H_

#include "ins_baseparams.h"


#define IMU_DATA_MAX_NUM    50

#define NORMAL_G            9.80665

//惯导初始化自校准结构体
typedef struct
{
	float SUM_Accel_x;
	float SUM_Accel_y;
	float SUM_Accel_z;
	int Num_Accel;

	float SUM_Gy_x;
	float SUM_Gy_y;
	float SUM_Gy_z;
	int Num_Gy;

	
}Self_calib;


extern ins_geopos_t Nav_location;
extern ins_vel_t Nav_veloc;
extern ins_geopos_t Out_location;
extern ins_vel_t Out_veloc;
extern int First_flag;

extern void Imu_apl_init(void);
extern void IN_OUT_Init(void);
extern void Self_cali_Init(void);
extern void Imu_Acce_data_get(void);
extern void Imu_Gy_data_get(void);
extern void Imu_Mag_data_get(void);
extern void Imu_Time_Get(uint16_t GPIO_Pin);



#endif

