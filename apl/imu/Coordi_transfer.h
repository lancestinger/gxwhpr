#ifndef Coordi_transfer__h
#define Coordi_transfer__h

#include "ins_baseparams.h"


extern int FIRST_ANGLE_GET;
extern int VELOC_VALID;

typedef struct
{
	double Sum;
	double Mean;
	double Var;
	double Std;
	int   NUM;
	double Data;

}Nav_std_sum;

typedef struct
{
	double loc_E;
	double loc_N;
	float  loc_U;
	
}Pre_locat;

typedef struct
{
	double vel_E[2];
	double vel_N[2];
	double vel_U[2];
	
}Pre_veloc;


//WGS-84坐标系
typedef struct tagWGS
{
double latitude;
double longitude;
double height;
}WGS;
typedef WGS *PWGS;

//空间笛卡尔坐标系 ECEF
typedef struct tagECEF
{
double x;
double y;
double z;
}ECEF;
typedef ECEF *PECEF;

//东北天坐标系ENU
typedef struct tagENU
{
double northing;
double easting;
double upping;
}ENU;
typedef ENU *PENU;


//GPS采集到的经纬度数据无法直接使用，需要先转换
extern double transfer(double val);

//WGS84 ---> ECEF
//pcg为WGS-84坐标系结构体指针，pcc为ECEF坐标系结构体指针
extern void WGSToECEF(PWGS pcg, PECEF pcc);

//ECEF ---> WGS84
//pcg为WGS-84坐标系结构体指针，pcc为ECEF坐标系结构体指针
extern void ECEFToWGS(PWGS pcg, PECEF pcc);

//ECEF ---> ENU
//pcc为ECEF坐标系结构体指针，pccCenter为东北天坐标原点的指针，pct为东北天坐标系结构体指针
extern void ECEFToENU(PECEF pcc, PWGS center, PENU pct);

//各结构体初始化
extern void Coordi_initial(void);

//标准差递归计算函数
extern int Std_Devia(Nav_std_sum *Val);

//坐标与速度标准差计算整体调用函数
extern int Std_Manage(ins_geopos_t *Nav_locat, ins_vel_t *Nav_vel);

//坐标延迟反推与坐标转换函数
extern int Lat_Lon_ext(uint32_t time, ins_geopos_t *Nav_locat, ins_vel_t *Nav_vel);

extern void ECEF_ENU_WGS_TEST(void);
#endif
