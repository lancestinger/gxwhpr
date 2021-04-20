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


//WGS-84����ϵ
typedef struct tagWGS
{
double latitude;
double longitude;
double height;
}WGS;
typedef WGS *PWGS;

//�ռ�ѿ�������ϵ ECEF
typedef struct tagECEF
{
double x;
double y;
double z;
}ECEF;
typedef ECEF *PECEF;

//����������ϵENU
typedef struct tagENU
{
double northing;
double easting;
double upping;
}ENU;
typedef ENU *PENU;


//GPS�ɼ����ľ�γ�������޷�ֱ��ʹ�ã���Ҫ��ת��
extern double transfer(double val);

//WGS84 ---> ECEF
//pcgΪWGS-84����ϵ�ṹ��ָ�룬pccΪECEF����ϵ�ṹ��ָ��
extern void WGSToECEF(PWGS pcg, PECEF pcc);

//ECEF ---> WGS84
//pcgΪWGS-84����ϵ�ṹ��ָ�룬pccΪECEF����ϵ�ṹ��ָ��
extern void ECEFToWGS(PWGS pcg, PECEF pcc);

//ECEF ---> ENU
//pccΪECEF����ϵ�ṹ��ָ�룬pccCenterΪ����������ԭ���ָ�룬pctΪ����������ϵ�ṹ��ָ��
extern void ECEFToENU(PECEF pcc, PWGS center, PENU pct);

//���ṹ���ʼ��
extern void Coordi_initial(void);

//��׼��ݹ���㺯��
extern int Std_Devia(Nav_std_sum *Val);

//�������ٶȱ�׼�����������ú���
extern int Std_Manage(ins_geopos_t *Nav_locat, ins_vel_t *Nav_vel);

//�����ӳٷ���������ת������
extern int Lat_Lon_ext(uint32_t time, ins_geopos_t *Nav_locat, ins_vel_t *Nav_vel);

extern void ECEF_ENU_WGS_TEST(void);
#endif
