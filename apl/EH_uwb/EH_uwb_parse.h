#ifndef _UWB_PARSE_H_
#define _UWB_PARSE_H_

#include "inttypes.h"
#include "Matrix_functions.h"
#define MAX_AP 10
#define High 0.8

//�����ղ���
typedef struct
{
	double B;		//γ��
	double L;		//����
	float H;		//�߶�
}PointBLH;

typedef struct
{
	float B;		//γ��
	float L;		//����
}PointBL;

typedef struct
{
	float lat;		//γ��
	float lon;		//����
}POS;

typedef struct
{
	float x;		
	float y;		
	float z;		
}Vector3;

typedef struct
{
	float x;		
	float y;				
}Vector2;

typedef struct
{
	uint16_t addr;
	uint16_t dist;
	uint8_t deg;
	int8_t RSSI;
}UWBbaseinfo;

typedef struct
{
	uint32_t id;
	uint32_t top;
	uint8_t  num;
	UWBbaseinfo info[MAX_AP];
}UWBdata;

typedef struct
{
	uint32_t head;
	uint16_t cmd;
	uint16_t reserved;
	uint32_t length;
	UWBdata data;
	uint8_t proof;	
}UWBagree;

typedef struct
{
	uint16_t addr;
	Vector3 zxy;	
}apdist;

typedef struct{
	UWBagree uwbdata;
	Vector3 ap[4];
	Vector3 tag;
}UWBParse;

extern apdist aplist[];
extern uint8_t Drop_Sign;
extern uint16_t Drop_Num;


void XYZToBLH(PointBLH *des , const Vector3 src);
void BLHToXYZ(Vector3 *des,const PointBLH src);
int CalculateTriangleThirdPoint(const Vector2 vA, const Vector2 vB, Vector2 *vC, double  a, double  b, double  c );
Vector2 * getFootPoint(Vector2 point, Vector2 pnt1, Vector2 pnt2, Vector2 *fpoint);
Vector2 *BLToxy(PointBL bL,Vector2 *xy,double L0);
PointBL *xyToBL(Vector2 xy,PointBL *BL,double L0);
int uwb_uart_parse(UWBagree * uwbdata,const char *data);
void nmea_BL2pos(const PointBL *info, POS *pos);
double Center_longitude(double longitude);
double Distance(Vector2 pnt1,Vector2 pnt2);

int aplistparse(UWBagree uwbagree,Vector3 *ap,float *matrix_x,float *matrix_y);
float Station_Z_Sort(float* coord_x, uint8_t stat_num);
void Uwb_Dist_VARC(uint16_t dist);


//void uwb_run(void);
void uwb_run(void);

#endif
