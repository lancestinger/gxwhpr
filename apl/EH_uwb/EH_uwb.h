#ifndef __UWB_H__
#define __UWB_H__

#include "EH_uwb_parse.h"
#include "apl/imu/Coordi_transfer.h"


#define filterNUM 5

extern uint8_t UWB_SWICH;
extern uint8_t Least_fail;
extern WGS ORIGN_eh;
extern ECEF ORIGN_eh_ECEF;

void uwb_run(void);
void uwb_parse_run(void);

typedef struct{
	UWBParse uwbParse;
}uwbStruct;

typedef struct{
	int nmeaParse;
}ubxStruct;

typedef struct{
	uint8_t type;
	uwbStruct *uwb;
	ubxStruct *ubx;
}netStruct;

typedef struct{
	int a;
}memsStruct;


typedef struct
{
	float dist;	
	int flag; 
}Vectinfo;

typedef struct
{
	float buf[DNUM];
}NumBuf;

typedef struct
{
	NumBuf buf[filterNUM];
	int in;
	int out;
}NumRING;

typedef enum
{
    EH_UWB_INIT        = 0,            /* EH_UWB初始化 */
    EH_UWB_CONNECT     = 1,            /* EH_UWB连接服务器 */
    EH_UWB_IDEL        = 2,            /* EH_UWB空闲 */
}EH_UWB_WorkState_t;

extern Vectinfo  uwbinfo;

void EH_uwb_apl_init(void);

	
#endif
