#ifndef __NTRIP_H__
#define __NTRIP_H__

#include "comm/project_def.h"


#define MOUNT            "RTCM32_GGB"
#define USER_CODE        "cXh1dG11MDAyOmUxMWE3OTc="
#define NTRIP_CODE_LEN   300

typedef enum
{
    NTRIP_INIT        = 0,            /* NTRIP初始化 */
    NTRIP_CONNECT     = 1,            /* NTRIP连接服务器 */
    NTRIP_IDEL        = 2,            /* NTRIP空闲 */
}NTRIP_WorkState_t;


extern osThreadId_t thread_Ntrip_id;

extern NTRIP_WorkState_t Ntrip_State;

extern char Ntrip_Mount[50];	  //Ntrip默认挂载点
extern char Ntrip_user_code[50];//Ntrip默认账号密码


//void _Ntrip_thread(void *arg);
extern void Ntrip_TCP_Init(void);
extern void Ntrip_QX_Connect(void);
extern void Ntrip_SIM_Restart(void);
extern void Ntrip_apl_Init(void);
extern void Ntrip_RTCM_to_UBX(uint8_t *buf, int total_len );




#endif
