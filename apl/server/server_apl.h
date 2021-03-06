#ifndef _SERVER_APL_H__
#define _SERVER_APL_H__

#include "project_def.h"
#include "apl/EH_uwb/EH_uwb.h"


#if defined(VERSION_YANCHONG)
	#define BASE64_LIST   "+o2G4POYvUi9tLf6wbQzrSTlZqXEjVekhCuy3aJd8mxFc1nMH7NABKpI5WRsD0/g"  //自定义表
#else
	#define BASE64_LIST   "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"  //标准base64表
#endif

typedef struct
{
    int cmd;
    int response;
    char task_uuid[20];
    char cmd_uuid[20];

}Server_Ori_Data;


typedef struct
{
    u16 id;             //ID
    double longitude;           //
    double latitude;           //
    double height;           //
    double tag_h; 			//
    u16 tranAntDelay;   //
    u16 recvAntDelay;   //
    u32 tx_power;				//		
	u8 on_left;					//    
    U8 mode;
	
}Server_Data;

typedef enum
{
	RTK_MODE    =0,
	UWB_MODE    =1,
	DATA_ERROR  =2,
	
}Server_upload_state;

typedef struct
{
	U32 RTK_TIME;
	U32 UWB_TIME;
}Pos_count;

extern Server_Data server_Data;
extern Server_Ori_Data serOriData;

extern int SERVER_RTK; 				  /* RTK数据Ready标志 */
extern int SERVER_UWB; 				  /* UWB数据Ready标志 */
extern int SERVER_GNSS; 			  /* GNSS数据Ready标志 */
extern int SERVER_ERROR;			  /* 定位数据错误标志 */

extern double UWB_angle;
extern double UWB_Veloc;

extern U8 UDP_DST[5];
extern int UDP_PORT;

extern U8 upload_hpr_state_to_server(void);
extern U8 parse_server_data(U8* buf, int len);
extern U8 UDP_upload_hpr_location(void);
extern U8 upload_hpr_update_feedback_to_server(Server_Ori_Data data,int state);
extern U8 upload_efs_all_para_to_server(void);
extern int num_strchr(const char *str, char c);
extern int base64_decode(const char * base64, unsigned char * dedata);
extern int base64_encode( const unsigned char * bindata, char * base64, int binlength );
extern void UWB_Vel_Angle_calc(void);
extern U8 EH_UWB_upload(netStruct netbuf);
extern void Server_apl_init(void);

#endif
