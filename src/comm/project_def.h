
#ifndef _PROJECT_DEF_H_
#define _PROJECT_DEF_H_
#include "pubdef.h"
#include "gnss/gnss_fml.h"
#include "monitor/monitor_apl.h"


#define GPS_MAX_NUM		50





//ȫ���¼���־,bitλ��Ч
typedef enum
{
	OS_EVENT_PPS = (0x1<<0),				//PPS�¼�
	OS_EVENT_DOPLER_UPDATE = (0x1<<1),	// �����ո����¼�
	OS_EVENT_PPS_SSHELL = (0x1<<2),		//PPSָ��SSHELL�¼�
}os_event_flag_enum;

//�豸����
typedef enum
{
	DEV_GPS_GLO = 0,
	DEV_GPS_BDS_GLO,
	// DEV_GPS_BDS,
	// DEV_BDS_GLO
}device_enum;

//���Դ��ڹ���ģʽ
typedef enum
{
    DBG_OUT_USART_NORMAL_MODEL   = 0,
    DBG_OUT_USART_RECEIVER_MODEL = 1
}DBG_OUT_USART_WORK_MODEL;

extern device_enum g_gnss_work_model;          //����ģʽ  0��ʾGPS&GLONASS  1��ʾGPS&BDS&GLONASS
extern DBG_OUT_USART_WORK_MODEL g_dbg_out_usart_model;     //���Դ��ڹ���ģʽ

//Э�������ṹ������
#pragma pack(4)

typedef struct
{
	U16 sat_num;
	U16 sat_type;
	U16 prn;
	U16 week;
	U16 iodc;
	U32 toc;
	S16 af2;
	S16 af1;
	S32 af0;
	U16 iode;
	S16 crs;
	S16 delta_n;
	S32 m0;
	S16 cuc;
	U64 e;
	S16 cus;
	U64 sqrta;
	U32 toe;
	S16 cic;
	S32 omega0;
	S16 cis;
	S32 i0;
	S16 crc;
	S32 w;
	S32 omegad;
	S16 tgd;
	S16 aodo;
	S16 idot;
}EPH_t;

typedef struct
{
	U8 valid;
	U8 use;
	U8 cn0;
	S8 elev;
	EPH_t eph;
}PROTOCOL_EPH_t;

typedef struct
{
	U8 cur_valid;
	U8 use;
	U16 nav_rcv_flag;
	U8 refresh_flag;
	S8 elev;
	//eph_type1_t eph;
}PROTOCOL_GPS_ORG_EPH_t;

typedef struct
{
	U8 gnss_num;
	U8 gnss_type;
	U16 prn;
	U32 toa;
	F32 deltI;
	F64 e;
	F64 sqrta;
	F64 omega0;
	F64 w;
	F64 m0;
	F64 omegad;
	F32 af1;
	F32 af0;
	U32 week;
}GPS_ALM_ORG_t;

typedef struct
{
	//U8 prn;			//���Ǳ��
	U8 valid_num;	//�ܵ���Ч������Ŀ
	U8 cur_valid;	//��ǰ�����Ƿ���Ч
	U16 resvd;
	GPS_ALM_ORG_t alm;
}PROTOCOL_ALM_t;

typedef struct
{
	U8 valid_num;	//�ܵ���Ч������Ŀ
	U8 cur_valid;	//��ǰ�����Ƿ���Ч
	U8 refresh_flag;	//ˢ�� ��־
	U8 use;
	U8 nav_rcv_flag;	//�������Ľ��ձ�־
	U32 pre_tb;			//����֮ǰ�������ο�ʱ��
//	eph_type2_t glo_eph;
}PROTOCOL_GLO_EPH_t;

typedef struct
{
    U8 predicNum;   //�������,��Ϊ0ʱ��ʾΪֱ��ʹ����������,δ������������
//    eph_type1_t eph;
}GPS_CUR_EPH_TypeDef;

typedef struct
{
    U8 predicNum;   //�������,��Ϊ0ʱ��ʾΪֱ��ʹ����������,δ������������
//    eph_type2_t eph;
}GLO_CUR_EPH_TypeDef;


//�������ݽṹ��
typedef struct
{
	U8 dhcp;		//DHCP���� 0---��ֹ�� 1---ʹ��
	U8 net_ip[4];
	U8 net_mask[4];
	U8 net_gateway[4];
	U8 net_mac[6];
	U8 server_ip[4];
	U16 server_port;
}net_para_t;

typedef struct
{
	U8 ubx_enable;
	S8 gps_leap_sec;				//GPS������
	// S8 time_zone;					//ʱ��
	// U8 pl_start_enable;			    //α�����ϵ�����
//	ref_type_enum ref_sel;			//�ο�Դѡ��
//	ref_switch_enum ref_sw_mode;	//�л�ģʽ
//	S32 ocxo_voltage[OCXO_NUM];	    //ѹ�ص�ѹ
//	lla_t usr_lla;					//�û�λ��
	// F32 el;							//��ֹ��
	// F32 max_speed;					//����ٶ�
//	F32 atten[4];			//˥��ֵ
//	F32 delay[4];
//	circle_para_t circle_para;		//Բ���˶�����
//	head_speed_t speed_para;		//ֱ���˶�����
//	head_acc_t accspeed_para;		//���ٶȲ���
	net_para_t net;					//�������
    // U32 work_model;                 //����ģʽ 0��ʾGPS&GLONASS 1��ʾGPS&BDS&GLONASS
    // U32 pl_auto_time_enable;        //α�����Զ���ʱʹ��
    // U32 circle_curPos_enable;       //ʹ�õ�ǰ��λλ����ΪԲ�Ľ���Բ���˶�ʹ��
    // U8 sendIP1[4];                  //����IP1
    // U8 sendIP2[4];                  //����IP2
    // U32 devInfoSendCycle;           //����״̬Ѳ����Ϣ����
    // F32 atten_lim[4];        //ͨ������˥����С����
    // F32 atten_offset[4][12];
    // U8 transCtrlMode;               //�������ģʽ(����)
    // U8 antsup;                      //��������״̬
    // U8 reserved2;                   //����
    // U8 reserved3;                   //����
	U32 crc;
}cfg_para_t;

typedef struct
{
	U32 used_seconds;	//��������
	//S32 remain_seconds;	//ʣ������
	U32 crc16;
}extra_para_t;

//����������
typedef struct
{
	cfg_para_t cfg;
	gnss_data_t* p_gnss_handle;
//	ocxo_handle_t* p_ocxo1_handle;
//	ref_handle_t* p_ref_handle;
//	time_handle_t* p_time_handle;
//	pl_handle_t* p_pl_handle;
	monitor_status_t* p_monitor;
}main_handle_t;

//license���ݽṹ
typedef struct
{
    U32 license;
    U32 crc;
}license_handle_t;

//hn�����ļ�ʹ�����ݽṹ
typedef struct
{
    U8 type;//1��ʾ������������,2��ʾ�������Գ���,����ֵ���ʾ������������
    U8 reserve1;
    U8 reserve2;
    U8 reserve3;
    U32 crc;
}hn_handle_t;



#pragma pack()


extern extra_para_t ex_para_g;
extern main_handle_t main_handle_g;
extern osEventFlagsId_t os_event_id_g;
extern license_handle_t license_handle_g;  //license��������
extern hn_handle_t hn_handle_g;    //hn���ò�������

extern void sys_para_init(void);
extern S8 sys_para_get_gps_leapsec(void);
extern void sys_set_ipaddr(U8* ip_buf);
extern void sys_set_maskaddr(U8* ip_buf);
extern void sys_set_gatewayaddr(U8* ip_buf);
extern void sys_set_macaddr(U8* ip_buf);
extern U8 sys_para_set_authcode(U32 code);
extern void sys_reset_para_save(void);
extern void sys_para_save(void);
extern void sys_net_cfg_start(void);
extern void sys_para_reset(void);

extern void license_reset(void);
extern void license_save(void);
extern void hn_cfg_save(void);
#endif
/*eof*/
