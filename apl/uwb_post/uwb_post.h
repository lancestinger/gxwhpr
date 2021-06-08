#ifndef __MAIN_H
#define __MAIN_H
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
//#include "deca_sleep.h"
#include "deca_plat/port/port.h"
#include <cmsis_os2.h>
#include "pubDef.h"




#define	FRAME_LEN_MAX		127
#define	KEY_VALUE_BUF_LEN		300



/*��վ����*/
#define TAG   					0    //��ǩ
#define ANCHOR   				1		 //��վ



typedef enum 
{
		MAIN_ANCHOR, 		//����վ
		SUB_ANCHOR,			//�ӻ�վ
		IDLE_ANCHOR,		//���л�վ
		UNKNOWN_ARNCHOR
} anchor_t;

typedef enum 
{
		HEAD_ANCHOR, 		//�׻�վ
		MID_ARNCHOR,		//�м��վ		
		TAIL_ANCHOR		  //β��վ
} anchor_post_type_t;




typedef enum 
{
		ACTIVATION_ALL_TAG_CMD,				//�����ǩ
		DYNAMIC_SLOT_ALLOCATION_CMD,	//��̬ʱ϶����
		TAG_REGISTER_CMD,							//��ǩע��
		ACK_TAG_REGISTER_CMD,					//��Ӧ��ǩע��
		SLOT_ALLOCATION_CMD, 					//ʱ϶����
		RANGING_CMD,									//���
		POLL,													//���POLL֡
		RESP,													//���RESP֡
		FINAL,												//���FINAL֡
		ACK,													//���ACK֡
		AUTO_CHOOSE_TX_POWER,					//�Զ�ѡ��tx power
		AUTO_CAL_ANT,									//�Զ�У׼�����ӳ�
		AUTO_CAL_ANT_ACK,							//�Զ�У׼�����ӳ�ȷ��
		AUTO_CAL_ANT_INFO,						//�Զ�У׼�����ӳ���Ϣ
		AUTO_CAL_ANT_INFO_ACK,				//�Զ�У׼�����ӳ���Ϣȷ��
		STOP_AUTO_CAL_ANT,						//ֹͣ�Զ�У׼����
		INTERFERENCE_SIGNAL						//�����ź�
} dm_comm_t;


typedef enum 
{
		MA_NET_ACTIVATION_ALL_TAG,						//�����ǩ
		MA_NET_TAG_REGISTER,									//��ǩע��
		MA_NET_SLOT_ALLOCATION, 							//ʱ϶����
		MA_NET_RANGING,												//���
		MA_NET_NETWORK_RELEASE,					  		//�����ͷ�
		MA_NET_WAITE													//�ȴ�
} m_archor_state_t;


typedef enum 
{
		SA_NET_NETWORKING, 										//����
		SA_NET_ACTIVATION_ALL_TAG,						//�����ǩ
		SA_NET_RANGING,												//���
		SA_NET_NETWORK_RELEASE,					  		//�����ͷ�
		SA_NET_WAITE					  							//�ȴ�
} s_archor_state_t;



typedef enum 
{
		TAG_WAITE_ACTIVATION,									//�ȴ�������
		TAG_REGISTER,													//ע��
		TAG_RECV_SLOT_ALLOCATION, 						//ʱ϶����
		TAG_RANGING,													//���
		TAG_SOLUTION													//�������������
} tag_state_t;



typedef enum 
{
		RET_SUCCESS, 											//�ɹ�
		RET_FAILED, 											//ʧ��
		RET_OTHER_FAILED,									//�������ܴ���
		RECV_TIME_OUT, 										//���ܳ�ʱ
		RECV_NO_IRQ, 											//�޽����ж�
		RECV_DATA_ERR, 										//�������ݴ���
		TIME_OUT,													//��ʱ
		NO_SIGNAL,
		NETWORK_TIME_OUT, 								//��λ���糬ʱ
} ret_t;


//dw1000���������״̬
typedef enum 
{
		DWT_TX_NO_DONE,
		DWT_TX_DONE,
		DWT_TX_ERR,
		DWT_RX_NO_DONE,
		DWT_RX_ERR,
		DWT_RX_TIMEOUT,
		DWT_RX_OK,
}dwt_txrxs_t;	



typedef enum 
{
		POS_INVALID=0,				//��λ��Ч
		POS_VALID,						//��λ��Ч
		POS_VALUE_INVALID,	  //��λֵ��Ч
		POS_VALUE_VALID,
} tag_pos_state_t;



typedef enum 
{
		START_AUTO_CHOOSE_TX_POWER=0,		//��ʼ�Զ�ѡ���豸tx_power
		SIGNAL_FRAME,																	//�ź�֡
		SIGNAL_RSSI_FRAME,														//�ź�ǿ��֡
		SIGNAL_RSSI_OK,																//�ź�ǿ��OK	
		STOP_AUTO_CHOOSE_TX_POWER,	          				//ֹͣ�Զ�ѡ��tx_power
} dwt_auto_tx_power_t;


typedef enum 
{
		START_TEST_RANGING=0,									//��ʼ���Ծ���
		STOP_TEST_RANGING,	          				//ֹͣ���Ծ���
} dwt_ranging_t;




//��ǩע����Ϣ
typedef struct 
{
		u16 tag_id; 												//��ǩid
		u16 tag_wait_time_ms; 							//��ǩ�ȴ�ע��ʱ��
} tag_reg_info_t; 



/*�豸������Ϣ*/

#pragma pack(4)
typedef struct 
{
		u32 tx_power;												//���书������
		u16 device_type; 										//�豸���ͣ�1.��վ�� 2.��ǩ
		u16 ant_delay;										  //�����ӳ�ʱ��		
		u16 tag_id; 												//��ǩid ��0-0xffff��
		u16 anchor_id; 											//��վid ��0-0xffff��
		u16 max_allow_tag_num;						  //��������ǩ����
		u16	dyn_slot_long;									//��̬ʱ϶ʱ��
		u16	ranging_slot_long;							//���ʱ϶ʱ��						
		u16 kalman_q;												//�������˲�-Q
		u16 kalman_r;												//�������˲�-R		
		u8 on_left;													//��վ����·�����,1�����󣬷�������
		u8 anchor_idle_num;									//���л�վ��������
		u8 chan;														//dw1000ͨ��
		u8 t2wall_fl_num;										//���ܻ������ǽ�ڵľ����˲�ƽ������
		double position[3]; 								//��վ����x,y,z
		double ref_position[3]; 						//�ο�������x,y,z
		float t2wall_actual_dist;					//���ܻ������ǽ�ڵ�ʵ�ʾ��룬��λ��
		float t2wall_threshold;						//���ܻ������ǽ�ڵ���ֵ���룬��λ��
		float tag_h; 											//��ǩ��Ե���Ĵ�ֱ�߶�
		float anchor_h; 									//��վ��Ե���Ĵ�ֱ�߶�
		float	sig_qa_thr;									//�ź�������ֵ
} device_config_t; 
#pragma pack()



/*��λ�����ϱ��¼�*/
typedef struct 
{
		u8 	anchor_state; 									//��վ״̬��Ϣ��0.����վ��1.�ӻ�վ��2.���л�վ
		u8 	anchor_sub_state; 							//��վ��״̬��Ϣ������վ��0.����״̬��1.�ȴ���ǩע��״̬��2.ʱ϶����״̬��3.���״̬��4.����״̬��		
		u8 	tag_state; 											//0.�ȴ�������״̬��1.ע��״̬��2.����ʱ϶����״̬��3.���״̬��4.����״̬	
		tag_pos_state_t 	tag_position_valid_flag; //��λ��Ч״̬
		double tag_position[3]; 						//��ǩ����x,y,z
		float t2ref_dist;									  //���ܻ����ο������
		float t2wall_dist;									//���ܻ������ǽ�ڵľ���
		float t2main_dist;									//���ܻ�������վ�ľ���
		float main_rssi;												  //�����ź�ǿ��
		float sub_rssi;
		float D0;                           //�����Ϣd0
		float D1;                           //�����Ϣd1
		u16 main_anchor_id; 								//����վid ��0-0xffff��		
		u16 sub_anchor_id; 								  //�ӻ�վid ��0-0xffff��
		u32 position_interval;							//��λ���
		u32 pos_Time;
} pos_info_t;




/*��λ������Ϣ*/
typedef struct 
{
		u16 main_anchor_id; 								//����վid ��0-0xffff��		
		u16 sub_anchor_id; 								  //�ӻ�վid ��0-0xffff��
		double anchor_position[6]; 					  //����վ��ӻ�վ����x,y,z��ǰ0~2Ϊ����վ���꣬2��5�Ǵӻ�վ����
		u8 on_left;
		float main_anchor_h;
		float sub_anchor_h;
		u16 max_allow_tag_num;							//��������ǩ����
		u16 dyn_slot_long;									//��̬ʱ϶ʱ��
		u16 ranging_slot_long;							//���ʱ϶ʱ��
		u32 register_timeout_us;						//ע�ᳬʱʱ��		
} locate_net_info_t; 


/*�Զ��������书������*/
typedef struct 
{
		u8 auto_choose_tx_power; 						//1:�����Զ�ѡ��tx_power	��0���ر��Զ�ѡ��tx_power
		float main_device_rssi_threshold;   //���豸����ź�ǿ����ֵ
		float sub_device_rssi_threshold;		//���豸����ź�ǿ����ֵ
		float main_device_rssi_rang;				//���豸�ź�ǿ�ȿɳ�������ź�ǿ����ֵ�ķ�Χ
		float sub_device_rssi_rang; 				//���豸�ź�ǿ�ȿɳ�������ź�ǿ����ֵ�ķ�Χ
} dwt_auto_tx_power_config_t; 

/*�Զ�У׼�����շ��ӳ�*/
typedef struct 
{
		u8 auto_ant_cal_open; 							//1:�������Զ�У׼��0���ر������Զ�У׼
		u8 salve_ant_cal_en;								//1:ʹ��У׼���豸���ߣ�0����ֹУ׼���豸����	
		s32 actual_dist_cm;									//ʵ�ʾ���
} dwt_auto_ant_cal_t; 



typedef struct 
{
	float rx_power;
	u16 std_noise;
	s16 rx_fp_power_diff;
	s16 f1_noise_amp_diff;
	
}rcv_signal_quality_t;



/*��ֵ��*/
typedef struct 
{
		u8 key_vaule_valid_flag; 						//0:��Ч��1����Ч
		u8 key_value_buf[KEY_VALUE_BUF_LEN];
} key_vaule_t; 


extern device_config_t			g_device_config;
extern locate_net_info_t		g_locate_net_info;
extern pos_info_t					  g_pos_info;
extern u8 g_ranging_flag;
extern u8 g_detection_signal_flag;
extern u8 g_interference_signal_flag;
extern u8 g_open_filter_flag;
extern u8 g_reset_filter;
extern dwt_auto_tx_power_config_t g_dwt_auto_tx_power_config;
extern dwt_auto_ant_cal_t g_dwt_auto_ant_cal;
extern anchor_post_type_t	g_anchor_post_type;


extern key_vaule_t g_key_vaule;





#define DM1000_TXRX_FLAG 	0x00000001ul

extern osEventFlagsId_t evt_id;



#if 1
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

#endif

#if 0
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_256,    /* Preamble length. Used in TX only. */
    DWT_PAC16,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (257 + 8 - 16)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

#endif





//#define TX_ANT_DLY 16540//16474//16583
//#define RX_ANT_DLY 16540//16474//16583
#define ANT_DLY 16540
#define FINAL_MSG_TS_LEN 4  //ʱ�����ݳ���

	

extern u16 ERROR_FLAG;  //��������������־λ���ﵽһ����������
extern  u16   Flash_Device_ID;         //��8λΪ�λ�վID����Χ0~6  ��8λΪ��ǩID 0~99    �������ڲ� ��ǩIDΪ0~247  �λ�վIDΪ248~245  ����վIDΪ255��
extern  float Triangle_scale;    //��վɸѡ����ʹ�õı�������
extern  float g_uwb_rg_ssi;
extern u8 g_detection_signal_flag;
extern anchor_post_type_t  g_anchor_post_type;






 void GPIO_Toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
 unsigned short get(short);

extern  void DWM1000_init(void);
extern  void DWM1000_check(void);
extern  void meas_distance(void);
extern  ret_t major_anchor(void);
extern  ret_t sub_anchor(void);
extern  void reset_position_default_para(void);
extern  void save_device_para(void);
extern  void get_device_para(void);
extern  void show_position_para(void);
extern  void show_anchor_state(void);
extern u32 calc_tx_power_config_value(u8* ptr);
extern float calc_tx_power_config_value_to_db(u32 value);
extern u16 antDelay2antDist(u16 ant_delay);
extern u16 antDist2antDelay(u16 ant_delay_dist);


 
#endif


