#ifndef __MAIN_H
#define __MAIN_H
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
//#include "deca_sleep.h"
#include "platform/port/port.h"
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
		ACTIVATION_ALL_TAG_CMD,				//�����ǩ
		DYNAMIC_SLOT_ALLOCATION_CMD,	//��̬ʱ϶����
		TAG_REGISTER_CMD,							//��ǩע��
		ACK_TAG_REGISTER_CMD,					//��Ӧ��ǩע��
		SLOT_ALLOCATION_CMD, 					//ʱ϶����
		RANGING_CMD,									//���
		POLL,													//���POLL֡
		RESP,													//���RESP֡
		FINAL,												//���FINAL֡
		ACK,														//���ACK֡
		AUTO_CHOOSE_TX_POWER,					//�Զ�ѡ��tx power
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
		RECV_TIME_OUT, 										//���ܳ�ʱ
		RECV_HIT, 												//������ײ
		TIME_OUT,													//��ʱ
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
		POS_VALID,					//��λ��Ч
		POS_VALUE_INVALID,	        //��λֵ��Ч
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
		u16 device_type; 										//�豸���ͣ�1.��վ�� 2.��ǩ
		u16 ant_tx_delay;										//���߷����ӳ�ʱ��
		u16 ant_rx_delay;										//���߽����ӳ�ʱ��		
		u16 tag_id; 												//��ǩid ��0-0xffff��
		u16 anchor_id; 											//��վid ��0-0xffff��
		u16	dyn_slot_long;									//��̬ʱ϶ʱ��
		u16	ranging_slot_long;							//���ʱ϶ʱ��						
		u16 kalman_q;												//�������˲�-Q
		u16 kalman_r;												//�������˲�-R	
		u8 on_left;											//��վ����·�����,1�����󣬷�������
		u8 anchor_idle_num;							//���л�վ��������
		u8 chan;												//dw1000ͨ��
		u32 tx_power;												//���书������
		double position[3]; 								//��վ����x,y,z
		double ref_position[3]; 						//�ο�������x,y,z
		double t2wall_actual_dist;					//���ܻ������ǽ�ڵ�ʵ�ʾ���
		float tag_h; 											//��ǩ��Ե���Ĵ�ֱ�߶�
		float anchor_h; 											//��վ��Ե���Ĵ�ֱ�߶�

} device_config_t; 
#pragma pack()



/*��λ�����ϱ��¼�*/
typedef struct 
{
		u8 	anchor_state; 									//��վ״̬��Ϣ��0.����վ��1.�ӻ�վ��2.���л�վ
		u8 	anchor_sub_state; 							//��վ��״̬��Ϣ������վ��0.����״̬��1.�ȴ���ǩע��״̬��2.ʱ϶����״̬��3.���״̬��4.����״̬��		
		u8 	tag_state; 											//0.�ȴ�������״̬��1.ע��״̬��2.����ʱ϶����״̬��3.���״̬��4.����״̬	
		tag_pos_state_t 	tag_position_valid_flag; //��λ��Ч״̬
		double tag_position[3]; 								//��ǩ����x,y,z
		double t2ref_dist;									//���ܻ����ο������
		double t2wall_dist;									//���ܻ������ǽ�ڵľ���
		double t2main_dist;
		double rssi;												//�����ź�ǿ��
		double D0;                                          //�����Ϣd0
		double D1;                                          //�����Ϣd1
		u16 main_anchor_id; 								//����վid ��0-0xffff��		
		u16 sub_anchor_id; 								  //�ӻ�վid ��0-0xffff��
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
extern dwt_auto_tx_power_config_t g_dwt_auto_tx_power_config;
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





#define TX_ANT_DLY 16540//16474//16583
#define RX_ANT_DLY 16540//16474//16583
#define FINAL_MSG_TS_LEN 4  //ʱ�����ݳ���

	

extern u16 ERROR_FLAG;  //��������������־λ���ﵽһ����������
extern  u16   Flash_Device_ID;         //��8λΪ�λ�վID����Χ0~6  ��8λΪ��ǩID 0~99    �������ڲ� ��ǩIDΪ0~247  �λ�վIDΪ248~245  ����վIDΪ255��
extern  float Triangle_scale;    //��վɸѡ����ʹ�õı�������

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
u32 calc_tx_power_config_value(u8* ptr);
float calc_tx_power_config_value_to_db(u32 value);




 
#endif


