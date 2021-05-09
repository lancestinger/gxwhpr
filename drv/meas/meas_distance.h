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



/*基站类型*/
#define TAG   					0    //标签
#define ANCHOR   				1		 //基站



typedef enum 
{
		MAIN_ANCHOR, 		//主基站
		SUB_ANCHOR,			//子基站
		IDLE_ANCHOR,		//空闲基站
		UNKNOWN_ARNCHOR
} anchor_t;



typedef enum 
{
		ACTIVATION_ALL_TAG_CMD,				//激活标签
		DYNAMIC_SLOT_ALLOCATION_CMD,	//动态时隙分配
		TAG_REGISTER_CMD,							//标签注册
		ACK_TAG_REGISTER_CMD,					//回应标签注册
		SLOT_ALLOCATION_CMD, 					//时隙分配
		RANGING_CMD,									//测距
		POLL,													//测距POLL帧
		RESP,													//测距RESP帧
		FINAL,												//测距FINAL帧
		ACK,														//测距ACK帧
		AUTO_CHOOSE_TX_POWER,					//自动选择tx power
		INTERFERENCE_SIGNAL						//干扰信号
} dm_comm_t;


typedef enum 
{
		MA_NET_ACTIVATION_ALL_TAG,						//激活标签
		MA_NET_TAG_REGISTER,									//标签注册
		MA_NET_SLOT_ALLOCATION, 							//时隙分配
		MA_NET_RANGING,												//测距
		MA_NET_NETWORK_RELEASE,					  		//组网释放
		MA_NET_WAITE													//等待
} m_archor_state_t;


typedef enum 
{
		SA_NET_NETWORKING, 										//组网
		SA_NET_ACTIVATION_ALL_TAG,						//激活标签
		SA_NET_RANGING,												//测距
		SA_NET_NETWORK_RELEASE,					  		//组网释放
		SA_NET_WAITE					  							//等待
} s_archor_state_t;



typedef enum 
{
		TAG_WAITE_ACTIVATION,									//等待被激活
		TAG_REGISTER,													//注册
		TAG_RECV_SLOT_ALLOCATION, 						//时隙分配
		TAG_RANGING,													//测距
		TAG_SOLUTION													//距离与坐标解算
} tag_state_t;



typedef enum 
{
		RET_SUCCESS, 											//成功
		RET_FAILED, 											//失败
		RECV_TIME_OUT, 										//接受超时
		RECV_HIT, 												//接受碰撞
		TIME_OUT,													//超时
		NETWORK_TIME_OUT, 								//定位网络超时
} ret_t;


//dw1000发送与接受状态
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
		POS_INVALID=0,				//定位无效
		POS_VALID,					//定位有效
		POS_VALUE_INVALID,	        //定位值无效
		POS_VALUE_VALID,
} tag_pos_state_t;



typedef enum 
{
		START_AUTO_CHOOSE_TX_POWER=0,		//开始自动选择设备tx_power
		SIGNAL_FRAME,																	//信号帧
		SIGNAL_RSSI_FRAME,														//信号强度帧
		SIGNAL_RSSI_OK,																//信号强度OK	
		STOP_AUTO_CHOOSE_TX_POWER,	          				//停止自动选择tx_power
} dwt_auto_tx_power_t;


typedef enum 
{
		START_TEST_RANGING=0,									//开始测试距离
		STOP_TEST_RANGING,	          				//停止测试距离
} dwt_ranging_t;




//标签注册信息
typedef struct 
{
		u16 tag_id; 												//标签id
		u16 tag_wait_time_ms; 							//标签等待注册时间
} tag_reg_info_t; 



/*设备配置信息*/

#pragma pack(4)
typedef struct 
{
		u16 device_type; 										//设备类型：1.基站、 2.标签
		u16 ant_tx_delay;										//天线发送延迟时间
		u16 ant_rx_delay;										//天线接受延迟时间		
		u16 tag_id; 												//标签id （0-0xffff）
		u16 anchor_id; 											//基站id （0-0xffff）
		u16	dyn_slot_long;									//动态时隙时长
		u16	ranging_slot_long;							//测距时隙时长						
		u16 kalman_q;												//卡尔曼滤波-Q
		u16 kalman_r;												//卡尔曼滤波-R	
		u8 on_left;											//基站在马路的左侧,1：在左，否则在右
		u8 anchor_idle_num;							//空闲基站持续次数
		u8 chan;												//dw1000通道
		u32 tx_power;												//发射功率增益
		double position[3]; 								//基站坐标x,y,z
		double ref_position[3]; 						//参考点坐标x,y,z
		double t2wall_actual_dist;					//接受机到隧道墙壁的实际距离
		float tag_h; 											//标签相对地面的垂直高度
		float anchor_h; 											//基站相对地面的垂直高度

} device_config_t; 
#pragma pack()



/*定位主动上报事件*/
typedef struct 
{
		u8 	anchor_state; 									//基站状态信息，0.主基站、1.从基站、2.空闲基站
		u8 	anchor_sub_state; 							//基站子状态信息。主基站：0.激活状态、1.等待标签注册状态、2.时隙分配状态、3.测距状态、4.空闲状态；		
		u8 	tag_state; 											//0.等待被激活状态、1.注册状态、2.接受时隙分配状态、3.测距状态、4.空闲状态	
		tag_pos_state_t 	tag_position_valid_flag; //定位有效状态
		double tag_position[3]; 								//标签坐标x,y,z
		double t2ref_dist;									//接受机到参考点距离
		double t2wall_dist;									//接受机到隧道墙壁的距离
		double t2main_dist;
		double rssi;												//接受信号强度
		double D0;                                          //测距信息d0
		double D1;                                          //测距信息d1
		u16 main_anchor_id; 								//主基站id （0-0xffff）		
		u16 sub_anchor_id; 								  //从基站id （0-0xffff）
} pos_info_t;




/*定位网络信息*/
typedef struct 
{
		u16 main_anchor_id; 								//主基站id （0-0xffff）		
		u16 sub_anchor_id; 								  //从基站id （0-0xffff）
		double anchor_position[6]; 					  //主基站与从基站坐标x,y,z，前0~2为主基站坐标，2至5是从基站坐标
		u8 on_left;
		float main_anchor_h;
		float sub_anchor_h;
} locate_net_info_t; 


/*自动调整发射功率增益*/
typedef struct 
{
		u8 auto_choose_tx_power; 						//1:开启自动选择tx_power	，0：关闭自动选择tx_power
		float main_device_rssi_threshold;   //主设备最低信号强度阈值
		float sub_device_rssi_threshold;		//从设备最低信号强度阈值
		float main_device_rssi_rang;				//主设备信号强度可超过最低信号强度阈值的范围
		float sub_device_rssi_rang; 				//从设备信号强度可超过最低信号强度阈值的范围
} dwt_auto_tx_power_config_t; 


/*键值对*/
typedef struct 
{
		u8 key_vaule_valid_flag; 						//0:无效，1：有效
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
#define FINAL_MSG_TS_LEN 4  //时间数据长度

	

extern u16 ERROR_FLAG;  //测距错误计算次数标志位，达到一定次数跳出
extern  u16   Flash_Device_ID;         //高8位为次基站ID，范围0~6  低8位为标签ID 0~99    （程序内部 标签ID为0~247  次基站ID为248~245  主基站ID为255）
extern  float Triangle_scale;    //基站筛选过程使用的比例参数

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


