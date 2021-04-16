
 /*! ----------------------------------------------------------------------------
 * @edition V3.2
 * @author 广州联网科技有限公司
 * @web www.gzlwkj.com
 */
 
#include <string.h>
#include <stdio.h>
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include "platform/deca/deca_sleep.h"
#include "platform/port/port.h"
//#include "usart.h"
#include "platform/flash/stmflash.h"
#include "meas_distance.h"
#include "platform/delay/delay.h"
//#include "timer.h"
#include "algorithm/loc.h"
//#include "oled.h"
//#include "dma.h"
#include "log.h"
#include "crc/crc.h"
#include "PubDef.h"
#include "uart/uart_drv.h"
#include "fml/uart/uart_fml.h"
#include "apl/imu/Coordi_transfer.h"
#include "apl/monitor/monitor_apl.h"




typedef signed long long int64;
typedef unsigned long long uint64;
typedef unsigned long long uint64;

/******************************************************************************
														内存FLASH区数据
*******************************************************************************/

u16   Flash_Device_ID;        	  //高8位为次基站ID，范围0~6  低8位为标签ID 0~99    （程序内部 标签ID为0~247  次基站ID为248~254  主基站ID为255）


/******************************************************************************
														系统流程自定义
*******************************************************************************/
u16 ERROR_FLAG;  						//测距错误计算次数标志位，达到一定次数跳出
u16 DWM1000_SET_DATA[5];     //DWM1000监测配置参数是否被改变的缓存数组
float Triangle_scale=1.2;    //基站筛选过程使用的比例参数
/******************************************************************************
														DWM1000测距计算变量
*******************************************************************************/
//static int16_t dist[20];    //LP低通滤波缓存
//static double tof;             //飞行时间
//static double distance,dist2;  //理论距离 ，估算距离

static const char device_type_str[][10] = 
{
	"TAG",
	"ANCHOR"
};

static const char anchor_type_str[][20] = 
{
	"MAIN_ANCHOR",
	"SUB_ANCHOR",
	"IDLE_ANCHOR",
	"UNKNOWN_ARNCHOR"
};

static const char dm_comm_type_str[][25] = 
{
	"ACTIVATION_ALL_TAG",
	"DYNAMIC_SLOT_ALLOCATION",
	"TAG_REGISTER",
	"ACK_TAG_REGISTER",
	"SLOT_ALLOCATION",
	"RANGING",
	"POLL",
	"RESP",
	"FINAL",
	"ACK"
};

static const char m_archor_state_type_str[][30] = 
{
	"MA_NET_ACTIVATION_ALL_TAG",
	"MA_NET_TAG_REGISTER",
	"MA_NET_SLOT_ALLOCATION",
	"MA_NET_RANGING",
	"MA_NET_NETWORK_RELEASE",
	"MA_NET_WAITE"
};

static const char s_archor_state_type_str[][30] = 
{
	"SA_NET_NETWORKING",
	"SA_NET_ACTIVATION_ALL_TAG",
	"SA_NET_RANGING",
	"SA_NET_NETWORK_RELEASE",
	"SA_NET_WAITE"
};

static const char tag_state_type_str[][30] = 
{
	"TAG_WAITE_ACTIVATION",
	"TAG_REGISTER",
	"TAG_RECV_SLOT_ALLOCATION",
	"TAG_RANGING",
	"TAG_SOLUTION"
};

static const char ret_type_str[][20] = 
{
	"RET_SUCCESS",
	"RET_FAILED",
	"RECV_TIME_OUT",
	"RECV_HIT",
	"TIME_OUT",
	"NETWORK_TIME_OUT"
};

static const char dwt_txrxs_type_str[][20] = 
{
	"DWT_TX_NO_DONE",
	"DWT_TX_DONE",
	"DWT_TX_ERR",
	"DWT_RX_NO_DONE",
	"DWT_RX_ERR",
	"DWT_RX_TIMEOUT",
	"DWT_RX_OK"
};


/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
UWB微秒（UUS）到设备时间单位（DTU，约15.65 ps）的转换系数。
 * 1 uus = 512 / 499.2 祍 and 1 祍 = 499.2 * 128 dtu. 
  1 UUS＝512／499.2，1＝499.2×128 DTU。*/
#define UUS_TO_DWT_TIME 65536
#define SPEED_OF_LIGHT 299702547


/******************************************************************************
														DWM1000组网变量
*******************************************************************************/
//6.4M，前导码长度128

#define PACKAGE_BUF_SIZE								127
#define BUF_SIZE												101
#define DW1000_100US_TIMER_CNT					0x6180  //计时次数/100us

#define MAX_DYNAMIC_SLOT_FRAME_SEND_CNT	15
#define MAX_DYNAMIC_SLOT_FRAME					100
#define MAX_TAG_NUM											100
#define SLOT_LONG												400//300  	  //单位微秒
#define RANGING_SLOT_LONG								3000//2200	  //单位微秒

#define NETWORK_CYCLE_TIME							3700		//网络周期，单位100us
#define ACTIVE_TAG_TIMEOUT							10000		//激活标签超时时间，单位1us
#define ALL_TAG_REGISTER_TIMEOUT				110000  //150000  //单位1us
#define TAG_WAIT_SLOT_TIMEOUT						5000	  //标签等待时隙表超时时间，单位1us
#define TAG_RANGING_TIMEOUT							230000  //10000//5000	//标签等待测距超时时间，单位1us
#define FREE_TIME												50      //5ms空闲时间，单位100us

#define TX_NORMAL_TIMEOUT								400//300  		//单位1us，发送120字节时使用的时间
#define RX_NORMAL_TIMEOUT								10000   //系统定时器，单位1.02us
#define DM9000_RX_TIMEOUT								800     //单位1.02us
						
#define TAG_WAIT_TIMEOUT								60000//1300000 //单位1us


#define NO_OPEN_RECV					0
#define OPEN_RECV						  1

#define ACK_REGISTER_MAX_NUM_PER 	25		//每次回应注册成功标签的个数
#define SEND_RANGING_SLOT_MAX_NUM_PER 	25		//每次发送测距时隙的个数


static uint8 g_rx_buf[FRAME_LEN_MAX];
static uint8 g_rx_len;

static anchor_t g_anchor_type;
static tag_state_t tag_state = TAG_WAITE_ACTIVATION;
static u16 g_recv_tag_id = 0;
static locate_net_info_t g_locate_net_info;
static dwt_txrxs_t  g_tx_state = DWT_TX_NO_DONE;
static dwt_txrxs_t  g_rx_state = DWT_RX_NO_DONE;

static u8 success_recv_total_cnt = 0;
static tag_reg_info_t tag_reg_info_buf[MAX_TAG_NUM];
u32 g_time_start, g_time_end, g_m_anchor_last_time_start;
u32 g_last_tag_position_time;
u8 first_acitve_flag = 0;
u8 g_ranging_flag = 0;
u8 g_interference_signal_flag = 0;
u8 g_detection_signal_flag = 0;
u8 g_get_tx_timestamp_flag = 0;
u8 g_get_rx_timestamp_flag = 0;
dwt_auto_tx_power_config_t g_dwt_auto_tx_power_config;
key_vaule_t g_key_vaule;



uint64 g_tx_timestamp = 0;
uint64 g_rx_timestamp = 0;








static float g_uwb_rg_ssi;



device_config_t			g_device_config;
locate_net_info_t		g_locate_net_info;
pos_info_t					g_pos_info;





extern volatile u16 g_timer_cnt_100ms;	



/******************************************************************************
														静态函数声明
*******************************************************************************/
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
u32 calc_tx_power_config_value(u8* ptr);
void init_anchor_type(void);
void join_running_network(void);
ret_t major_anchor(void);
ret_t sub_anchor(void);
ret_t idle_anchor(void);
ret_t tag_test_111byte_ranging(void);
void auto_choose_slave_tx_power(void);
void auto_choose_main_tx_power(void);
ret_t anchor_test_ranging(void);
ret_t tag_test_ranging(void);









//保存设备参数
void save_device_para(void)
{
	FILE* fd = 0;
	u16 crc;
	u16 len;
	u8 buf[100];

	if(g_device_config.device_type == TAG)
	{
		Flash_Device_ID = g_device_config.tag_id;	
	}
	else if(g_device_config.device_type == ANCHOR)
	{
		Flash_Device_ID = g_device_config.anchor_id;
	}

	len = sizeof(device_config_t);
	GLOBAL_MEMCPY(buf, (u8*)&g_device_config, len);
	crc = crc16(buf, len);
	buf[len] = crc/256;
	buf[len+1] = crc%256; 
	len += 2;

	if((fd = fopen(POS_PARA_FILE, "w")) != NULL)
	{
		fwrite(buf, len, 1, fd);
		fclose(fd);
		DBG_PRINT(("定位参数保存成功!\r\n"));
	}
	else
	{
		DBG_PRINT(("定位参数保存失败!\r\n"));
	}	

}



void reset_position_default_para(void)
{
		g_device_config.device_type=TAG;
		Flash_Device_ID=TAG_ID_SUM; 
		if(g_device_config.device_type == TAG)
		{
			g_device_config.tag_id = Flash_Device_ID;	
		}
		else if(g_device_config.device_type == ANCHOR)
		{
			g_device_config.anchor_id = Flash_Device_ID;
		}
		
		g_device_config.kalman_q = 3;
		g_device_config.kalman_r = 10;
		g_device_config.ant_rx_delay = RX_ANT_DLY;
		g_device_config.ant_tx_delay = TX_ANT_DLY;
		g_device_config.dyn_slot_long = SLOT_LONG;
		g_device_config.ranging_slot_long = RANGING_SLOT_LONG;
		g_device_config.position[0] = 0; 
		g_device_config.position[1] = 0;
		g_device_config.position[2] = 0; 
		g_device_config.tx_power = 0xc0c0c0c0; //增益配置为0
		g_device_config.anchor_idle_num = 1;
		g_device_config.anchor_h = 0;
		g_device_config.tag_h = 0;
		g_device_config.chan = 2;

}


//获取设备参数
void get_device_para(void)
{
	FILE* fd = 0;
	u16 crc;
	u16 len;
	u8 buf[100];

	len = sizeof(device_config_t) + 2;
	
	if((fd = fopen(POS_PARA_FILE, "r")) != NULL)
	{
		fread(buf, len, 1, fd);
		fclose(fd);

		crc = crc16(buf, sizeof(device_config_t));
		if(crc == ((buf[len-2]<<8)|buf[len-1]))
		{
				g_device_config = *(device_config_t*)&buf[0];
	 
				if(g_device_config.device_type == TAG)
				{
					Flash_Device_ID = g_device_config.tag_id;
				}
				else if(g_device_config.device_type == ANCHOR)
				{
					Flash_Device_ID = g_device_config.anchor_id;
				}

		}
		else
		{
				DBG_PRINT(("数据库校验错误，恢复缺省定位相关数据!\r\n"));
				reset_position_default_para();			
				save_device_para();
		}

		
	}
	else
	{
		DBG_PRINT(("打开文件失败，恢复缺省定位相关数据!\r\n"));
		reset_position_default_para();		
		save_device_para();	
	}

	g_device_config.dyn_slot_long = SLOT_LONG;
	g_device_config.ranging_slot_long = RANGING_SLOT_LONG;

	
}


//显示设备参数
void show_position_para(void)
{

	GLOBAL_PRINT(("device_type: %d\r\n", g_device_config.device_type));

	if(g_device_config.device_type == TAG)
	{
		GLOBAL_PRINT(("tag_id: %d\r\n", g_device_config.tag_id));
		GLOBAL_PRINT(("tag_h: %lf\r\n", g_device_config.tag_h));
	}
	else if(g_device_config.device_type == ANCHOR)
	{
		GLOBAL_PRINT(("anchor_id: %d\r\n", g_device_config.anchor_id));	
		GLOBAL_PRINT(("on_left: %d\r\n", g_device_config.on_left));
		GLOBAL_PRINT(("anchor_h: %lf\r\n", g_device_config.anchor_h));
		GLOBAL_PRINT(("anchor_idle_num: %d\r\n", g_device_config.anchor_idle_num));		
		GLOBAL_PRINT(("position:latitude %lf, longitude %lf, height %lf\r\n", g_device_config.position[0], g_device_config.position[1], g_device_config.position[2]));	
	}

	GLOBAL_PRINT(("ant_tx_delay: %d\r\n", g_device_config.ant_tx_delay));
	GLOBAL_PRINT(("ant_rx_delay: %d\r\n", g_device_config.ant_rx_delay));	
	GLOBAL_PRINT(("tx_power: %2.1f db\r\n", calc_tx_power_config_value_to_db(g_device_config.tx_power)));	
	GLOBAL_PRINT(("dyn_slot_long: %d\r\n", g_device_config.dyn_slot_long));
	GLOBAL_PRINT(("ranging_slot_long: %d\r\n", g_device_config.ranging_slot_long));	
	GLOBAL_PRINT(("chan: %d\r\n", g_device_config.chan));	
}


static float calculatePower(float base, float N, uint8_t pulseFrequency) {
  float A, corrFac;

    if(DWT_PRF_16M == pulseFrequency) {
        A = 115.72;
        corrFac = 2.3334;
    } else {
        A = 121.74;
        corrFac = 1.1667;
    }

    float estFpPwr = 10.0 * log10(base / (N * N)) - A;

    if(estFpPwr <= -88) {
        return estFpPwr;
    } else {
        // approximation of Fig. 22 in user manual for dbm correction
        estFpPwr += (estFpPwr + 88) * corrFac;
    }

    return estFpPwr;
}

float dwGetReceivePower(void) {
    dwt_rxdiag_t *diagnostics;
    dwt_readdiagnostics(diagnostics);
//  float C = (&diagnostics->stdNoise)[3];
	float J = (&diagnostics->stdNoise)[3];
  float N = diagnostics->rxPreamCount;

  float twoPower17 = 131072.0;
  //return calculatePower(C * twoPower17, N, config.prf);
	return calculatePower(J * twoPower17, N, config.prf);
}



static void rx_ok_cb(const dwt_callback_data_t *cb_data)
{
    int i;		


		if((cb_data->event == DWT_SIG_RX_ERROR) || (cb_data->event == DWT_SIG_RX_PHR_ERROR) || (cb_data->event == DWT_SIG_RX_SYNCLOSS) || \
			(cb_data->event == DWT_SIG_RX_SFDTIMEOUT) || (cb_data->event == DWT_SIG_RX_PTOTIMEOUT))
		{
			g_rx_state = DWT_RX_ERR;
			DBG_PRINT("rx_ok_cb DWT_RX_ERR value is %d\r\n", cb_data->event);	
		}
		else if(cb_data->event == DWT_SIG_RX_TIMEOUT)
		{
			g_uwb_rg_ssi = 0;
			g_rx_state = DWT_RX_TIMEOUT;
			
		}
		else if(cb_data->event == DWT_SIG_RX_OKAY)
		{	
			g_rx_state = DWT_RX_OK; 
		  for (i = 0 ; i < FRAME_LEN_MAX; i++ )
		  {
		      g_rx_buf[i] = 0;
		  }

		  /* A frame has been received, copy it to our local buffer. */
		  if (cb_data->datalength <= FRAME_LEN_MAX)
		  {
		  		g_rx_len = cb_data->datalength; 
		      dwt_readrxdata(g_rx_buf, cb_data->datalength, 0);	

					if(g_get_rx_timestamp_flag == 1)
					{
						g_rx_timestamp = get_rx_timestamp_u64();
					}
					g_get_rx_timestamp_flag = 0;
					
					//DBG_PRINT("rx_ok_cb DWT_SIG_RX_OKAY value is %d\r\n", cb_data->event);
		  }
			else
			{
				DBG_PRINT("cb_data->datalength is %d\r\n", cb_data->datalength);
			}

		}
		else
		{
		
			DBG_PRINT("rx_ok_cb other env  is %d\r\n", cb_data->event);
		}

		if(g_pos_info.tag_state == TAG_WAITE_ACTIVATION || g_pos_info.anchor_sub_state == SA_NET_ACTIVATION_ALL_TAG || \
			g_ranging_flag == 1 || g_dwt_auto_tx_power_config.auto_choose_tx_power == 1 || g_detection_signal_flag == 1)	
		{
			g_uwb_rg_ssi = dwGetReceivePower();
		}				

}


static void tx_conf_cb(const dwt_callback_data_t *cb_data)
{
	if(cb_data->event == DWT_SIG_TX_DONE)
	{
		if(g_get_tx_timestamp_flag == 1)
		{
			g_tx_timestamp = get_tx_timestamp_u64();
		}
		g_get_tx_timestamp_flag = 0;
		g_tx_state = DWT_TX_DONE;
	}
	else if(cb_data->event == DWT_SIG_TX_ERROR)
	{
		DBG_PRINT("tx_conf_cb other env is %d\r\n", cb_data->event);
		g_tx_state = DWT_TX_ERR;
	}
	else
	{
		DBG_PRINT("tx_conf_cb other env is %d\r\n", cb_data->event);
	}

}


/******************************************************************************
												         DWM100初始化
*******************************************************************************/
void DWM1000_init(void)
{
	
		config.chan=g_device_config.chan;
//		config.dataRate=FLASH_Data_rat;	

	reset_DW1000();//重启DW1000 
    GLOBAL_PRINT(("DWMstep 1\r\n"));
    spi_set_rate_low();//降低SPI频率
    GLOBAL_PRINT(("DWMstep 2\r\n"));
    dwt_initialise(DWT_LOADUCODE);//初始化DW1000
	GLOBAL_PRINT(("DWMstep 3\r\n"));
    dwt_write32bitreg(TX_POWER_ID, g_device_config.tx_power);
	GLOBAL_PRINT(("DWMstep 4\r\n"));
    spi_set_rate_high();//回复SPI频率
    GLOBAL_PRINT(("DWMstep 5\r\n"));
    dwt_configure(&config);//配置DW1000
   // dwt_setrxantennadelay(RX_ANT_DLY);		//设置接收天线延迟
   // dwt_settxantennadelay(TX_ANT_DLY);		//设置发射天线延迟
    dwt_setrxantennadelay(g_device_config.ant_tx_delay);		//设置接收天线延迟
    dwt_settxantennadelay(g_device_config.ant_rx_delay);		//设置发射天线延迟  
     
	  dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_TFRS | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO), 1);
	//dwt_setinterrupt(DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO), 1);//无发送中断
	//	dwt_setinterrupt(DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO), 1);//无发送中断,前导码超时中断
	dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb);
    GLOBAL_PRINT(("DWMstep 8\r\n"));
}
/******************************************************************************
												         检测DWM1000配置参数是否被改变
*******************************************************************************/
void DWM1000_check(void)
{
	u8 DWM1000_SET_EN=0;

		if(DWM1000_SET_DATA[3]!=g_device_config.device_type)//检查模块角色是否被改变
	{
		DWM1000_SET_DATA[3]=g_device_config.device_type;
		DWM1000_SET_EN=1;
	}

  if(DWM1000_SET_EN==1) DWM1000_init(); //如果被改变，执行初始化
	
}




/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}


u32 calc_tx_power_config_value(u8* ptr)
{
	char *p_end;
	float db, temp;
	u32 reg_value;
	u32 tx_power, tx_power_temp;

	//BOOSTP125
	db = GLOBAL_STRTOD(ptr, &p_end);
	temp = db - (u32)db;
	if(temp != 0.5) 
	{
		db -= temp;
	}		
	
	if(db <= 18.0)
	{ 	
		tx_power_temp = ((u32)(6-(((u32)db)/3))<<5) + (u32)((db-((((u32)db)/3)*3))*2);	
	}
	else
	{
		tx_power_temp = (u32)((db - 18)*2);
	}
	
	tx_power = tx_power_temp<<24;
	
	//BOOSTP250
//	db = GLOBAL_STRTOD(p_end, &p_end);
	if(db <= 18.0)
	{
		tx_power_temp = ((u32)(6-(((u32)db)/3))<<5) + (u32)((db-((((u32)db)/3)*3))*2);	
	}
	else
	{
		tx_power_temp = (u32)((db - 18)*2);
	}
	
	tx_power |= tx_power_temp<<16;	
	
	
	//BOOSTP500
//	db = GLOBAL_STRTOD(p_end, &p_end);
	if(db <= 18.0)
	{
		tx_power_temp = ((u32)(6-(((u32)db)/3))<<5) + (u32)((db-((((u32)db)/3)*3))*2);	
	}
	else
	{
		tx_power_temp = (u32)((db - 18)*2);
	}
	
	tx_power |= tx_power_temp<<8; 
	
	//BOOSTNORM
//	db = GLOBAL_STRTOD(p_end, &p_end);
	if(db <= 18.0)
	{
		tx_power_temp = ((u32)(6-(((u32)db)/3))<<5) + (u32)((db-((((u32)db)/3)*3))*2);		
	}
	else
	{
		tx_power_temp = (u32)((db - 18)*2);
	}
	
	tx_power |= tx_power_temp;	



}


float calc_tx_power_config_value_to_db(u32 value)
{
	u8 hight, low, config_value;
	float db;

	config_value = (value & 0x00ff0000) >> 16;
	hight = config_value & (~0x1f);
	hight = hight >> 5;
	low = config_value & 0x1f;

	db = 18 - 3.0*hight;
	db += (low/2.0);

	return db;
	
}




/*注：系统时间计数器总共有40位，低9位通常为0，计时频率为499.2 MHz×128（63.8976 GHz），即每15.65ps累加一次，15.65ps * 0x618000 = 99.999us
	dw1000延迟发送时间要大于300us，否则会发送失败
*/
ret_t dm9000_send_data(u8 *buf, u16 len, u8 mode, u32 delay_tx_time, u32 timeout_us)
{	
	ret_t ret = RET_SUCCESS;
	u8 date;
	u32 time_start;
	u32 time_end;
	u32 nus;
	u32 flags;
	u32 tx_time;


	time_start = get_cur_time();
	dwt_forcetrxoff();
	dwt_rxreset();

	osEventFlagsClear (evt_id, DM1000_TXRX_FLAG);
	g_tx_state = DWT_TX_NO_DONE;

	dwt_writetxdata(len, buf, 0);//写入发送数据
	dwt_writetxfctrl(len, 0);//设定发送长度

	if(mode == DWT_START_TX_DELAYED)
	{
			//tx_time = dwt_readsystimestamphi32() + delay_tx_time;
			tx_time = delay_tx_time;
			dwt_setdelayedtrxtime(tx_time);
	}		

	dwt_starttx(mode);

	time_end = get_cur_time();
	nus = get_count_time(time_start, time_end);

	time_start = get_cur_time();
	
	if(timeout_us != osWaitForever)
			flags = osEventFlagsWait(evt_id,DM1000_TXRX_FLAG,osFlagsWaitAny, (timeout_us/100));
	else
			flags = osEventFlagsWait(evt_id,DM1000_TXRX_FLAG,osFlagsWaitAny, osWaitForever);

	if(flags == osFlagsErrorTimeout)
	{
		dwt_forcetrxoff(); //this will clear all events
		dwt_rxreset();	
		DBG_PRINT("poll sent timeout\r\n");
		ret = RET_FAILED;
		goto end;
	}
	
	time_end = get_cur_time();
	nus = get_count_time(time_start, time_end);

	dwt_isr();
	
	if(g_tx_state == DWT_TX_DONE)
	{
		time_end = get_cur_time();
		nus = get_count_time(time_start, time_end);
	//	DBG_PRINT("sent ok\r\n");
		goto end;
	}
	else if(g_tx_state == DWT_TX_ERR)
	{		
		time_end = get_cur_time();
		nus = get_count_time(time_start, time_end);		
		ret = RET_FAILED;
		DBG_PRINT("sent failed\r\n");
		goto end;
	}
	else
	{
		ret = RET_FAILED;
		DBG_PRINT("sent other failed\r\n");
	}
	
end:
		if(ret != RET_SUCCESS)
		{
			DWM1000_init();
		}

    return ret;	
	 
}




/**
* 功能：dm9000接受数据 
* 参数：
* 	rx_buf：接受数据缓存
* 返回：接受到的数据长度
**/
/*注：系统时间计数器总共有40位，低9位通常为0，计时频率为499.2 MHz×128（63.8976 GHz），即每15.65ps累加一次，15.65ps * 0x618000 = 99.999us*/
ret_t dm9000_recv_data(u8 *rx_data_buf, u16 *rx_data_len, u8 open_recv, u32 rx_delay_time, u32 timeout_us)
{
	u32 status_reg = 0;
	u32 len;
	ret_t ret = RET_SUCCESS;
	u32 time_start;
	u32 time_end;
	u32 nus;
	u32 flags;
	u8 date;
	float time;
	u32 rx_time;


	g_rx_state = DWT_RX_NO_DONE;
	g_rx_len = 0;

	dwt_forcetrxoff();
	dwt_rxreset();

	time_start = get_cur_time();
	osEventFlagsClear(evt_id, DM1000_TXRX_FLAG);

	if(timeout_us < 300)
	{
		DBG_PRINT("recv failed, timeout time is too short\r\n");
	}
	
	
	if(open_recv == OPEN_RECV)
	{
		//dwt_setrxtimeout(0);//设定接收超时时间，0位没有超时时间
		if(timeout_us != osWaitForever)
				dwt_setrxtimeout(timeout_us);
		else
				dwt_setrxtimeout(0);//设定接收超时时间，0位没有超时时间


		if(rx_delay_time > 0)
		{
			rx_time = rx_delay_time;
			dwt_setdelayedtrxtime(rx_time);
			dwt_rxenable(1); //延迟接受
		}
		else
		{
			dwt_rxenable(0);//打开接收
		}
	}


	if(timeout_us != osWaitForever)
	{
		time = timeout_us * 1.0256;
		timeout_us = (u32)time;
		timeout_us += 100; //防止dw1000故障无法产生接受超时中断
	}

		time_end = get_cur_time();
		nus = get_count_time(time_start, time_end);	
	
	time_start = get_cur_time();

	if(timeout_us != osWaitForever)
			flags = osEventFlagsWait(evt_id,DM1000_TXRX_FLAG,osFlagsWaitAny, (timeout_us/100));
	else
			flags = osEventFlagsWait(evt_id,DM1000_TXRX_FLAG,osFlagsWaitAny, osWaitForever);
	if(flags == osFlagsErrorTimeout)
	{
		time_start = get_cur_time();	
		dwt_forcetrxoff(); //this will clear all events
		dwt_rxreset();
		time_end = get_cur_time();
		nus = get_count_time(time_start, time_end);
		DBG_PRINT("recv failed, no recv irq\r\n");
		ret = RECV_TIME_OUT;
		goto end;
	}

	time_end = get_cur_time();
	nus = get_count_time(time_start, time_end);	

//	date = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3);
//	date =	dwt_checkIRQ();


	time_start = get_cur_time();			
	dwt_isr();	

	if(g_rx_state == DWT_RX_ERR)
	{
		time_end = get_cur_time();
		nus = get_count_time(time_start, time_end);		
		ret = RECV_HIT;
		DBG_PRINT("recv hit\r\n");
		goto end;
	}
	else if(g_rx_state == DWT_RX_OK)
	{
		if(g_rx_len > 0)
		{
			memcpy(rx_data_buf, g_rx_buf, g_rx_len);
			*rx_data_len = g_rx_len;
				time_end = get_cur_time();
			nus = get_count_time(time_start, time_end);		
		}
		else
		{ 	
			DBG_PRINT("g_rx_len is 0\r\n");
			ret = RET_FAILED;
			goto end;
		} 		
	}
	else if(g_rx_state == DWT_RX_TIMEOUT)
	{	
		DBG_PRINT("recv timeout\r\n");
		ret = RECV_TIME_OUT;
		goto end;
	} 
	else
	{
		ret = RET_FAILED;
		DBG_PRINT("recv other failed\r\n");
	}



end:

	return ret;


}



ret_t make_package(dm_comm_t cmd, u16 src_id, u16 target_id, u8 src_device_type, u8 src_device_sub_type, u8* buf, u16 buf_len, u8* package_buf, u16 *package_buf_len)
{
	ret_t ret = RET_SUCCESS;
	u32 crc;

	package_buf[0] = src_id & 0xff; 
	package_buf[1] = (src_id >> 8) & 0xff;
	package_buf[2] = target_id & 0xff;		
	package_buf[3] = (target_id >> 8) & 0xff;
	package_buf[4] = (src_device_type << 6) | src_device_sub_type;
	package_buf[5] = cmd;	
	package_buf[6] = buf_len;

	if(buf_len > 0)
		memcpy(&package_buf[7], buf, buf_len);
	
	*package_buf_len = 7 + buf_len;
	

	crc = crc16(package_buf, *package_buf_len);
	package_buf[*package_buf_len] = crc/256;
	*package_buf_len = *package_buf_len + 1;
	package_buf[*package_buf_len] = crc%256;
	*package_buf_len = *package_buf_len + 1;
	*package_buf_len = *package_buf_len + 2; //dw1000中要使用2位作为校验位
	

end:
	return ret;


}



ret_t parsing_package(u8* package_buf, u16 package_buf_len, u8 *cmd, u16 *src_id,u16 *target_id,  u8 *src_device_type, u8 *src_device_sub_type, u8* buf, u16 *buf_len)
{
	u32 crc;
	ret_t ret = RET_SUCCESS;
	

	package_buf_len -= 2;	//减掉2位dw1000添加的校验位
	crc = crc16(package_buf, (package_buf_len-2));
	if(crc != ((package_buf[package_buf_len-2]<<8)|package_buf[package_buf_len-1]))
	{
		ret = RET_FAILED;
		goto end;
	}

	
	*src_id =(package_buf[1]<<8) | package_buf[0];
	*target_id = (package_buf[3]<<8) | package_buf[2];
	*src_device_type = package_buf[4] >> 6; 
	*src_device_sub_type = package_buf[4] & 0x3f; 
	*cmd = package_buf[5];

	if(buf == NULL || buf_len == NULL)
	{
		goto end;
	}

	*buf_len = package_buf[6];
	
	memcpy(buf, &package_buf[7], *buf_len);

end:
	return ret;

}





void detection_signal(void)
{	
	u8 package_buf[PACKAGE_BUF_SIZE];
	u8 buf[300] = {0};
	u16 package_buf_len;
	u16 buf_len;
	u16 i;
	u16 cnt = 0;
	u16 time_out_cnt = 0;
	u16 suc_rcv_cnt = 0;
	float suc_rcv_rate = 0.0;
	ret_t ret = RET_SUCCESS;
	
	while(1)
	{
		if(g_detection_signal_flag == 0)
			return;
	
		ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, 50000);		
		if(ret != RET_SUCCESS)
		{ 
			if(ret != RECV_TIME_OUT)
			{
				time_out_cnt = 0;
				DBG_PRINT("fail: RSSI:%2.2f, signal is too little\r\n",g_uwb_rg_ssi);
			}
			else
			{
				cnt = 0;
				suc_rcv_cnt = 0;
				DBG_PRINT("no signal\r\n");
				time_out_cnt++;
				if(time_out_cnt > 200)//10s
				{
					time_out_cnt = 0;
					suc_rcv_rate = 0.0;
					sprintf((u8*)&g_key_vaule.key_value_buf[0], "recv_suc_rate:%2.1f,current_rssi:no signal", suc_rcv_rate);
					g_key_vaule.key_vaule_valid_flag = 1;
					DBG_PRINT("%s\r\n", g_key_vaule.key_value_buf);
				}
				continue;			
			}
				
		}
		else
		{
			time_out_cnt = 0;
			suc_rcv_cnt++;
			DBG_PRINT("suc: RSSI:%2.2f, package_buf_len: %d\r\n",g_uwb_rg_ssi, package_buf_len);
			buf_len = 0;
			for (i = 0; i < package_buf_len; i++)
			{
				buf_len += sprintf((u8*)&buf[0] + buf_len, "%x ", package_buf[i]);
			}		
			buf[buf_len] = 0;
			DBG_PRINT("data: %s\r\n", buf);
		}	


		cnt++;
		if(cnt >= 500) //10s
		{
			suc_rcv_rate = suc_rcv_cnt/500.0; 
			suc_rcv_rate *= 100;		
			
			sprintf((u8*)&g_key_vaule.key_value_buf[0], "recv_suc_rate:%2.1f,current_rssi:%2.2f", suc_rcv_rate, g_uwb_rg_ssi);
			DBG_PRINT("%s\r\n",g_key_vaule.key_value_buf);
			cnt = 0;
			suc_rcv_cnt = 0;
			g_key_vaule.key_vaule_valid_flag = 1;
		}

		
	}

}

void interference_signal(void)
{
	u8 package_buf[PACKAGE_BUF_SIZE];
	u16 package_buf_len;
	ret_t ret = RET_SUCCESS;

	while(1)
	{
		if(g_interference_signal_flag == 0)
			return;

		delay_ms(20);
	
		ret = make_package(INTERFERENCE_SIGNAL, 0xffff, 0xffff, TAG, TAG, NULL, 0, package_buf, &package_buf_len);
		if(ret != RET_SUCCESS)
		{
			continue;
		} 
		
		ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);
		if(ret != RET_SUCCESS)
		{
			DBG_PRINT(" data single send fail,ret=%s\r\n",ret_type_str[ret]);
			continue;
		}
		else
		{
			DBG_PRINT(" data single send suc\r\n");
		}	
		
	}	

}




u32 m_active_sn = 0;
ret_t m_activation_all_tag(void)
{
		u8 package_buf[PACKAGE_BUF_SIZE];
		u8 buf[BUF_SIZE] = {0};
		u16 package_buf_len;
		u16 buf_len;
		u16 src_id, target_id;
		u32 time_start, time_end, nus, timeout;
		u32 time_start2, time_end2, nus2, timeout2;
		u32 offset_time;
		u8 cmd;
		u8 src_device_type;
		u8 src_device_sub_type; 
		ret_t ret = RET_SUCCESS;
		u32 delay_tx_time;
		
		
		time_start = get_cur_time();

		while(1)
		{
			time_end = get_cur_time();
			nus = get_count_time(time_start, time_end);

		  osDelay(10);
		
			if(nus > ACTIVE_TAG_TIMEOUT)
			{			
				ret = RET_FAILED;
				goto end;
			}	

			//同步计时器时间
			g_time_end = dwt_readsystimestamphi32();
			if(g_time_end >= g_time_start)
			{
				offset_time = g_time_end - g_time_start;
			}
			else
			{
				offset_time = (0xffffffff - g_time_start) + g_time_end + 1;
			}
	
			delay_tx_time = g_time_end + DW1000_100US_TIMER_CNT*10;//1ms后发送	
			offset_time += DW1000_100US_TIMER_CNT*10;

			buf[0] = m_active_sn & 0xff;
			buf[1] = m_active_sn >> 8;
			buf[2] = m_active_sn >> 16;
			buf[3] = m_active_sn >> 24;			
			buf[4] = offset_time >> 24;
			buf[5] = offset_time >> 16;
			buf[6] = offset_time >> 8;
			buf[7] = offset_time & 0xff;
			memcpy(&buf[8], &g_device_config.position[0], 8);
			memcpy(&buf[16], &g_device_config.position[1], 8);
			memcpy(&buf[24], &g_device_config.position[2], 8);
			buf[32] = g_device_config.on_left;
			memcpy(&buf[33], (u8*)&g_device_config.anchor_h, 4);

			src_id = Flash_Device_ID;
			target_id = Flash_Device_ID + 1;

			time_start2 = get_cur_time();	
			//发送激活标签帧
			ret = make_package(ACTIVATION_ALL_TAG_CMD, src_id, target_id, ANCHOR, MAIN_ANCHOR, buf, 37, package_buf, &package_buf_len);
			if(ret != RET_SUCCESS)
			{
				continue;
			} 
		
			ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_DELAYED, delay_tx_time, (TX_NORMAL_TIMEOUT+1000));
			if(ret != RET_SUCCESS)
			{
				DBG_PRINT("MA m_active data single send fail,ret=%s\r\n",ret_type_str[ret]);
				continue;
			}

				
			time_end2 = get_cur_time();
			nus2 = get_count_time(time_start2, time_end2); 
			DBG_PRINT("MA m_active data single send suc! sn=%u\r\n",m_active_sn);


			m_active_sn++;
			
			//接受从基站激活标签帧
			ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, (DM9000_RX_TIMEOUT+1000));
			if(ret != RET_SUCCESS)
			{		
				DBG_PRINT("MA s_ctive data rcv failed0,ret=%s\r\n",ret_type_str[ret]);
				continue;
			}

				
			ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, NULL, NULL);
			if(!((ret == RET_SUCCESS) && (cmd == ACTIVATION_ALL_TAG_CMD) && \
				(target_id == Flash_Device_ID) && (src_id == (Flash_Device_ID+1))&& \
				(src_device_type == ANCHOR) && (src_device_sub_type == SUB_ANCHOR)))
			{
				DBG_PRINT("MA s_active data rcv failed1: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
											ret_type_str[ret],
											dm_comm_type_str[cmd],
											target_id,src_id,
											device_type_str[src_device_type],
											anchor_type_str[src_device_sub_type],buf_len);
				continue;
			}
			else
			{
				
				time_end2 = get_cur_time();
				nus2 = get_count_time(time_start2, time_end2); 	
				DBG_PRINT("-------------------nus2 is %8x\r\n", nus2);
				ret = RET_SUCCESS;
				DBG_PRINT("MA s_active data rcv suc! sid:%d , tid:%d\r\n",src_id,target_id);
				osDelay(1);
				goto end;
			}

		}

end:
		return ret;


}



ret_t s_wait_activation_all_tag(void)
{

		u8 package_buf[PACKAGE_BUF_SIZE];
		u8 buf[BUF_SIZE] = {0};
		u16 package_buf_len;
		u16 buf_len;
		u16 src_id, target_id;
		u32 time_start, time_end, nus, timeout;
		u32 main_time_start,main_time_cur;
		u32 time_start2, time_end2, nus2, timeout2;
		u32 cur_time, start_time;
		u32 offset_time;
		u8 cmd;
		u8 src_device_type;
		u8 src_device_sub_type;
		u32 sn;
		float uwb_rssi;
		
		ret_t ret = RET_SUCCESS;

		
		time_start = get_cur_time();

		while(1)
		{
			time_end = get_cur_time();
			nus = get_count_time(time_start, time_end);
			if(nus > ACTIVE_TAG_TIMEOUT)
			{			
				DBG_PRINT("SA active timeout! %d/%d (unit:1us)\r\n",nus,ACTIVE_TAG_TIMEOUT);
				ret = RET_FAILED;
				goto end;
			}	
			
			//接受主基站激活帧
			ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, (ACTIVE_TAG_TIMEOUT - nus));
			if(ret != RET_SUCCESS)
			{	
				DBG_PRINT("SA active data rcv failed0! ret=%s\r\n",ret_type_str[ret]);
				continue;
			}			
		
			ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
			if(!((ret == RET_SUCCESS) && (cmd == ACTIVATION_ALL_TAG_CMD) && \
				(target_id == Flash_Device_ID) && (src_id == (Flash_Device_ID-1))&& \
				(src_device_type == ANCHOR) && (src_device_sub_type == MAIN_ANCHOR) && (buf_len == 37)))
			{
				DBG_PRINT("SA active data rcv failed1: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
											ret_type_str[ret],
											dm_comm_type_str[cmd],
											target_id,src_id,
											device_type_str[src_device_type],
											anchor_type_str[src_device_sub_type],buf_len);
				
				
				if(((ret == RET_SUCCESS) && (cmd == AUTO_CHOOSE_TX_POWER) && (src_id == 0xffff) && (target_id == 0xfffe) &&\
					(src_device_type == ANCHOR) && (src_device_sub_type == MAIN_ANCHOR) && (buf_len == 17)))
				{
					//接受到AUTO_CHOOSE_TX_POWER命令
					auto_choose_slave_tx_power();
				}
				else if(((ret == RET_SUCCESS) && (cmd == POLL) &&\
					(src_device_type == TAG) && (src_device_sub_type == TAG) && (buf_len == 1)))
				{
					//接受到START_TEST_RANGING命令
					if(buf[0] == START_TEST_RANGING)
					{
						anchor_test_ranging();
					}
					
				}
											
				continue;
			}

			sn = buf[0] + (buf[1] << 8) + (buf[2] <<16) + (buf[3] <<24);
			DBG_PRINT("main anchor src_id: %d, sn: %u\r\n", src_id, sn);	

			uwb_rssi = g_uwb_rg_ssi;

			//同步开始计数时间
			offset_time = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];
			offset_time += 17472; //0.7*0x6180 = 17472,表示70us的计数
			cur_time = dwt_readsystimestamphi32();			
			start_time = cur_time - offset_time;	
			if(cur_time > start_time)
			{
				g_time_start = start_time;
			}
			else
			{
				g_time_start = 0xffffffff - offset_time + cur_time;
			}
			
			
			//激活所有标签	
			memcpy(&buf[0], &g_device_config.position[0], 8);
			memcpy(&buf[8], &g_device_config.position[1], 8);
			memcpy(&buf[16], &g_device_config.position[2], 8);
			memcpy(&buf[24], (u8*)&g_device_config.anchor_h, 4);

			target_id = Flash_Device_ID - 1;
			src_id = Flash_Device_ID;	


			delay_us(100);


			//激活所有标签
			ret = make_package(ACTIVATION_ALL_TAG_CMD, src_id, target_id, ANCHOR, SUB_ANCHOR, buf, 28, package_buf, &package_buf_len);
			if(ret != RET_SUCCESS)
			{
				continue;
			} 

			time_start2 = get_cur_time();
			ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);
			if(ret != RET_SUCCESS)
			{
				DBG_PRINT("SA active data send failed! ret=%s\r\n",ret_type_str[ret]);
				continue;
			}
			else
			{
				DBG_PRINT("SA active data send suc!\r\n");
				time_end2 = get_cur_time();
				nus2 = get_count_time(time_start2, time_end2); 	
				//DBG_PRINT("-------------------nus2 slave is %8x\r\n", nus2);	
				DBG_PRINT("-------------------RSSI:%2.2f\r\n", uwb_rssi);	
				goto end;
			}

		}

end:	
		return ret;

}




ret_t tag_wait_activation(void)
{
	u8 package_buf[PACKAGE_BUF_SIZE];
	u8 buf[BUF_SIZE] = {0};
	u16 package_buf_len;
	u16 buf_len;
	u16 src_id, target_id;
	u32 time_start, time_end, nus, timeout;
	u32 syn_time;
	u8 cmd;
	u8 src_device_type;
	u8 src_device_sub_type; 
	u8 state = 0;
	u32 sn;
	float uwb_rg_ssi;
	u8 main_anchor_activ_fail_cnt = 0;
	static u8 have_anchor_flag = 0;
	ret_t ret = RET_SUCCESS;

	dwt_forcetrxoff();
	dwt_rxreset();
	
	while(1)
	{
		switch(state)
		{
			case 0:
			{	

				if(g_dwt_auto_tx_power_config.auto_choose_tx_power == 1)
				{
					auto_choose_main_tx_power();
				}		
				else if(g_ranging_flag == 1)
				{
					tag_test_ranging();
				}
				else if(g_detection_signal_flag == 1)
				{
					detection_signal();
				}
				else if(g_interference_signal_flag == 1)
				{
					interference_signal();
				}				

				if(have_anchor_flag == 1)
				{
					ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, TAG_WAIT_TIMEOUT);
					if(ret != RET_SUCCESS)
					{ 
						if(ret == RECV_TIME_OUT)
						{
							main_anchor_activ_fail_cnt++;
							if(main_anchor_activ_fail_cnt >= 25) //25*TAG_WAIT_TIMEOUT
							{
								g_pos_info.tag_position_valid_flag = POS_INVALID;	
								have_anchor_flag = 0;
								DBG_PRINT("tag_position_valid_flag = POS_INVALID");
							}
						}
						DBG_PRINT("main anchor actived failed0[%1d],\r\n",ret);
						continue;
					}						
				}
				else if(have_anchor_flag == 0)
				{					
					ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, osWaitForever); //一直等待接受
					if(ret != RET_SUCCESS)
					{ 
						DBG_PRINT("main anchor actived failed0[%1d],\r\n",ret);
						continue;
					}	
					have_anchor_flag = 1;
				}

				main_anchor_activ_fail_cnt = 0;
			
				ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
				if(!((ret == RET_SUCCESS) && (cmd == ACTIVATION_ALL_TAG_CMD) && \
					(src_device_type == ANCHOR) && (src_device_sub_type == MAIN_ANCHOR) && (buf_len == 37) && (g_uwb_rg_ssi >= -92.0)))
				{
					state = 0;
					DBG_PRINT("main anchor actived rcv failed1: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
												ret_type_str[ret],
												dm_comm_type_str[cmd],
												target_id,src_id,
												device_type_str[src_device_type],
												anchor_type_str[src_device_sub_type],buf_len);

					//如果接受到AUTO_CHOOSE_TX_POWER命令
					if(((ret == RET_SUCCESS) && (cmd == AUTO_CHOOSE_TX_POWER) && (src_id == 0xffff) && (target_id == 0xfffe) &&\
						(src_device_type == ANCHOR) && (src_device_sub_type == MAIN_ANCHOR) && (buf_len == 17)))
					{
						auto_choose_slave_tx_power();
					}
					else if(((ret == RET_SUCCESS) && (cmd == POLL) &&\
						(src_device_type == TAG) && (src_device_sub_type == TAG) && (buf_len == 1)))
					{
						//接受到START_TEST_RANGING命令
						if(buf[0] == START_TEST_RANGING)
						{
							anchor_test_ranging();
						}				
					}

												
				}
				else
				{
					sn = buf[0] + (buf[1] << 8) + (buf[2] <<16) + (buf[3] <<24);
					g_locate_net_info.main_anchor_id = src_id;
					g_locate_net_info.anchor_position[0] = *(double*)&buf[8];
					g_locate_net_info.anchor_position[1] = *(double*)&buf[16]; 
					g_locate_net_info.anchor_position[2] = *(double*)&buf[24];
					g_locate_net_info.on_left = buf[32];
					memcpy((u8*)&g_locate_net_info.main_anchor_h, &buf[33], 4);
					
					state = 1;
					
					DBG_PRINT("main anchor [%3d] actived suc, sn: %u ,RSSI:%2.2f\r\n", src_id, sn,g_uwb_rg_ssi);
					
				}
				
				break;
			}
			case 1:
			{
				ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT+200);
				if(ret != RET_SUCCESS)
				{ 
					state = 0;
					
					DBG_PRINT("sub anchor active is failed0! ret=%s\r\n",ret_type_str[ret]);
					if(g_locate_net_info.main_anchor_id == 0)
						DBG_PRINT("sub anchor active is failed0! ret=%s\r\n",ret_type_str[ret]);
					
					continue;
				}	
				
				ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
 				if(!((ret == RET_SUCCESS) && (cmd == ACTIVATION_ALL_TAG_CMD) && \
					(src_device_type == ANCHOR) && (src_device_sub_type == SUB_ANCHOR) && ((g_locate_net_info.main_anchor_id+1) == src_id) && \
					(buf_len == 28) && (g_uwb_rg_ssi >= -92.0)))
				{
					if(g_locate_net_info.main_anchor_id == 0)
						DBG_PRINT("sub anchor [%3d] actived failed1,%1d,%1d,%1d,%1d,%3d\r\n", src_id,ret,cmd,src_device_type,src_device_sub_type,buf_len);
					state = 0;
				}
				else
				{
					g_locate_net_info.sub_anchor_id = src_id;
					g_locate_net_info.anchor_position[3] = *(double*)&buf[0];
					g_locate_net_info.anchor_position[4] = *(double*)&buf[8]; 
					g_locate_net_info.anchor_position[5] = *(double*)&buf[16];
					memcpy((u8*)&g_locate_net_info.sub_anchor_h, &buf[24], 4);
					
					DBG_PRINT("sub anchor [%3d] actived suc, RSSI:%2.2f\r\n", src_id,g_uwb_rg_ssi);
					
					ret = RET_SUCCESS;
					goto end;
				}				

				break;
			}

		}
	}


end:
		return ret;

}




ret_t m_wait_tag_register(void)
{	
		u8 package_buf[PACKAGE_BUF_SIZE];
		u8 buf[BUF_SIZE] = {0};
		u8 buf2[BUF_SIZE] = {0};
		u8 buf3[BUF_SIZE] = {0};
		u16 buf_len, data_len;
		u16 package_buf_len;
		u8 dynamic_slot;
		u16 target_id;
		u16 src_id;
		u16 id;
		u8 src_device_type;
		u8 src_device_sub_type; 
		u8 cmd;
		u8 cnt;
		u8 temp;
		u32 time_start, time_end, nus, timeout, offset_time;
		u32 time_start0, time_end0, nus0, timeout0;
		u32 time_start1, time_end1, nus1, timeout1;
		u32 time_start2, time_end2, nus2, timeout2;
		u32 time_start3, time_end3, nus3, timeout3;
		u32 time;
		u16 tag_wait_time_ms;
		u16 confl_slot_num; //冲突时隙个数
		u16 free_slot_num; //空闲时隙个数
		u16 success_frame_num;
		u16 ack_success_frame_num;
		u8 num, i, j;
		u32 delay_tx_time;
		ret_t ret = RET_SUCCESS;

		success_recv_total_cnt = 0;
		target_id = 0xffff;
		dynamic_slot = MAX_DYNAMIC_SLOT_FRAME;
		dwt_forcetrxoff();
		dwt_rxreset();

		time_start1 = get_cur_time();
	//	for(cnt=0; cnt<MAX_DYNAMIC_SLOT_FRAME_SEND_CNT; cnt++)
		while(1)
		{
				//DBG_PRINT("cnt is %d\r\n", cnt);
				time_end1 = get_cur_time();
				nus1 = get_count_time(time_start1, time_end1);
				if(nus1 > ALL_TAG_REGISTER_TIMEOUT)
				{				
					DBG_PRINT("MA:tag register all done with timeout! %d/%d(unit:1us) \r\n",nus1,ALL_TAG_REGISTER_TIMEOUT);
					if(success_recv_total_cnt > 0)
					{
						ret = RET_SUCCESS;
					}
					else
					{
						ret = RET_FAILED;
					}
				 	goto end;
				} 

		
				 //1.广播动态时隙帧	
				delay_us(50);//暂停50us，等待标签准备好接受
				buf[0] = dynamic_slot;
				ret = make_package(DYNAMIC_SLOT_ALLOCATION_CMD, Flash_Device_ID, target_id, ANCHOR, MAIN_ANCHOR, buf, 1, package_buf, &package_buf_len);
				if(ret != RET_SUCCESS)
				{
					continue;
				}
		
				ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);
			//	ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_DELAYED, 1*0x6180, (TX_NORMAL_TIMEOUT+300));	
				if(ret != RET_SUCCESS)
				{
				
					DBG_PRINT("MA:dynamic slot frame sent fail,ret= %s ,sid=%d , tid=%d \r\n", ret_type_str[ret],Flash_Device_ID,target_id);
					continue;
				}

				DBG_PRINT("MA:dynamic slot frame sent suc,ret= %s ,sid=%d , tid=%d \r\n\r\n", ret_type_str[ret],Flash_Device_ID,target_id);

				DBG_PRINT("MA:tag register process start!\r\n");
				//2.接受标签注册
				success_frame_num = 0;
				confl_slot_num = 0;
				free_slot_num = 0;

				time_start2 = get_cur_time();
				time_start = dwt_readsystimestamphi32();		
				nus = g_device_config.dyn_slot_long * dynamic_slot; 
				timeout = (nus/100) * DW1000_100US_TIMER_CNT; //dm1000每0x618000次计数大概是100us

				while(1)
				{		
					time_end = dwt_readsystimestamphi32();
					if(time_end >= time_start)
					{
						offset_time = time_end - time_start;
					}
					else
					{
						offset_time = (0xffffffff - time_start) + time_end + 1;
					}
			
					if(offset_time > timeout)
					{
						DBG_PRINT("MA:tag register single done with timeout! %d/%d(unit:4.006ns) \r\n",offset_time,timeout);
						break; //动态时隙时间完毕
					} 

					cnt++;
					
					time = (timeout - offset_time)/DW1000_100US_TIMER_CNT;
					time = time*100;

					memset(package_buf, 0, PACKAGE_BUF_SIZE);
					ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, time);
					if(ret == RECV_HIT)
					{
						confl_slot_num++;
						DBG_PRINT("MA: tag register data hit! cnt=%d, confl_slot_num=%d\r\n",cnt,confl_slot_num);
					}
					else if(ret == RET_SUCCESS)
					{			
						ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
						if(!((cmd == TAG_REGISTER_CMD) && \
							(target_id == Flash_Device_ID)&&\
							(src_device_type == TAG) && (src_device_sub_type == TAG)&& \
							(buf_len == 2)))
						{
							DBG_PRINT("MA tag register data rcv failed1: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
											ret_type_str[ret],
											dm_comm_type_str[cmd],
											target_id,src_id,
											device_type_str[src_device_type],
											anchor_type_str[src_device_sub_type],buf_len);
						}
						else
						{
							if(success_recv_total_cnt < MAX_TAG_NUM ) 
							{
								tag_wait_time_ms = (buf[0] << 8) | buf[1];
								tag_reg_info_buf[success_recv_total_cnt].tag_id = src_id;
								tag_reg_info_buf[success_recv_total_cnt].tag_wait_time_ms = tag_wait_time_ms;
								buf2[success_frame_num*2] = src_id >> 8;
								buf2[success_frame_num*2 + 1] = src_id & 0xff;
								success_frame_num++;
								success_recv_total_cnt++;
								DBG_PRINT("MA:[%2d-%3d-%3d]-tag %3d register is success,wait_time: %dms!\r\n",cnt,success_recv_total_cnt,success_frame_num,src_id,tag_wait_time_ms);
							}
						}

					}			
				}

				if(confl_slot_num == 0)
				{	
					if(success_recv_total_cnt == 0)
					{
						ret = RET_FAILED;
						DBG_PRINT("MA:tag register all done ,there is no tag!\r\n");
						goto end;
					}		
				}
				
				

				//3.回应标签注册成功信息,分多次
				DBG_PRINT("register ack!\r\n");
				id = (buf2[0] << 8 | buf2[1]);
				DBG_PRINT("id0: %d\r\n", id);
				id = (buf2[2] << 8 | buf2[3]);
				DBG_PRINT("id1: %d\r\n", id);

				target_id = 0xffff;
				data_len = 0;
				ack_success_frame_num = 0;
				num = success_frame_num/ACK_REGISTER_MAX_NUM_PER;
				if(success_frame_num%ACK_REGISTER_MAX_NUM_PER > 0)
				{
					num += 1;
				}

				
				while(num >= 1)
				{
					delay_us(50);//等待标签准备好接受
					DBG_PRINT("MA:ack num: %d\r\n", num);			
					if((success_frame_num - ack_success_frame_num) > ACK_REGISTER_MAX_NUM_PER)
					{	
						data_len = ACK_REGISTER_MAX_NUM_PER*2;
						memcpy(&buf3[1], &buf2[ack_success_frame_num*2], data_len);
						buf3[0] = num;
						data_len += 1;
						
						make_package(ACK_TAG_REGISTER_CMD, Flash_Device_ID, target_id, ANCHOR, MAIN_ANCHOR, buf3, data_len, package_buf, &package_buf_len);	
						ack_success_frame_num += ACK_REGISTER_MAX_NUM_PER;
						num--;
					}
					else
					{
						data_len = (success_frame_num - ack_success_frame_num)*2;	
						memcpy(&buf3[1], &buf2[ack_success_frame_num*2], data_len);
						buf3[0] = num;
						data_len += 1;											
						make_package(ACK_TAG_REGISTER_CMD, Flash_Device_ID, target_id, ANCHOR, MAIN_ANCHOR, buf3, data_len, package_buf, &package_buf_len);	
						ack_success_frame_num += (success_frame_num - ack_success_frame_num);
						num--;
					}
								
					ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);			
					if(ret != RET_SUCCESS)
					{
							DBG_PRINT("MA:ack data send failed-------------!\r\n");
					}	

				}

							
				DBG_PRINT("MA:ack data send suc,cnt=%d ,all_suc_cnt:%d,single_suc_cnt:%d,confl_slot_cnt:%d-------------!\r\n",cnt,success_recv_total_cnt,success_frame_num,confl_slot_num);
				
				if(success_recv_total_cnt >= MAX_TAG_NUM || confl_slot_num == 0)
				{
					ret = RET_SUCCESS;
					goto end;
				}				

				if(confl_slot_num > 0)
				{
					dynamic_slot = 2.23922*confl_slot_num; //预估剩余标签数
				}

		}
					
		
end:
		DBG_PRINT("MA:register exit,cnt:%d, dynamic_slot: %d, success_recv_total_cnt: %d\r\n",cnt, dynamic_slot,success_recv_total_cnt);

	return ret;


}




ret_t tag_register(void)
{	
		u8 package_buf[PACKAGE_BUF_SIZE];
		u8 buf[BUF_SIZE] = {0};
		u8 *pbuf;
		u16 package_buf_len;
		u16 buf_len, pbuf_len;
		u8 dynamic_slot;
		u8 slot;
		u16 target_id;
		u16 src_id;
		u16 tag_id;
		u8 src_device_type;
		u8 src_device_sub_type; 
		u8 cmd;
		u32 tag_wait_time_us;
		u16 tag_wait_time_ms;
		u32 time_start, time_end, nus, timeout;	
		u32 nus0,	nus1;
		u32 time_out;
		u32 state = 0;
		u32 cur_time, cur_dw1000_time;
		u32 delay_tx_time, delay_rx_time;
		ret_t ret = RET_SUCCESS;
		u8 i = 0;
		u8 num = 0;
		u8 cnt = 0;

		dwt_forcetrxoff();
		dwt_rxreset();
		time_start = get_cur_time();
		while(1)
		{
			time_end = get_cur_time();
			nus = get_count_time(time_start, time_end);
			if(nus >= ALL_TAG_REGISTER_TIMEOUT)
			{	
				ret = RET_FAILED;
				goto end;
			}	


			//1.接收主基站发送的动态时隙帧
			if((ALL_TAG_REGISTER_TIMEOUT - nus) > 40000)
			{
					time_out = 40000;
			}
			else
			{
					time_out = (ALL_TAG_REGISTER_TIMEOUT - nus);
			}

			memset(package_buf, 0, PACKAGE_BUF_SIZE);
			ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, time_out);
			if(ret != RET_SUCCESS)
			{
				DBG_ERR_PRINT("no recv DYNAMIC_SLOT_ALLOCATION_CMD\r\n");	
				continue;
			}
			
			ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
			if(!((ret == RET_SUCCESS) && (cmd == DYNAMIC_SLOT_ALLOCATION_CMD) && \
				(target_id == 0xffff) && (src_id == g_locate_net_info.main_anchor_id)&& \
				(src_device_type == ANCHOR) && (src_device_sub_type == MAIN_ANCHOR)&& \
				(buf_len == 1)))
			{
				DBG_ERR_PRINT("no recv DYNAMIC_SLOT_ALLOCATION_CMD,cmd:%d, src_id:%d, \r\n", cmd, src_id);	
				continue;
			}
			DBG_PRINT("RCV Dynamic Slot Frame SUC: M_ID[%3d] \r\n",g_locate_net_info.main_anchor_id);

			//2.向主基站发送注册帧
			cur_dw1000_time = dwt_readsystimestamphi32();	//获取本轮动态时隙开始时间	 	
			dynamic_slot = buf[0];
			nus0 = (dynamic_slot * g_device_config.dyn_slot_long); //本轮动态时隙结束的时间
			delay_rx_time = cur_dw1000_time + (nus0/100) * DW1000_100US_TIMER_CNT;

			//随机选择时隙，并等待时隙到达
			srand(cur_dw1000_time + Flash_Device_ID);
			slot = rand() % dynamic_slot;
			nus1 = (slot * g_device_config.dyn_slot_long + 300); //300表示等待时隙偏移300us后(让主基站先准备好接受)，标签才发送注册帧
			delay_tx_time = cur_dw1000_time + (nus1/100) * DW1000_100US_TIMER_CNT; //dm1000每0x618000次计数大概是100us。
 			
			cur_time = get_cur_time();
			tag_wait_time_us = get_count_time(g_last_tag_position_time, cur_time)  ; 
			tag_wait_time_ms = tag_wait_time_us/1000;
			buf[0] = tag_wait_time_ms >> 8;
			buf[1] = tag_wait_time_ms & 0xff;

			DBG_PRINT("dynamic_slot %d, slot %d!\r\n", dynamic_slot, slot);
			
			make_package(TAG_REGISTER_CMD, Flash_Device_ID, g_locate_net_info.main_anchor_id, TAG, TAG, buf, 2, package_buf, &package_buf_len);
		//	dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
			dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_DELAYED, delay_tx_time, (TX_NORMAL_TIMEOUT+nus1));	


			//3.接受主基站注册确认帧
			DBG_PRINT("recv register ack....\r\n");
			cnt = 0;
			num = 2; //赋值2，使得能进入while循环中
			while(num > 1) 
			{
				if(cnt == 0)
				{
					ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, delay_rx_time, (nus0+DM9000_RX_TIMEOUT));
					cnt++;
				}
				else
				{
					ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
					cnt++;
				}
				
				if(ret == RET_SUCCESS)
				{
					DBG_PRINT("parsing register ack\r\n");
					ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
					if(((ret == RET_SUCCESS) && (cmd == ACK_TAG_REGISTER_CMD) && \
						(target_id == 0xffff) && (src_id == g_locate_net_info.main_anchor_id)&& \
						(src_device_type == ANCHOR) && (src_device_sub_type == MAIN_ANCHOR)&& \
						(buf_len <= 100)))
					{
						num = buf[0];
						pbuf = &buf[1];
						pbuf_len = buf_len - 1;
						for(i=0; i<pbuf_len; i++)
						{
							tag_id = pbuf[i] << 8 | pbuf[i+1];
							if(tag_id == Flash_Device_ID)
							{
								DBG_PRINT("tag register is success!, num: %d\r\n", num);
								ret = RET_SUCCESS;
								goto end;
							}
							i++;
						}
						
						DBG_PRINT("tag register is no matching!, num: %d\r\n", num);	
					}	
				}
				else
				{
					num--;//避免死循环
					DBG_ERR_PRINT("no recv ACK_TAG_REGISTER_CMD, num: %d\r\n", num);			
				}	
			}

	 }
		
end:
	return ret;


}




//根据等待时间从小到大排序
void quick_sort(tag_reg_info_t *a,int start,int end)
{
	int i = start,j = end;
	tag_reg_info_t key = a[start];   //在起始位置挖坑，等待被填
	
	while(start < end)
	{	
		//下面这个循环从右边找小于等于基数的元素，挖出来填到左边坑
		while(start < end)
		{
			if(a[end].tag_wait_time_ms <= key.tag_wait_time_ms)
			{
				a[start].tag_wait_time_ms = a[end].tag_wait_time_ms;  //把a[end]挖出来，填到a[start]坑里
				a[start].tag_id = a[end].tag_id;
				start++;
				break;
			}
			end--;
		}
		//下面这个循环从左边找大于基数的元素，挖出来填到右边坑
		while(start < end)
		{
			if(a[start].tag_wait_time_ms > key.tag_wait_time_ms)
			{
				a[end].tag_wait_time_ms = a[start].tag_wait_time_ms; //把a[start]挖出来，填到a[end]坑里
				a[end].tag_id = a[start].tag_id;
				end--;
				break;
			}
			start++;
		}
	}
	//上面循环结束一定会有 ，start == end,这个下标位置的坑，用基数填
	int mid = start;
	a[mid] = key;
   /*到此，挖坑填数，结束，下面是分治*/
	if(i  < mid-1)
		quick_sort(a,i,mid-1);
	if(j  > mid+1)
     	quick_sort(a,mid+1,j);	
}





ret_t m_slot_allocation(void)
{	
		u8 package_buf[PACKAGE_BUF_SIZE];
		u8 buf[BUF_SIZE] = {0};
		u8 buf2[BUF_SIZE] = {0};
		u16 package_buf_len;
		u16 buf_len, data_len;
		u16 target_id;
		u16 src_id;
		u32 time_start, time_end, nus, timeout;			
		u8 i, cnt;
		u8 index = 0;
		u16 success_frame_num;
		u16 send_ranging_slot_num;
		u8 num;		
		
		ret_t ret = RET_SUCCESS;
		
		time_start = get_cur_time();

		quick_sort(tag_reg_info_buf,0,(success_recv_total_cnt-1)); //根据等待时间从小到大排序
		
		//发送测距时隙表给标签，等待时间越久，时隙越靠前
		index = 0;
		//buf[index] = success_recv_total_cnt;	
		//index++;
		for(i=success_recv_total_cnt; i>0; i--)
		{		
			buf[index] = tag_reg_info_buf[i-1].tag_id >> 8;	
			buf[index+1] = tag_reg_info_buf[i-1].tag_id & 0xff;
			index += 2;
		}

		target_id = 0xffff;
		data_len = 0;
		send_ranging_slot_num = 0;
		num = success_recv_total_cnt/SEND_RANGING_SLOT_MAX_NUM_PER;
		if(success_recv_total_cnt%SEND_RANGING_SLOT_MAX_NUM_PER > 0)
		{
			num += 1;
		}
		
		while(num >= 1)
		{	
			osDelay(2); //等待标签准备好接受
			DBG_PRINT("MA:send num: %d\r\n", num); 
			if((success_recv_total_cnt - send_ranging_slot_num) > SEND_RANGING_SLOT_MAX_NUM_PER)
			{ 
				data_len = SEND_RANGING_SLOT_MAX_NUM_PER*2;
				memcpy(&buf2[2], &buf[send_ranging_slot_num*2], data_len);
				buf2[0] = num;
				buf2[1] = SEND_RANGING_SLOT_MAX_NUM_PER;
				data_len += 2;
				
				make_package(SLOT_ALLOCATION_CMD, Flash_Device_ID, target_id, ANCHOR, MAIN_ANCHOR, buf2, data_len, package_buf, &package_buf_len); 
				send_ranging_slot_num += SEND_RANGING_SLOT_MAX_NUM_PER;
			}
			else
			{
				data_len = (success_recv_total_cnt - send_ranging_slot_num)*2; 
				memcpy(&buf2[2], &buf[send_ranging_slot_num*2], data_len);
				buf2[0] = num;
				buf2[1] = success_recv_total_cnt - send_ranging_slot_num;
				data_len += 2;											
				make_package(SLOT_ALLOCATION_CMD, Flash_Device_ID, target_id, ANCHOR, MAIN_ANCHOR, buf2, data_len, package_buf, &package_buf_len); 
				send_ranging_slot_num += (success_recv_total_cnt - send_ranging_slot_num);
			}
						
			ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT); 		
			if(ret != RET_SUCCESS)
			{
				
				DBG_PRINT("MA:slot allocation data send failed-------------!\r\n");
			} 
		
			num--;
		}


		while(1)
		{
			time_end = get_cur_time();
			nus = get_count_time(time_start, time_end);	
			if(nus > TAG_WAIT_SLOT_TIMEOUT) 
			{
				break;
			}
		}
							
end:
	return ret;


}



ret_t tag_recv_slot_allocation(u8 *slot)
{	
	u8 package_buf[PACKAGE_BUF_SIZE];
	u8 buf[BUF_SIZE] = {0};
	u8 *pbuf;
	u16 package_buf_len;
	u16 buf_len;
	u16 target_id;
	u16 src_id;
	u16 tag_id;	
	u8 src_device_type;
	u8 src_device_sub_type; 
	u8 cmd;
	u8 i;
	u32 time_start, time_end, nus, timeout, time;		
	u8 slot_num, slot_frame_num,slot_num_cnt;
	u8 success_flag = 0;
	ret_t ret = RET_FAILED;

	*slot = 0;
	//接受主基站的测距时隙表
	time = MAX_TAG_NUM*g_device_config.dyn_slot_long + 20000;
	time_start = get_cur_time();
	while(1)
	{
		time_end = get_cur_time();
		nus = get_count_time(time_start, time_end);
		if(nus > time) 
		{
			if(success_flag != 1)
			{
				ret = RET_FAILED;
			}
			osDelay(100);
			DBG_PRINT("tag slot timeout ,%d/%d ,ret=%s \r\n", nus,time,ret_type_str[ret]);
			
			goto end;
		}

		if(success_flag == 1)
		{
			ret = RET_SUCCESS;
			continue;
		}

		timeout = time - nus;
		memset(package_buf, 0, PACKAGE_BUF_SIZE);
		ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, timeout);
		if(ret != RET_SUCCESS)
		{
			DBG_PRINT("tag slot single failed(data rcv failed): %d\r\n", ret);
			continue;
		}
	
		ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
		
		//检测主基站是否还处于等待标签注册
		if(((ret == RET_SUCCESS) && (cmd == DYNAMIC_SLOT_ALLOCATION_CMD) && \
			(target_id == 0xffff) && (src_id == g_locate_net_info.main_anchor_id)&& \
			(src_device_type == ANCHOR) && (src_device_sub_type == MAIN_ANCHOR)&& \
			(buf_len == 1)))
		{
				slot_num = buf[0]; 
				time = slot_num* g_device_config.dyn_slot_long + TAG_WAIT_SLOT_TIMEOUT;
				time_start = get_cur_time(); //最后一次收到主基站动态时隙帧的时间
				DBG_PRINT("tag slot single failed(data parse failed0): %d,%d,%d,%d,%d,%d,%d\r\n", ret,cmd,target_id,src_id,src_device_type,src_device_sub_type,buf_len);
				continue;
		}
	
		//检测主基站发送的时隙表
		if(!((ret == RET_SUCCESS) && (cmd == SLOT_ALLOCATION_CMD) && \
			(target_id == 0xffff) && (src_id == g_locate_net_info.main_anchor_id)&& \
			(src_device_type == ANCHOR) && (src_device_sub_type == MAIN_ANCHOR)&& \
			(buf_len <= 100)))
		{
			continue;
		} 

		slot_num_cnt = 0;
		slot_frame_num = buf[0]; 
		slot_num = buf[1]; //时隙表中的时隙个数
		pbuf = (u8*)&buf[2];
		buf_len -= 2;
		for(i=0; i<buf_len; i++)
		{		
			tag_id = (pbuf[i]<<8) | pbuf[i+1];
			DBG_PRINT("tag_id: %d/%d\r\n", tag_id,buf_len);
			if(tag_id == Flash_Device_ID)
			{
				*slot += slot_num_cnt; //时隙
				success_flag = 1;
				DBG_PRINT("tag slot allocation suc, slot is %d/%d\r\n", *slot,slot_num);
			}
			i++;
			slot_num_cnt++;
		}

		if(success_flag  != 1)
		{
			*slot += slot_num;
		}		

		DBG_PRINT("slot_frame_num: %d\r\n", slot_frame_num);
		
	}
	
end:
		return ret;


}


#if 0
//0
void test_position(void)
{

	PECEF pcc;
	PWGS center, pcg;
	PENU pct;
	int rett;
	double x,y,z;
	double tag_h;
	double d1,d2,d3;
	ECEF a, b, c;
	ENU temp;
	double clua_x_y[4];
	
	
	pcc = (PECEF)GLOBAL_MALLOC(sizeof(ECEF));
	center = (PWGS)GLOBAL_MALLOC(sizeof(WGS));
	pcg = (PWGS)GLOBAL_MALLOC(sizeof(WGS));
	pct = (PENU)GLOBAL_MALLOC(sizeof(ENU));


	//定位解算
	pcg->latitude = 30.57477812;
	pcg->longitude = 104.05835120;
	pcg->height = 512.99;
	WGSToECEF(pcg, pcc);
	a.x = pcc->x;
	a.y = pcc->y;
	a.z = pcc->z;	
	
						
	pcg->latitude = 30.57477677;
	pcg->longitude = 104.05844831;
	pcg->height = 513.019; 				
	WGSToECEF(pcg, pcc);
	b.x = pcc->x;
	b.y = pcc->y;
	b.z = pcc->z;
	
	pcg->latitude = 30.57473120;
	pcg->longitude = 104.05840174;
	pcg->height = 511.495; 								
	WGSToECEF(pcg, pcc);
	c.x = pcc->x;
	c.y = pcc->y;
	c.z = pcc->z;
	
	
	//a <-> c
	center->latitude = 30.57477812;
	center->longitude = 104.05835120;
	center->height = 512.99;
	ECEFToENU((PECEF)&c, center, pct);							
	d1 = sqrt(pow(pct->easting,2) + pow(pct->northing,2) + pow(pct->upping,2)); 
	temp.easting = pct->easting;
	temp.northing = pct->northing;
	temp.upping = pct->upping;
     	
	
	//c <-> b
	center->latitude =30.57473120;
	center->longitude = 104.05840174;
	center->height = 511.495;	
	ECEFToENU((PECEF)&b, center, pct);
	d2 = sqrt(pow(pct->easting,2) + pow(pct->northing,2) + pow(pct->upping,2)); 

	//a <-> b
	center->latitude = 30.57477812;
	center->longitude = 104.05835120;
	center->height = 512.99;
	ECEFToENU((PECEF)&b, center, pct);
	d3 = sqrt(pow(pct->easting,2) + pow(pct->northing,2) + pow(pct->upping,2)); 
	DBG_PRINT("----------d1: %lf, d2: %lf, d3: %lf m\r\n", d1,d2,d3);
	
	negative_rotate3(pct->easting, pct->northing, pct->upping, temp.easting, temp.northing, temp.upping, &x, &y, &z);
	tag_h = z;


//	d1= 7.10;
//	d2= 6.78;
	//tag_h = -1.7;

	
	rett = CalcPosition_enu(d1,pct->easting,pct->northing,pct->upping,d2,tag_h,0,clua_x_y);

	if(rett < 0)
	{
		DBG_PRINT("----------pos invalid value\r\n");
	}
	else
	{ 							
		//坐标滤波
		pct->easting = clua_x_y[0];
		pct->northing = clua_x_y[1];
		pct->upping = clua_x_y[2];
		ENUToECEF(pcc, center, pct);
		ECEFToWGS(pcg, pcc);
		
	//	pcg->latitude = KalmanFilter(pcg->latitude,g_device_config.kalman_q,g_device_config.kalman_r,0); //卡尔曼滤波
	//	pcg->longitude = KalmanFilter(pcg->longitude,g_device_config.kalman_q,g_device_config.kalman_r,1); 
	//	pcg->height = KalmanFilter(pcg->height,g_device_config.kalman_q,g_device_config.kalman_r,2); 
		
		g_pos_info.tag_position[0] = pcg->latitude;
		g_pos_info.tag_position[1] = pcg->longitude;
		g_pos_info.tag_position[2] = pcg->height;
	

		DBG_PRINT("----------latitude: %lf, longitude: %lf, height: %lf m\r\n", g_pos_info.tag_position[0], g_pos_info.tag_position[1], g_pos_info.tag_position[2]);								
	}


	GLOBAL_FREE(pcc);
	GLOBAL_FREE(center);
	GLOBAL_FREE(pcg);
	GLOBAL_FREE(pct);

	delay_ms(2000);

				

}

#endif



#if 0
//1
void test_position(void)
{

	PECEF pcc;
	PWGS center, pcg;
	PENU pct;
	int rett;
	double x,y,z;
	double tag_h;
	double d1,d2,d3;
	ECEF a, b, c;
	ENU temp;
	double clua_x_y[4];
	
	
	pcc = (PECEF)GLOBAL_MALLOC(sizeof(ECEF));
	center = (PWGS)GLOBAL_MALLOC(sizeof(WGS));
	pcg = (PWGS)GLOBAL_MALLOC(sizeof(WGS));
	pct = (PENU)GLOBAL_MALLOC(sizeof(ENU));


	//定位解算
	pcg->latitude = 30.57493110;
	pcg->longitude = 104.05791284;
	pcg->height = 512.863;
	WGSToECEF(pcg, pcc);
	a.x = pcc->x;
	a.y = pcc->y;
	a.z = pcc->z;	
	
						
	pcg->latitude = 30.57489152;
	pcg->longitude = 104.05788480;
	pcg->height = 512.966; 				
	WGSToECEF(pcg, pcc);
	b.x = pcc->x;
	b.y = pcc->y;
	b.z = pcc->z;
	
	pcg->latitude = 30.57492398;
	pcg->longitude = 104.05786886;
	pcg->height = 511.703; 								
	WGSToECEF(pcg, pcc);
	c.x = pcc->x;
	c.y = pcc->y;
	c.z = pcc->z;
	
	
	//a <-> c
	center->latitude = 30.57493110;
	center->longitude = 104.05791284;
	center->height = 512.863;
	ECEFToENU((PECEF)&c, center, pct);							
	d1 = sqrt(pow(pct->easting,2) + pow(pct->northing,2) + pow(pct->upping,2)); 
	temp.easting = pct->easting;
	temp.northing = pct->northing;
	temp.upping = pct->upping;
     	
	
	//c <-> b
	center->latitude =30.57492398;
	center->longitude = 104.05786886;
	center->height = 511.703;	
	ECEFToENU((PECEF)&b, center, pct);
	d2 = sqrt(pow(pct->easting,2) + pow(pct->northing,2) + pow(pct->upping,2)); 

	//a <-> b
	center->latitude = 30.57493110;
	center->longitude = 104.05791284;
	center->height = 512.863;
	ECEFToENU((PECEF)&b, center, pct);
	d3 = sqrt(pow(pct->easting,2) + pow(pct->northing,2) + pow(pct->upping,2)); 
	DBG_PRINT("----------d1: %lf, d2: %lf, d3: %lf m\r\n", d1,d2,d3);
	
	negative_rotate3(pct->easting, pct->northing, pct->upping, temp.easting, temp.northing, temp.upping, &x, &y, &z);
	tag_h = z;


	d1= 4.42;
	d2= 3.77;
	tag_h = -0.86;


	
	rett = CalcPosition_enu(d1,pct->easting,pct->northing,pct->upping,d2,tag_h,0,clua_x_y);

	if(rett < 0)
	{
		DBG_PRINT("----------pos invalid value\r\n");
	}
	else
	{ 							
		//坐标滤波
		pct->easting = clua_x_y[0];
		pct->northing = clua_x_y[1];
		pct->upping = clua_x_y[2];
		ENUToECEF(pcc, center, pct);
		ECEFToWGS(pcg, pcc);
		
	//	pcg->latitude = KalmanFilter(pcg->latitude,g_device_config.kalman_q,g_device_config.kalman_r,0); //卡尔曼滤波
	//	pcg->longitude = KalmanFilter(pcg->longitude,g_device_config.kalman_q,g_device_config.kalman_r,1); 
	//	pcg->height = KalmanFilter(pcg->height,g_device_config.kalman_q,g_device_config.kalman_r,2); 
		
		g_pos_info.tag_position[0] = pcg->latitude;
		g_pos_info.tag_position[1] = pcg->longitude;
		g_pos_info.tag_position[2] = pcg->height;
	

		DBG_PRINT("----------latitude: %lf, longitude: %lf, height: %lf m\r\n", g_pos_info.tag_position[0], g_pos_info.tag_position[1], g_pos_info.tag_position[2]);								
	}


	GLOBAL_FREE(pcc);
	GLOBAL_FREE(center);
	GLOBAL_FREE(pcg);
	GLOBAL_FREE(pct);

	delay_ms(2000);

				

}

#endif


#if 0
//2
void test_position(void)
{

	PECEF pcc;
	PWGS center, pcg;
	PENU pct;
	int rett;
	double x,y,z;
	double tag_h;
	double d1,d2,d3;
	ECEF a, b, c;
	ENU temp;
	double clua_x_y[4];
	
	
	pcc = (PECEF)GLOBAL_MALLOC(sizeof(ECEF));
	center = (PWGS)GLOBAL_MALLOC(sizeof(WGS));
	pcg = (PWGS)GLOBAL_MALLOC(sizeof(WGS));
	pct = (PENU)GLOBAL_MALLOC(sizeof(ENU));


	//定位解算
	pcg->latitude = 30.57493125;
	pcg->longitude = 104.05791217;
	pcg->height = 512.903;
	WGSToECEF(pcg, pcc);
	a.x = pcc->x;
	a.y = pcc->y;
	a.z = pcc->z;	
	
						
	pcg->latitude = 30.57485180;
	pcg->longitude = 104.05785392;
	pcg->height = 512.955; 				
	WGSToECEF(pcg, pcc);
	b.x = pcc->x;
	b.y = pcc->y;
	b.z = pcc->z;
	
	pcg->latitude = 30.57492565;
	pcg->longitude = 104.05785617;
	pcg->height = 512.055; 								
	WGSToECEF(pcg, pcc);
	c.x = pcc->x;
	c.y = pcc->y;
	c.z = pcc->z;
	
	
	//a <-> c
	center->latitude = 30.57493125;
	center->longitude = 104.05791217;
	center->height = 512.903;
	ECEFToENU((PECEF)&c, center, pct);							
	d1 = sqrt(pow(pct->easting,2) + pow(pct->northing,2) + pow(pct->upping,2)); 
	temp.easting = pct->easting;
	temp.northing = pct->northing;
	temp.upping = pct->upping;
     	
	
	//c <-> b
	center->latitude =30.57492565;
	center->longitude = 104.05785617;
	center->height = 512.055;
	ECEFToENU((PECEF)&b, center, pct);
	d2 = sqrt(pow(pct->easting,2) + pow(pct->northing,2) + pow(pct->upping,2)); 

	//a <-> b
	center->latitude = 30.57493125;
	center->longitude = 104.05791217;
	center->height = 512.903;

	ECEFToENU((PECEF)&b, center, pct);
	d3 = sqrt(pow(pct->easting,2) + pow(pct->northing,2) + pow(pct->upping,2)); 
	DBG_PRINT("----------d1: %lf, d2: %lf, d3: %lf m\r\n", d1,d2,d3);

	positive_rotate3(pct->easting, pct->northing, pct->upping, temp.easting, temp.northing, temp.upping, &x, &y, &z);
	tag_h = z;


	//d1= 4.42;
	//d2= 3.77;
	//tag_h = -0.86;

	
	rett = CalcPosition_enu(d1,pct->easting,pct->northing,pct->upping,d2,tag_h,0,clua_x_y);

	if(rett < 0)
	{
		DBG_PRINT("----------pos invalid value\r\n");
	}
	else
	{ 							
		//坐标滤波
		pct->easting = clua_x_y[0];
		pct->northing = clua_x_y[1];
		pct->upping = clua_x_y[2];
		ENUToECEF(pcc, center, pct);
		ECEFToWGS(pcg, pcc);
		
	//	pcg->latitude = KalmanFilter(pcg->latitude,g_device_config.kalman_q,g_device_config.kalman_r,0); //卡尔曼滤波
	//	pcg->longitude = KalmanFilter(pcg->longitude,g_device_config.kalman_q,g_device_config.kalman_r,1); 
	//	pcg->height = KalmanFilter(pcg->height,g_device_config.kalman_q,g_device_config.kalman_r,2); 
		
		g_pos_info.tag_position[0] = pcg->latitude;
		g_pos_info.tag_position[1] = pcg->longitude;
		g_pos_info.tag_position[2] = pcg->height;
	

		DBG_PRINT("----------latitude: %lf, longitude: %lf, height: %lf m\r\n", g_pos_info.tag_position[0], g_pos_info.tag_position[1], g_pos_info.tag_position[2]);								
	}


	GLOBAL_FREE(pcc);
	GLOBAL_FREE(center);
	GLOBAL_FREE(pcg);
	GLOBAL_FREE(pct);

	delay_ms(2000);

				

}

#endif

#if 1
//3
void test_position(void)
{

	PECEF pcc;
	PWGS center, pcg;
	PENU pct;
	int rett;
	double x,y,z;
	double tag_h;
	double d1,d2,d3;
	ECEF a, b, c;
	ENU temp;
	double clua_x_y[4];
	
	
	pcc = (PECEF)GLOBAL_MALLOC(sizeof(ECEF));
	center = (PWGS)GLOBAL_MALLOC(sizeof(WGS));
	pcg = (PWGS)GLOBAL_MALLOC(sizeof(WGS));
	pct = (PENU)GLOBAL_MALLOC(sizeof(ENU));


	//定位解算
	pcg->latitude = 30.572123;
	pcg->longitude = 104.056049;
	pcg->height = 474.364;
	WGSToECEF(pcg, pcc);
	a.x = pcc->x;
	a.y = pcc->y;
	a.z = pcc->z;	
	
						
	pcg->latitude = 30.570365;
	pcg->longitude = 104.055617;
	pcg->height = 473.988; 				
	WGSToECEF(pcg, pcc);
	b.x = pcc->x;
	b.y = pcc->y;
	b.z = pcc->z;
	
	pcg->latitude = 30.57492565;
	pcg->longitude = 104.05785617;
	pcg->height = 512.055; 								
	WGSToECEF(pcg, pcc);
	c.x = pcc->x;
	c.y = pcc->y;
	c.z = pcc->z;
	
	
	//a <-> c
	center->latitude = 30.57493125;
	center->longitude = 104.05791217;
	center->height = 512.903;
	ECEFToENU((PECEF)&c, center, pct);							
	d1 = sqrt(pow(pct->easting,2) + pow(pct->northing,2) + pow(pct->upping,2)); 
	temp.easting = pct->easting;
	temp.northing = pct->northing;
	temp.upping = pct->upping;
     	
	
	//c <-> b
	center->latitude =30.57492565;
	center->longitude = 104.05785617;
	center->height = 512.055;
	ECEFToENU((PECEF)&b, center, pct);
	d2 = sqrt(pow(pct->easting,2) + pow(pct->northing,2) + pow(pct->upping,2)); 

	//a <-> b
	center->latitude = 30.572123;
	center->longitude = 104.056049;
	center->height = 474.364;
	

	ECEFToENU((PECEF)&b, center, pct);
	d3 = sqrt(pow(pct->easting,2) + pow(pct->northing,2) + pow(pct->upping,2)); 
	DBG_PRINT("----------d1: %lf, d2: %lf, d3: %lf m\r\n", d1,d2,d3);

	positive_rotate3(pct->easting, pct->northing, pct->upping, temp.easting, temp.northing, temp.upping, &x, &y, &z);
	tag_h = z;


	//d1= 4.42;
	//d2= 3.77;
	//tag_h = -0.86;

	
	rett = CalcPosition_enu(d1,pct->easting,pct->northing,pct->upping,d2,tag_h,0,clua_x_y);

	if(rett < 0)
	{
		DBG_PRINT("----------pos invalid value\r\n");
	}
	else
	{ 							
		//坐标滤波
		pct->easting = clua_x_y[0];
		pct->northing = clua_x_y[1];
		pct->upping = clua_x_y[2];
		ENUToECEF(pcc, center, pct);
		ECEFToWGS(pcg, pcc);
		
	//	pcg->latitude = KalmanFilter(pcg->latitude,g_device_config.kalman_q,g_device_config.kalman_r,0); //卡尔曼滤波
	//	pcg->longitude = KalmanFilter(pcg->longitude,g_device_config.kalman_q,g_device_config.kalman_r,1); 
	//	pcg->height = KalmanFilter(pcg->height,g_device_config.kalman_q,g_device_config.kalman_r,2); 
		
		g_pos_info.tag_position[0] = pcg->latitude;
		g_pos_info.tag_position[1] = pcg->longitude;
		g_pos_info.tag_position[2] = pcg->height;
	

		DBG_PRINT("----------latitude: %lf, longitude: %lf, height: %lf m\r\n", g_pos_info.tag_position[0], g_pos_info.tag_position[1], g_pos_info.tag_position[2]);								
	}


	GLOBAL_FREE(pcc);
	GLOBAL_FREE(center);
	GLOBAL_FREE(pcg);
	GLOBAL_FREE(pct);

	delay_ms(2000);

				

}

#endif





ret_t tag_ranging(u8 tag_slot) 
{	   			
		u8 package_buf[PACKAGE_BUF_SIZE];
		u8 buf[BUF_SIZE] = {0};
		u16 package_buf_len;
		u16 buf_len;
		u8 dynamic_slot;
		u16 target_id;
		u16 src_id;
		u16 tag_id;
		u8 src_device_type;
		u8 src_device_sub_type; 
		u8 cmd;
		u16 tag_wait_time_us;
		u32 time_start, time_end, nus, timeout; 
		u32 time_start1, time_end1, nus1, timeout1;
		u32 time_out;
		u32 state = 0;
		u32 Time_ts[6];  					//飞行时间缓存记录
		u32 Time_ts2[6];  					//飞行时间缓存记录	
		u32 position_interval; //定位间隔
		u32 time;
		u32 delay_tx_time;
		u32 cur_dw1000_time;
		ret_t ret = RET_FAILED;

		//tag_slot = 3;
		time_start = get_cur_time(); 
		
		//1.等待测距时隙到达
		time = (tag_slot * g_device_config.ranging_slot_long+300);
		cur_dw1000_time = dwt_readsystimestamphi32();
		delay_tx_time = cur_dw1000_time + (time/100) * DW1000_100US_TIMER_CNT; //dm1000每0x618000次计数大概是100us。
		
		//2.测距
		while(1)
		{	
				switch(state)
				{
						//0.发送POLL帧
						case 0: 
						{	
								ret = make_package(POLL, Flash_Device_ID, g_locate_net_info.main_anchor_id, TAG, TAG, NULL, 0, package_buf, &package_buf_len);
								if(ret != RET_SUCCESS)
								{
									goto end;
								}
								g_get_tx_timestamp_flag = 1;
								//ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
								ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_DELAYED, delay_tx_time, (time + TX_NORMAL_TIMEOUT));	
								if(ret != RET_SUCCESS)
								{
									DBG_ERR_PRINT("0. poll sent fail,ret= %s\r\n", ret_type_str[ret]);
									goto end;
								}

							//	time_start = get_cur_time(); 

								Time_ts[0] = g_tx_timestamp;										//获得POLL发送时间T1
								Time_ts2[0] = Time_ts[0];				
								state=1;
								DBG_PRINT("0. poll sent suc,stamp0:%u\r\n",Time_ts[0]);
								
								break;
						}			

						//1.接受主基站RESP帧		
						case 1:
						{	
								g_get_rx_timestamp_flag = 1;
								
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
								if(ret != RET_SUCCESS)
								{
									DBG_ERR_PRINT("1. tag range m_resp rcv failed0: %s\r\n", ret_type_str[ret]);
									goto end;
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == RESP) && \
									(target_id == Flash_Device_ID) && (src_id == g_locate_net_info.main_anchor_id)&& \
									(src_device_type == ANCHOR) && (src_device_sub_type == MAIN_ANCHOR)&& \
									(buf_len == 4)))
								{
									DBG_ERR_PRINT("1. tag range m_resp rcv failed1: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
																ret_type_str[ret],
																dm_comm_type_str[cmd],
																target_id,src_id,
																device_type_str[src_device_type],
																anchor_type_str[src_device_sub_type],buf_len);
									goto end;
								}

								Time_ts[1] = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];//获得主基站POLL接收时间T2	
								Time_ts[3] = g_rx_timestamp;	//获得接受到主基站RESP时间T4												
								state=2;
								DBG_PRINT("1. m[%d] resp rcv suc,stamp1:%u stamp3:%u\r\n",g_locate_net_info.main_anchor_id,Time_ts[1],Time_ts[3]);
					
								break;

						}

						//2.接受从基站RESP帧
						case 2:
						{	
								g_get_rx_timestamp_flag = 1;
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
								if(ret != RET_SUCCESS)
								{
									DBG_ERR_PRINT("2. tag range s_resp rcv failed0: %s\r\n", ret_type_str[ret]);
									goto end;
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == RESP) && \
									(target_id == Flash_Device_ID) && (src_id == g_locate_net_info.sub_anchor_id)&& \
									(src_device_type == ANCHOR) && (src_device_sub_type == SUB_ANCHOR)&& \
									(buf_len == 4)))
								{
									DBG_ERR_PRINT("2. tag range s_resp rcv failed1: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
																ret_type_str[ret],
																dm_comm_type_str[cmd],
																target_id,src_id,
																device_type_str[src_device_type],
																anchor_type_str[src_device_sub_type],buf_len);
									goto end;
								}
							
								Time_ts2[1] = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];//获得从基站POLL接收时间T2	
								Time_ts2[3] = g_rx_timestamp;//获得接受到从基站RESP时间T4														
								state=3;
								DBG_PRINT("2. s[%d] resp rcv suc,stamp1:%u stamp3:%u\r\n",g_locate_net_info.sub_anchor_id,Time_ts2[1],Time_ts2[3]);
					
								break;

						}

						//3.发送FINAL帧
		        		case 3:
						{
								ret = make_package(FINAL, Flash_Device_ID, g_locate_net_info.main_anchor_id, TAG, TAG, NULL, 0, package_buf, &package_buf_len);
								if(ret != RET_SUCCESS)
								{
									goto end;
								}
								delay_us(50);
								g_get_tx_timestamp_flag = 1;
								ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
								if(ret != RET_SUCCESS)
								{
									DBG_ERR_PRINT("3.final sent fail,ret= %s\r\n", ret_type_str[ret]);
									goto end;
								}

								Time_ts[4] = g_tx_timestamp;										//获得FINAL发送时间T5
								Time_ts2[4] = Time_ts[4];
								state=4;
								DBG_PRINT("3. final sent suc,stamp4:%u\r\n",Time_ts[4]);
					
								break;

						}

						//4.接受主基站ACK帧
						case 4:
						{
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
								if(ret != RET_SUCCESS)
								{
									DBG_ERR_PRINT("4. tag range m_ack rcv failed0: %s\r\n", ret_type_str[ret]);
									goto end;
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == ACK) && \
									(target_id == Flash_Device_ID) && (src_id == g_locate_net_info.main_anchor_id)&& \
									(src_device_type == ANCHOR) && (src_device_sub_type == MAIN_ANCHOR)&& \
									(buf_len == 8)))
								{
									DBG_ERR_PRINT("4. tag range m_ack rcv failed1: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
																ret_type_str[ret],
																dm_comm_type_str[cmd],
																target_id,src_id,
																device_type_str[src_device_type],
																anchor_type_str[src_device_sub_type],buf_len);
									goto end;
								}
								
								Time_ts[2] = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];//获得主基站RESP帧发送时间T3
								Time_ts[5] = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];//获得主基站RESP帧发送时间T6	
								state=5;
								DBG_PRINT("4. m[%d] ack rcv suc,stamp2:%u stamp5:%u\r\n",g_locate_net_info.main_anchor_id,Time_ts[2],Time_ts[5]);
					
								break;					
						}

						//5.接受从基站ACK帧
						case 5:						
						{
								//printf("%d\n",SYS_Calculate_ACTIVE_FLAG);
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
								if(ret != RET_SUCCESS)
								{
									DBG_ERR_PRINT("5. tag range s_ack rcv failed0: %s\r\n", ret_type_str[ret]);
									goto end;
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == ACK) && \
									(target_id == Flash_Device_ID) && (src_id == g_locate_net_info.sub_anchor_id)&& \
									(src_device_type == ANCHOR) && (src_device_sub_type == SUB_ANCHOR)&& \
									(buf_len == 8)))
								{
									DBG_ERR_PRINT("4. tag range s_ack rcv failed1: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
																ret_type_str[ret],
																dm_comm_type_str[cmd],
																target_id,src_id,
																device_type_str[src_device_type],
																anchor_type_str[src_device_sub_type],buf_len);
									goto end;
								}
								
								Time_ts2[2] = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];//获得从基站RESP帧发送时间T3
								Time_ts2[5] = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];//获得从基站RESP帧发送时间T6	
								state=6;					

								time_end = get_cur_time();
								nus = get_count_time(time_start, time_end);
								DBG_PRINT("5. s[%d] ack rcv suc,stamp2:%u stamp5:%u\r\n\r\n",g_locate_net_info.sub_anchor_id,Time_ts2[2],Time_ts2[5]);
								DBG_PRINT("tag range elapse_time:%d us\r\n",nus);
					
								break;


						}

						//6.计算标签与主基站、从基站的距离、计算
						case 6:
						{
								//uint32 Time_ts_F[6];
								double Ra, Rb, Da, Db;
		            uint64 tof_dtu;
								double tof;             //飞行时间
								double distance,dist;  //理论距离 ，估算距离
								double d0,d1;
								double clua_x_y_z[4];
								s32 dis0, dis1;
								u8 cla_flag=0;
								PECEF pcc;
								PWGS center, pcg;
								PENU pct;
								int rett;
								double tag_h;
								float hight; //标签相对基站的高度
								
				

								pcc = (PECEF)GLOBAL_MALLOC(sizeof(ECEF));
								center = (PWGS)GLOBAL_MALLOC(sizeof(WGS));
								pcg = (PWGS)GLOBAL_MALLOC(sizeof(WGS));
								pct = (PENU)GLOBAL_MALLOC(sizeof(ENU));

								time_start1 = get_cur_time();

								if(Time_ts[3] >= Time_ts[0])
								{
									Ra = (double)(Time_ts[3] - Time_ts[0]);//Tround1 = T4 - T1
								}
								else
								{
									Ra = (double)((0xffffffff - Time_ts[0]) + Time_ts[3] + 1);
								}	

								if(Time_ts[5] >= Time_ts[2])
								{
									Rb = (double)(Time_ts[5] - Time_ts[2]);//Tround2 = T6 - T3
								}
								else
								{
									Rb = (double)((0xffffffff - Time_ts[2]) + Time_ts[5] + 1);
								}	

								if(Time_ts[4] >= Time_ts[3])
								{
									Da = (double)(Time_ts[4] - Time_ts[3]);//Treply2 = T5 - T4
								}
								else
								{
									Da = (double)((0xffffffff - Time_ts[3]) + Time_ts[4] + 1);
								}	

								if(Time_ts[2] >= Time_ts[1])
								{
									Db = (double)(Time_ts[2] - Time_ts[1]);//Treply1 = T3 - T2
								}
								else
								{
									Db = (double)((0xffffffff - Time_ts[1]) + Time_ts[2] + 1);
								}	

#if 0
								Ra = (double)(Time_ts[3] - Time_ts[0]);//Tround1 = T4 - T1  
		            Rb = (double)(Time_ts[5] - Time_ts[2]);//Tround2 = T6 - T3 
								Da = (double)(Time_ts[4] - Time_ts[3]);//Treply2 = T5 - T4  
								Db = (double)(Time_ts[2] - Time_ts[1]);//Treply1 = T3 - T2  
#endif
								tof_dtu = (uint64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));//计算公式
								tof = tof_dtu * DWT_TIME_UNITS;
								distance = tof * SPEED_OF_LIGHT;//距离=光速*飞行时间
								dist = distance - dwt_getrangebias(config.chan,(float)distance, config.prf);//距离减去矫正系数	
								dis0 = dist*100;//dis 为单位为cm的距离		

								if(Time_ts2[3] >= Time_ts2[0])
								{
									Ra = (double)(Time_ts2[3] - Time_ts2[0]);//Tround1 = T4 - T1
								}
								else
								{
									Ra = (double)((0xffffffff - Time_ts2[0]) + Time_ts2[3] + 1);
								}	

								if(Time_ts2[5] >= Time_ts2[2])
								{
									Rb = (double)(Time_ts2[5] - Time_ts2[2]);//Tround2 = T6 - T3
								}
								else
								{
									Rb = (double)((0xffffffff - Time_ts2[2]) + Time_ts2[5] + 1);
								}	

								if(Time_ts2[4] >= Time_ts2[3])
								{
									Da = (double)(Time_ts2[4] - Time_ts2[3]);//Treply2 = T5 - T4
								}
								else
								{
									Da = (double)((0xffffffff - Time_ts2[3]) + Time_ts2[4] + 1);
								}	

								if(Time_ts2[2] >= Time_ts2[1])
								{
									Db = (double)(Time_ts2[2] - Time_ts2[1]);//Treply1 = T3 - T2
								}
								else
								{
									Db = (double)((0xffffffff - Time_ts2[1]) + Time_ts2[2] + 1);
								}	


#if 0
								Ra = (double)(Time_ts2[3] - Time_ts2[0]);//Tround1 = T4 - T1  
								Rb = (double)(Time_ts2[5] - Time_ts2[2]);//Tround2 = T6 - T3 
								Da = (double)(Time_ts2[4] - Time_ts2[3]);//Treply2 = T5 - T4  
								Db = (double)(Time_ts2[2] - Time_ts2[1]);//Treply1 = T3 - T2 
#endif

								tof_dtu = (uint64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));//计算公式
								tof = tof_dtu * DWT_TIME_UNITS;
								distance = tof * SPEED_OF_LIGHT;//距离=光速*飞行时间
								dist = distance - dwt_getrangebias(config.chan,(float)distance, config.prf);//距离减去矫正系数	
								dis1 = dist*100;//dis 为单位为cm的距离

								//距离低通滤波
								//dis0 = LP(dis0,0);
								//dis1 = LP(dis1,1);	
								d0 = dis0 / 100.0;
								d1 = dis1 / 100.0;

								g_pos_info.D0 = d0;
								g_pos_info.D1 = d1;

								g_pos_info.main_anchor_id = g_locate_net_info.main_anchor_id;
								g_pos_info.sub_anchor_id = g_locate_net_info.sub_anchor_id;


								//定位解算					
								center->latitude = g_locate_net_info.anchor_position[0];
								center->longitude = g_locate_net_info.anchor_position[1];
								center->height = g_locate_net_info.anchor_position[2];								
								pcg->latitude = g_locate_net_info.anchor_position[3];
								pcg->longitude = g_locate_net_info.anchor_position[4];
								pcg->height = g_locate_net_info.anchor_position[5];		
								WGSToECEF(pcg, pcc);									
								ECEFToENU(pcc, center, pct); //转换为东北天坐标

								if(dis0 >= dis1)
								{
									hight = g_device_config.tag_h - g_locate_net_info.main_anchor_h;
								}
								else
								{
									hight = g_device_config.tag_h - g_locate_net_info.sub_anchor_h;
								}
								
								rett = CalcPosition_enu(d0,pct->easting,pct->northing,pct->upping,d1,hight,g_locate_net_info.on_left,clua_x_y_z);
								if(rett < 0)
								{
									g_pos_info.tag_position[0] = 0;
									g_pos_info.tag_position[1] = 0;
									g_pos_info.tag_position[2] = 0;	
									g_pos_info.tag_position_valid_flag = POS_INVALID;
									DBG_PRINT("6. main_%d dis: %d cm,sub_%d: %d cm, tag pos is invalid value, rett: %d\r\n\r\n", g_locate_net_info.main_anchor_id,dis0, g_locate_net_info.sub_anchor_id,dis1, rett);
								}
								else
								{								
									//坐标滤波
									pct->easting = clua_x_y_z[0];
									pct->northing = clua_x_y_z[1];
									pct->upping = clua_x_y_z[2];
									ENUToECEF(pcc, center, pct);
									ECEFToWGS(pcg, pcc);
									
							//		pcg->latitude = KalmanFilter(pcg->latitude,g_device_config.kalman_q,g_device_config.kalman_r,0); //卡尔曼滤波
							//		pcg->longitude = KalmanFilter(pcg->longitude,g_device_config.kalman_q,g_device_config.kalman_r,1); 
							//		pcg->height = KalmanFilter(pcg->height,g_device_config.kalman_q,g_device_config.kalman_r,2); 
									
									g_pos_info.tag_position[0] = pcg->latitude;
									g_pos_info.tag_position[1] = pcg->longitude;
									g_pos_info.tag_position[2] = pcg->height;
									g_pos_info.tag_position_valid_flag = POS_VALID;
	
									DBG_PRINT("6. main_%d dis: %d cm,sub_%d dis: %d cm, tag pos latitude: %lf, longitude: %lf, height: %lf m\r\n", g_locate_net_info.main_anchor_id,dis0, g_locate_net_info.sub_anchor_id,dis1, g_pos_info.tag_position[0], g_pos_info.tag_position[1], g_pos_info.tag_position[2]);							
								}

								//upload_efs_env_to_server();

								time_end1 = get_cur_time();
								nus1 = get_count_time(time_start1, time_end1);
								
								time_end = get_cur_time();
								nus = get_count_time(time_start, time_end);		
								position_interval = get_count_time(g_last_tag_position_time, get_cur_time());	
								g_last_tag_position_time = get_cur_time();

								
								DBG_PRINT("tag postion calc take time : %d us , pos_intval : %d us\r\n", nus1,position_interval);

								GLOBAL_FREE(pcc);
								GLOBAL_FREE(center);
								GLOBAL_FREE(pcg);
								GLOBAL_FREE(pct);

								
								ret = RET_SUCCESS;
								goto end;
			
						}

						default:
								break;
			}

		}

end:		
		DBG_PRINT("exit, state :%d\r\n", state);
		return ret;   
}



ret_t tag_test_ranging(void) 
{	   			
		u8 package_buf[PACKAGE_BUF_SIZE];
		u8 buf[BUF_SIZE] = {0};
		u16 package_buf_len;
		u16 buf_len;
		u8 dynamic_slot;
		u16 target_id;
		u16 src_id;
		u16 tag_id;
		u8 src_device_type;
		u8 src_device_sub_type; 
		u8 cmd;
		u16 tag_wait_time_us;
		u32 time_start, time_end, nus, timeout; 	
		ret_t ret = RET_SUCCESS;
		u32 time_out;
		u32 state = 0;
		u32 Time_ts[6];  					//飞行时间缓存记录
		u32 Time_ts2[6];  					//飞行时间缓存记录
		u32 dm9000_cur_time, dm9000_start_time, tx_time, time;
		int32_t dis0, dis1;
		u32 ranging_cnt = 0;
		u32 tmp = 0;
		u8 i;

		
		//2.测距
		while(1)
		{	
				if(g_ranging_flag == 0)
				{
					buf[0] = STOP_TEST_RANGING;
					make_package(POLL, Flash_Device_ID, g_locate_net_info.main_anchor_id, TAG, TAG, buf, 1, package_buf, &package_buf_len);

					for(i=0; i<5; i++)
					{
						dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
						delay_ms(1);
					}

					return RET_SUCCESS;
				}
		
				switch(state)
				{
						//0.发送POLL帧
						case 0: 
						{	
								time_start = get_cur_time();
								buf[0] = START_TEST_RANGING;
								ret = make_package(POLL, Flash_Device_ID, g_locate_net_info.main_anchor_id, TAG, TAG, buf, 1, package_buf, &package_buf_len);
								if(ret != RET_SUCCESS)
								{
									break;
								}
								g_get_tx_timestamp_flag = 1;
								ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
								if(ret != RET_SUCCESS)
								{
									break;
								}
								
								Time_ts[0] = g_tx_timestamp;										//获得POLL发送时间T1			
								state=1;
					
								break;
						}			

						//1.接受主基站RESP帧		
						case 1:
						{	
								g_get_rx_timestamp_flag = 1;
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
								if(ret != RECV_TIME_OUT)
								{
									DBG_PRINT("short data RSSI:%2.2f\r\n", g_uwb_rg_ssi);
								}
								
								if(ret != RET_SUCCESS)
								{
									break;									
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == RESP) && \
									(target_id == Flash_Device_ID)&& \
									(src_device_type == ANCHOR) && \
									(buf_len == 4)))
								{
									break;
								}

								Time_ts[1] = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];//获得主基站POLL接收时间T2	
								Time_ts[3] = g_rx_timestamp;	//获得接受到主基站RESP时间T4												
								state=2;

					
								break;

						}


						//2.发送FINAL帧
		        case 2:
						{
					//		printf("%d\n",SYS_Calculate_ACTIVE_FLAG); 
								ret = make_package(FINAL, Flash_Device_ID, g_locate_net_info.main_anchor_id, TAG, TAG, NULL, 0, package_buf, &package_buf_len);
								if(ret != RET_SUCCESS)
								{
									break;
								}
								g_get_tx_timestamp_flag = 1;
								ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
								if(ret != RET_SUCCESS)
								{
									break;
								}
								
								Time_ts[4] = g_tx_timestamp;										//获得FINAL发送时间T5
								state=3;
					
								break;


						}

						//3.接受主基站ACK帧
						case 3:
						{
								//printf("%d\n",SYS_Calculate_ACTIVE_FLAG);
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
								if(ret != RECV_TIME_OUT)
								{
									DBG_PRINT("long data RSSI:%2.2f\r\n", g_uwb_rg_ssi);
								}

								
								if(ret != RET_SUCCESS)
								{
									break;
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == ACK) && \
									(target_id == Flash_Device_ID)&& \
									(src_device_type == ANCHOR) && \
									(buf_len == 52)))
								{
									break;
								}
								
								Time_ts[2] = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];//获得主基站RESP帧发送时间T3
								Time_ts[5] = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];//获得主基站RESP帧发送时间T6	
								state=4;
			
								break;					
						}


						//4.计算标签与主基站、从基站的距离、计算
						case 4:
						{
								//uint32 Time_ts_F[6];
								double Ra, Rb, Da, Db;
		            uint64 tof_dtu;
								double tof;
								double distance,dist2;
								s32 dis0, dis1;
								double clua_x_y[2];
								u8 cla_flag=0;

								if(Time_ts[3] >= Time_ts[0])
								{
									Ra = (double)(Time_ts[3] - Time_ts[0]);//Tround1 = T4 - T1
								}
								else
								{
									Ra = (double)((0xffffffff - Time_ts[0]) + Time_ts[3] + 1);
								}	

								if(Time_ts[5] >= Time_ts[2])
								{
									Rb = (double)(Time_ts[5] - Time_ts[2]);//Tround2 = T6 - T3
								}
								else
								{
									Rb = (double)((0xffffffff - Time_ts[2]) + Time_ts[5] + 1);
								}	

								if(Time_ts[4] >= Time_ts[3])
								{
									Da = (double)(Time_ts[4] - Time_ts[3]);//Treply2 = T5 - T4
								}
								else
								{
									Da = (double)((0xffffffff - Time_ts[3]) + Time_ts[4] + 1);
								}	

								if(Time_ts[2] >= Time_ts[1])
								{
									Db = (double)(Time_ts[2] - Time_ts[1]);//Treply1 = T3 - T2
								}
								else
								{
									Db = (double)((0xffffffff - Time_ts[1]) + Time_ts[2] + 1);
								}									

#if 0
								Ra = (double)(Time_ts[3] - Time_ts[0]);//Tround1 = T4 - T1  
		            Rb = (double)(Time_ts[5] - Time_ts[2]);//Tround2 = T6 - T3 
		            Da = (double)(Time_ts[4] - Time_ts[3]);//Treply2 = T5 - T4  
		            Db = (double)(Time_ts[2] - Time_ts[1]);//Treply1 = T3 - T2  
#endif
		            tof_dtu = (uint64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));//计算公式
		            tof = tof_dtu * DWT_TIME_UNITS;
		            distance = tof * SPEED_OF_LIGHT;//距离=光速*飞行时间
								dist2 = distance - dwt_getrangebias(config.chan,(float)distance, config.prf);//距离减去矫正系数	
								dis0 = dist2*100;//dis 为单位为cm的距离		


								//距离低通滤波
							//	dis0 = LP(dis0,0);
							
								ranging_cnt++;
								tmp += dis0;

								if(ranging_cnt >= 1000)
								{
									tmp = tmp / ranging_cnt;				
									DBG_PRINT("----------------------------ave distance : %d\r\n", tmp);
									ranging_cnt = 0;	
									tmp = 0;
								}

								DBG_PRINT("dis0: %d\r\n", dis0);
								osDelay(100);
								state = 5;
								
								ret = RET_SUCCESS;
								break;
			
						}

						default:
								break;
			}

			
				if(state != 5)
				{
					if(ret != RET_SUCCESS)
					{
						DBG_PRINT("faile, state :%d\r\n", state);
						state = 0;
					}
				}
				else
				{
					state = 0;
				} 	

		}

end:		
		
		return ret;   
}




ret_t tag_test_111byte_ranging(void) 
{	   			
		u8 package_buf[PACKAGE_BUF_SIZE];
		u8 buf[BUF_SIZE] = {0};
		u16 package_buf_len;
		u16 buf_len;
		u8 dynamic_slot;
		u16 target_id;
		u16 src_id;
		u16 tag_id;
		u8 src_device_type;
		u8 src_device_sub_type; 
		u8 cmd;
		u16 tag_wait_time_us;
		u32 time_start, time_end, nus, timeout; 	
		ret_t ret = RET_SUCCESS;
		u32 time_out;
		u32 state = 0;
		u32 Time_ts[6];  					//飞行时间缓存记录
		u32 Time_ts2[6];  					//飞行时间缓存记录
		u32 dm9000_cur_time, dm9000_start_time, tx_time, time;
		int32_t dis0, dis1;
		float uwb_rg_ssi;
		static u32 ranging_cnt = 0;
		static u32 tmp = 0;
		
		
		//2.测距
		while(1)
		{	
				switch(state)
				{
						//0.发送POLL帧
						case 0: 
						{	
								time_start = get_cur_time();
								ret = make_package(POLL, Flash_Device_ID, g_locate_net_info.main_anchor_id, TAG, TAG, NULL, 0, package_buf, &package_buf_len);
								if(ret != RET_SUCCESS)
								{
									goto end;
								}
								g_get_tx_timestamp_flag = 1;
								ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
								if(ret != RET_SUCCESS)
								{
									goto end;
								}
								
								Time_ts[0] = g_tx_timestamp;										//获得POLL发送时间T1			
								state=1;
					
								break;
						}			

						//1.接受主基站RESP帧		
						case 1:
						{	
								g_get_rx_timestamp_flag = 1;
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
								if(ret != RET_SUCCESS)
								{
									goto end;
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == RESP) && \
									(target_id == Flash_Device_ID)&& \
									(src_device_type == ANCHOR) && \
									(buf_len == 4)))
								{
									goto end;
								}

								Time_ts[1] = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];//获得主基站POLL接收时间T2	
								Time_ts[3] = g_rx_timestamp;	//获得接受到主基站RESP时间T4												
								state=2;
								
								uwb_rg_ssi = g_uwb_rg_ssi;

								DBG_PRINT("short data RSSI:%2.2f\r\n", uwb_rg_ssi);
					
								break;

						}


						//2.发送FINAL帧
		        case 2:
						{
					//		printf("%d\n",SYS_Calculate_ACTIVE_FLAG); 
								ret = make_package(FINAL, Flash_Device_ID, g_locate_net_info.main_anchor_id, TAG, TAG, NULL, 0, package_buf, &package_buf_len);
								if(ret != RET_SUCCESS)
								{
									goto end;
								}
								g_get_tx_timestamp_flag = 1;
								ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
								if(ret != RET_SUCCESS)
								{
									goto end;
								}
								
								Time_ts[4] = g_tx_timestamp;										//获得FINAL发送时间T5
								state=3;
					
								break;


						}

						//3.接受主基站ACK帧
						case 3:
						{
								//printf("%d\n",SYS_Calculate_ACTIVE_FLAG);
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
								if(ret != RET_SUCCESS)
								{
									goto end;
								}

								uwb_rg_ssi = g_uwb_rg_ssi;
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == ACK) && \
									(target_id == Flash_Device_ID)&& \
									(src_device_type == ANCHOR) && \
									(buf_len == 100)))
								{
									goto end;
								}
								
								Time_ts[2] = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];//获得主基站RESP帧发送时间T3
								Time_ts[5] = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];//获得主基站RESP帧发送时间T6	
								state=4;
					
								break;					
						}


						//4.计算标签与主基站、从基站的距离、计算
						case 4:
						{
								//uint32 Time_ts_F[6];
								double Ra, Rb, Da, Db;
		            int64 tof_dtu;
								s32 dist2;
								double distance,dist; 
								double tof; 
								double clua_x_y[2];
								u8 cla_flag=0;

								Ra = (double)(Time_ts[3] - Time_ts[0]);//Tround1 = T4 - T1  
		            Rb = (double)(Time_ts[5] - Time_ts[2]);//Tround2 = T6 - T3 
		            Da = (double)(Time_ts[4] - Time_ts[3]);//Treply2 = T5 - T4  
		            Db = (double)(Time_ts[2] - Time_ts[1]);//Treply1 = T3 - T2  
		            tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));//计算公式
		            tof = tof_dtu * DWT_TIME_UNITS;
		            distance = tof * SPEED_OF_LIGHT;//距离=光速*飞行时间
								dist2 = distance - dwt_getrangebias(config.chan,(float)distance, config.prf);//距离减去矫正系数	
								dis0 = dist2*100;//dis 为单位为cm的距离		


								//距离低通滤波
							//	dis0 = LP(dis0,0);

								ranging_cnt++;
								tmp += dis0;

								if(ranging_cnt >= 1000)
								{
									tmp = tmp / ranging_cnt;				
									DBG_PRINT("----------------------------ave distance : %d\r\n", tmp);
									ranging_cnt = 0;	
									tmp = 0;
								}
								
								DBG_PRINT("dis0: %d, long data RSSI:%2.2f\r\n", dis0, uwb_rg_ssi);
								osDelay(100);
	
								ret = RET_SUCCESS;
								goto end;
			
						}

						default:
								break;
			}

		}

end:		
		DBG_PRINT("exit, state :%d\r\n", state);
		return ret;   
}



ret_t m_anchor_ranging(void)
{
		uint32 i;
		u8 package_buf[PACKAGE_BUF_SIZE];
		u8 buf[BUF_SIZE] = {0};
		u16 package_buf_len;
		u16 buf_len;
		u8 dynamic_slot;
		u8 slot;
		u16 target_id;
		u16 src_id;
		u16 tag_id;
		u8 src_device_type;
		u8 src_device_sub_type; 
		u8 cmd;
		u32 time_start, time_end, nus, timeout; 	
		ret_t ret = RET_SUCCESS;
		u32 time_out;
		u32 T2, T3, T6;
		u32 delay_tx_time;		
		u32 state = 0;

		time_start = get_cur_time();
		while(1)
		{		
				time_end = get_cur_time();
				nus = get_count_time(time_start, time_end);
				if(nus > TAG_RANGING_TIMEOUT)
				{				
					ret = TIME_OUT;
					DBG_PRINT("MA: range all time out,%d/%d(unit:1us)\r\n",nus,TAG_RANGING_TIMEOUT);
				 	goto end;
				} 	

				if((TAG_RANGING_TIMEOUT - nus) > 0xffff)
				{
						time_out = 0xffff;
				}
				else
				{
						time_out = (TAG_RANGING_TIMEOUT - nus);
				}
		
				
				switch(state)
				{
						//0.接受标签POLL帧
						case 0:
						{
								g_get_rx_timestamp_flag = 1;
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, time_out);
								if(ret != RET_SUCCESS)
								{
									state=0;
									DBG_PRINT("5-0 MA: poll data rcv failed0! ret=%s\r\n",ret_type_str[ret]);
									break;
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == POLL) && \
									(target_id == Flash_Device_ID) && \
									(src_device_type == TAG) && (src_device_sub_type == TAG)))
								{
									DBG_PRINT("state :%d\r\n", state);
									state=0;
									DBG_PRINT("5-0 MA: poll data rcv failed1: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
													ret_type_str[ret],
													dm_comm_type_str[cmd],
													target_id,src_id,
													device_type_str[src_device_type],
													anchor_type_str[src_device_sub_type],buf_len);
									break;
								}						

								T2 = g_rx_timestamp;	//获得接受到标签POLL帧时间T2
								g_recv_tag_id = src_id;
								state=1;
								DBG_PRINT("5-0 MA: poll data rcv suc, sid:%d , stamp2:%u \r\n",src_id,T2);

								break;
						}


						//1.发送RESP帧
						case 1: 
						{
								//printf("%d\n",SYS_Calculate_ACTIVE_FLAG); 
								buf[0] = T2 >> 24;
								buf[1] = T2 >> 16;
								buf[2] = T2 >> 8;
								buf[3] = T2 & 0xff;
								ret = make_package(RESP, Flash_Device_ID, g_recv_tag_id, ANCHOR, g_anchor_type, buf, 4, package_buf, &package_buf_len);
								if(ret != RET_SUCCESS)
								{
									state=0;
									break;

								}

								delay_us(50);
								g_get_tx_timestamp_flag = 1;
								ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
								if(ret != RET_SUCCESS)
								{
									DBG_ERR_PRINT("5-1 mresp data send fail !\r\n");
									state=0;
									break;
								}

								T3 = g_tx_timestamp;										//获得RESP帧接受时间T3
								state=2;
								DBG_PRINT("5-1. mresp data send suc, tid:%d , stamp3:%u \r\n",g_recv_tag_id,T3);

								break;
								
						}			


						//2.接受从基站RESP帧
						case 2: 
						{
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
								if(ret != RET_SUCCESS)
								{
									state=0;
									DBG_ERR_PRINT("5-2 MA: sresp data rcv failed0! ret=%s\r\n",ret_type_str[ret]);
									break;
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == RESP) && \
									(src_id == (Flash_Device_ID + 1)) && \
									(src_device_type == ANCHOR) && (src_device_sub_type == SUB_ANCHOR)))
								{
									state=0;
									DBG_ERR_PRINT("5-2 MA: sresp data rcv failed1: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
													ret_type_str[ret],
													dm_comm_type_str[cmd],
													target_id,src_id,
													device_type_str[src_device_type],
													anchor_type_str[src_device_sub_type],buf_len);
									break;
								}						
								state=3;

								break;

						}						

						//3.接受FINAL帧
						case 3: 
						{
								g_get_rx_timestamp_flag = 1;
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
								if(ret != RET_SUCCESS)
								{
									state=0;
									DBG_ERR_PRINT("5-3 MA: final data rcv failed0! ret=%s\r\n",ret_type_str[ret]);
									break;
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == FINAL) && \
									(target_id == Flash_Device_ID) && \
									(src_device_type == TAG) && (src_device_sub_type == TAG)))
								{
									state=0;
									DBG_ERR_PRINT("5-3 MA: final data rcv failed1: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
													ret_type_str[ret],
													dm_comm_type_str[cmd],
													target_id,src_id,
													device_type_str[src_device_type],
													anchor_type_str[src_device_sub_type],buf_len);
									break;
								}						

								T6 = g_rx_timestamp;	//获得FINAL帧接受时间T6
								state=4;
								DBG_PRINT("5-3 MA: final data rcv suc , sid:%d , stamp6:%u \r\n",src_id,T6);

								break;

						}

						//4.发送ACK帧
						case 4:
						{
								//printf("%d\n",SYS_Calculate_ACTIVE_FLAG); 
								buf[0] = T3 >> 24;
								buf[1] = T3 >> 16;
								buf[2] = T3 >> 8;
								buf[3] = T3 & 0xff;
								buf[4] = T6 >> 24;
								buf[5] = T6 >> 16;
								buf[6] = T6 >> 8;
								buf[7] = T6 & 0xff;
								
								ret = make_package(ACK, Flash_Device_ID, g_recv_tag_id, ANCHOR, g_anchor_type, buf, 8, package_buf, &package_buf_len);
								if(ret != RET_SUCCESS)
								{
									state=0;
									break;
								}

								delay_us(50);
								ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);		
								if(ret != RET_SUCCESS)
								{
									state=0;
									DBG_ERR_PRINT("5-4. mack data send fail!  \r\n");
									break;
								}

								ret = RET_SUCCESS;
								state=5;
								DBG_PRINT("5-4. mack data send suc, tid:%d  \r\n",g_recv_tag_id);
								break;

						}

						//5.接受从基站ACK帧
						case 5: 
						{
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
								if(ret != RET_SUCCESS)
								{
									state=0;
									DBG_ERR_PRINT("5-5 MA: sack data rcv failed0! ret=%s\r\n",ret_type_str[ret]);
									continue;
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == ACK) && \
									(src_id == (Flash_Device_ID + 1)) && \
									(src_device_type == ANCHOR) && (src_device_sub_type == SUB_ANCHOR)))
								{
									state=0;
									DBG_ERR_PRINT("5-5 MA: sack data rcv failed1: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
													ret_type_str[ret],
													dm_comm_type_str[cmd],
													target_id,src_id,
													device_type_str[src_device_type],
													anchor_type_str[src_device_sub_type],buf_len);
									continue;
								}						
								state=0;
								DBG_PRINT("5-5. sack data rcv suc, sid:%d , tid:%d \r\n",src_id,target_id);

								break;

						}								

						default:
								break;

				}
				
		}

end:
		DBG_PRINT("MA exit, state :%d, range all done!\r\n", state);
		return ret;	
}





ret_t anchor_test_ranging(void)
{
		uint32 i;
		u8 package_buf[PACKAGE_BUF_SIZE];
		u8 buf[BUF_SIZE] = {0};
		u16 package_buf_len;
		u16 buf_len;
		u8 dynamic_slot;
		u8 slot;
		u16 target_id;
		u16 src_id;
		u16 tag_id;
		u8 src_device_type;
		u8 src_device_sub_type; 
		u8 cmd;
		u8 slot_num;
		u32 time_start, time_end, nus, timeout; 	
		ret_t ret = RET_SUCCESS;
		u32 time_out;
		u32 T2, T3, T6;
		u32 delay_tx_time;		
		u32 state = 0;
		float uwb_rg_ssi;


		while(1)
		{					
				switch(state)
				{
						//0.接受标签POLL帧
						case 0:
						{
								g_get_rx_timestamp_flag = 1;
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, osWaitForever);
								if(ret != RECV_TIME_OUT)
								{
									DBG_PRINT("short data RSSI:%2.2f\r\n", g_uwb_rg_ssi);
								}

								
								if(ret != RET_SUCCESS)
								{
									break;
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == POLL) && (buf_len == 1) && \
									(src_device_type == TAG) && (src_device_sub_type == TAG)))
								{
									break;
								}	

								if(buf[0] == STOP_TEST_RANGING)
								{
									return RET_SUCCESS;
								}
							

								T2 = g_rx_timestamp;	//获得接受到标签POLL帧时间T2
								g_recv_tag_id = src_id;
								state=1;

								break;
						}


						//1.发送RESP帧
						case 1: 
						{
								//printf("%d\n",SYS_Calculate_ACTIVE_FLAG); 
								buf[0] = T2 >> 24;
								buf[1] = T2 >> 16;
								buf[2] = T2 >> 8;
								buf[3] = T2 & 0xff;
								ret = make_package(RESP, Flash_Device_ID, g_recv_tag_id, ANCHOR, g_anchor_type, buf, 4, package_buf, &package_buf_len);
								if(ret != RET_SUCCESS)
								{
									break;

								}
								g_get_tx_timestamp_flag = 1;
								ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
								if(ret != RET_SUCCESS)
								{
									break;
								}

								T3 = g_tx_timestamp;										//获得RESP帧接受时间T3
								state=2;

								break;
								
						}						

						//2.接受FINAL帧
						case 2: 
						{
								g_get_rx_timestamp_flag = 1;
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
								if(ret != RECV_TIME_OUT)
								{
									DBG_PRINT("short data RSSI:%2.2f\r\n", g_uwb_rg_ssi);
								}
								
								if(ret != RET_SUCCESS)
								{
									break;
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == FINAL) && \
									(src_device_type == TAG) && (src_device_sub_type == TAG)))
								{
									break;
								}						

								T6 = g_rx_timestamp;	//获得FINAL帧接受时间T6
								state=3;

								break;

						}

						//3.发送ACK帧
						case 3:
						{
								//printf("%d\n",SYS_Calculate_ACTIVE_FLAG); 
								buf[0] = T3 >> 24;
								buf[1] = T3 >> 16;
								buf[2] = T3 >> 8;
								buf[3] = T3 & 0xff;
								buf[4] = T6 >> 24;
								buf[5] = T6 >> 16;
								buf[6] = T6 >> 8;
								buf[7] = T6 & 0xff;
								
								ret = make_package(ACK, Flash_Device_ID, g_recv_tag_id, ANCHOR, g_anchor_type, buf, 52, package_buf, &package_buf_len);
								if(ret != RET_SUCCESS)
								{
									break;
								}
										
								ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);		
								if(ret != RET_SUCCESS)
								{
									break;
								}

								state = 4;
								ret = RET_SUCCESS;
								
								break;

						}
						
						default:
								break;

				}

				if(state != 4)
				{
					if(ret != RET_SUCCESS)
					{
						DBG_PRINT("faile, state :%d\r\n", state);
						state = 0;
					}
				}
				else
				{
					state = 0;
				} 	

		}


		return ret;	
}



void auto_choose_slave_tx_power(void)
{
		u8 package_buf[PACKAGE_BUF_SIZE];
		u8 buf[BUF_SIZE] = {0};
		u16 package_buf_len;
		u16 buf_len;
		u16 target_id;
		u16 src_id;
		u16 tag_id;
		u8 src_device_type;
		u8 src_device_sub_type; 
		u8 cmd;
		u32 tx_power;
		u32 w_reg_value, r_reg_value;
		u8 tx_power_str[20]; 
		float uwb_rg_ssi = 0;
		float main_uwb_rg_ssi = 0;
		float tx_power_db = 0;	
		u8 state = 0;
		u32 cnt = 0;	
		float main_device_rssi_threshold;
		float sub_device_rssi_threshold;
		float main_device_rssi_rang;		
		float sub_device_rssi_rang; 		
		dwt_auto_tx_power_t dwt_auto_tx_power_state;
	
		ret_t ret = RET_SUCCESS;

		while(1)
		{
			switch(state)
			{
				case 0:	//接受开始启动自动选择tx_power帧
				{
						ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, osWaitForever);
						if(ret != RET_SUCCESS)
						{
							break;
						}	
						
						ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
						if(!((ret == RET_SUCCESS) && (cmd == AUTO_CHOOSE_TX_POWER) &&\
							(src_device_type == ANCHOR) && (src_device_sub_type == MAIN_ANCHOR) && (src_id == 0xffff) && (target_id == 0xfffe)))
						{
							if(buf[0] == STOP_AUTO_CHOOSE_TX_POWER)
							{
								return;
							}
							else if(buf[0] == SIGNAL_RSSI_FRAME)
							{
								state = 1;
								break;
							}
							
							break;
						} 

						if(buf[0] == STOP_AUTO_CHOOSE_TX_POWER)
						{
							return;
						}
						else if(buf[0] == SIGNAL_RSSI_FRAME)
						{
							state = 1;
							break;
						}
						else if(buf[0] == START_AUTO_CHOOSE_TX_POWER)
						{
							memcpy((u8*)&main_device_rssi_threshold, (u8*)&buf[1], 4);
							memcpy((u8*)&sub_device_rssi_threshold, (u8*)&buf[5], 4);
							memcpy((u8*)&main_device_rssi_rang, (u8*)&buf[9], 4);
							memcpy((u8*)&sub_device_rssi_rang, (u8*)&buf[13], 4);

							tx_power_db =33.5;
							sprintf(tx_power_str,"%2.1f %2.1f %2.1f %2.1f",tx_power_db,tx_power_db,tx_power_db,tx_power_db);							
							tx_power = calc_tx_power_config_value(tx_power_str);	
							dwt_write32bitreg(TX_POWER_ID, tx_power);
							r_reg_value = dwt_read32bitreg(TX_POWER_ID);
							if(tx_power != r_reg_value)
							{			
								DBG_PRINT("tx_power config is failed\r\n");			
							}
							
							state = 1;

						}
				}
				break;

				case 1:	//发送信号帧
				{	
						buf[0] = SIGNAL_FRAME;
						memcpy((u8*)&buf[1], (u8*)&r_reg_value, 4);
						make_package(AUTO_CHOOSE_TX_POWER, 0xfffe, 0xffff, ANCHOR, SUB_ANCHOR, buf, 52, package_buf, &package_buf_len);
						ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
						if(ret != RET_SUCCESS)
						{	
							state = 2;
							break;
						}

						state = 2;		
				}
				break;
						
				case 2:	//接受信号强度帧，并配置tx_power
				{
						ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
						if(ret != RET_SUCCESS)
						{
							tx_power_db += 0.5; 
							if(tx_power_db >33.5)
											tx_power_db = 33.5;
						}	
						else
						{		
							ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
							if(!((ret == RET_SUCCESS) && (cmd == AUTO_CHOOSE_TX_POWER) &&\
								(src_device_type == ANCHOR) && (src_device_sub_type == MAIN_ANCHOR) && (src_id == 0xffff) && (target_id == 0xfffe)))
							{
								state = 1;
								break;
							}

							if(buf[0] == STOP_AUTO_CHOOSE_TX_POWER)
							{
								return;
							}		

							if(buf[0] == START_AUTO_CHOOSE_TX_POWER)
							{
								state = 0;
								break;
							}							

							if(buf[0] == SIGNAL_RSSI_OK)
							{
	
								g_device_config.tx_power = tx_power;
								save_device_para();	
								DBG_PRINT("tx_power config is suc, config value: 0x%x\r\n",  g_device_config.tx_power);
				
								state = 3;
								break;
							}		

							memcpy((u8*)&uwb_rg_ssi, (u8*)&buf[1], 4);
							DBG_PRINT("this device RSSI:%2.2f\r\n", uwb_rg_ssi);
							if(uwb_rg_ssi < sub_device_rssi_threshold && uwb_rg_ssi > (sub_device_rssi_threshold - sub_device_rssi_rang))
							{
								state = 1;
								break;
							} 
							else
							{
								if(uwb_rg_ssi <=  (sub_device_rssi_threshold - sub_device_rssi_rang))
								{
									tx_power_db += 0.5;
									if(tx_power_db >33.5)
											tx_power_db = 33.5;
								}
								else if(uwb_rg_ssi >= sub_device_rssi_threshold)
								{ 		
									tx_power_db -= 0.5;
									if(tx_power_db < 0)
											tx_power_db = 0;
								}

								
							}							

						}

						sprintf(tx_power_str,"%2.1f %2.1f %2.1f %2.1f",tx_power_db,tx_power_db,tx_power_db,tx_power_db);							
						tx_power = calc_tx_power_config_value(tx_power_str);	
						dwt_write32bitreg(TX_POWER_ID, tx_power);
						r_reg_value = dwt_read32bitreg(TX_POWER_ID);
						if(tx_power != r_reg_value)
						{			
							DBG_PRINT("tx_power config is failed\r\n");			
						}
									
						state = 1;
				}
				break;


				case 3://接受主设备信号帧
				{
					ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
					if(ret != RET_SUCCESS)
					{
						break;
					} 
					
					ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
					if(!((ret == RET_SUCCESS) && (cmd == AUTO_CHOOSE_TX_POWER)&&\
						(src_device_type == ANCHOR) && (src_device_sub_type == MAIN_ANCHOR) && (src_id == 0xffff) && (target_id == 0xfffe)))
					{
						break;
					}

					if(buf[0] == STOP_AUTO_CHOOSE_TX_POWER)
					{
						return;
					}	

					if(buf[0] == START_AUTO_CHOOSE_TX_POWER)
					{
						state = 0;
						break;
					}					

					
					if(dwt_auto_tx_power_state == SIGNAL_RSSI_OK)
					{
						break;		
					}

					main_uwb_rg_ssi = g_uwb_rg_ssi;
					if(g_uwb_rg_ssi < main_device_rssi_threshold && g_uwb_rg_ssi > (main_device_rssi_threshold - main_device_rssi_rang))
					{
						cnt++;
						uwb_rg_ssi += g_uwb_rg_ssi;
						if(cnt >= 100)
						{
							main_uwb_rg_ssi = uwb_rg_ssi /100;											
							DBG_PRINT("main device tx_power choose is finished, RSSI:%2.2f\r\n", main_uwb_rg_ssi);
							dwt_auto_tx_power_state = SIGNAL_RSSI_OK;
							cnt = 0;
							state = 4;
							break;
						}
					}
					else
					{
						cnt = 0;
						uwb_rg_ssi = 0;
					}							
						
					state = 4;
					dwt_auto_tx_power_state = SIGNAL_RSSI_FRAME;
							
				}	
				break;

				case 4:	//发送信号强度帧
				{	
						buf[0] = dwt_auto_tx_power_state;
						memcpy(&buf[1], (u8*)&main_uwb_rg_ssi, 4);
						make_package(AUTO_CHOOSE_TX_POWER, 0xfffe, 0xffff, ANCHOR, MAIN_ANCHOR, buf, 5, package_buf, &package_buf_len);	
						ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
						if(ret != RET_SUCCESS)
						{	
							state = 3;
							break;
						}
					
						state = 3;				
				}
				break;
					

				default:
					break;
			}

		}
		
}


void auto_choose_main_tx_power(void)
{
		u8 package_buf[PACKAGE_BUF_SIZE];
		u8 buf[BUF_SIZE] = {0};
		u16 package_buf_len;
		u16 buf_len, len;
		u16 target_id;
		u16 src_id;
		u16 tag_id;
		u8 src_device_type;
		u8 src_device_sub_type; 
		u8 cmd;
		u32 tx_power, sub_tx_power;
		float tx_power_db = 0;
		u32 w_reg_value, r_reg_value;
		u8 tx_power_str[20]; 
		float uwb_rg_ssi = 0;
		float sub_uwb_rg_ssi;
			
		
		u8 state = 0;
		u32 cnt = 0;
		u32 i;
		float main_device_rssi_threshold;
		float sub_device_rssi_threshold;
		float main_device_rssi_rang;		
		float sub_device_rssi_rang;
		
		dwt_auto_tx_power_t dwt_auto_tx_power_state = START_AUTO_CHOOSE_TX_POWER;
		
		ret_t ret = RET_SUCCESS;

		while(1)
		{

			main_device_rssi_threshold = g_dwt_auto_tx_power_config.main_device_rssi_threshold;
			sub_device_rssi_threshold = g_dwt_auto_tx_power_config.sub_device_rssi_threshold;
			main_device_rssi_rang = g_dwt_auto_tx_power_config.main_device_rssi_rang;		
			sub_device_rssi_rang = g_dwt_auto_tx_power_config.sub_device_rssi_rang;		
			if(g_dwt_auto_tx_power_config.auto_choose_tx_power == 0)
			{
				state = 5;
			}
		
			switch(state)
			{
				//自动选择从设备tx_power
				case 0:	//开启自动选择tx_power，或告知从设备接受信号强度
				{
						buf[0] = dwt_auto_tx_power_state;
						len = 1;
						if(dwt_auto_tx_power_state == SIGNAL_RSSI_FRAME)
						{
							memcpy((u8*)&buf[1], (u8*)&sub_uwb_rg_ssi, 4);
							len += 4;
						}
						else if(dwt_auto_tx_power_state == START_AUTO_CHOOSE_TX_POWER)
						{
							memcpy(&buf[1], (u8*)&main_device_rssi_threshold, 4);
							memcpy(&buf[5], (u8*)&sub_device_rssi_threshold, 4);
							memcpy(&buf[9], (u8*)&main_device_rssi_rang, 4);
							memcpy(&buf[13], (u8*)&sub_device_rssi_rang, 4);	
							len += 16;

							tx_power_db = 33.5;					
							sprintf(tx_power_str,"%2.1f %2.1f %2.1f %2.1f",tx_power_db,tx_power_db,tx_power_db,tx_power_db);
							tx_power = calc_tx_power_config_value(tx_power_str);	
							dwt_write32bitreg(TX_POWER_ID, tx_power);
							r_reg_value = dwt_read32bitreg(TX_POWER_ID);
							if(tx_power != r_reg_value)
							{			
								DBG_PRINT("tx_power config is failed\r\n");			
							}
							
						}
						make_package(AUTO_CHOOSE_TX_POWER, 0xffff, 0xfffe, ANCHOR, MAIN_ANCHOR, buf, len, package_buf, &package_buf_len);	
						ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
						if(ret != RET_SUCCESS)
						{		
							if(dwt_auto_tx_power_state == SIGNAL_RSSI_FRAME)
							{
								state = 1;
							}
						
							break;
						}

						state = 1;
				}
				break;

				case 1:	//接受从设备信号
				{	
						ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
						if(ret != RET_SUCCESS)
						{
							if(dwt_auto_tx_power_state == START_AUTO_CHOOSE_TX_POWER)
							{
								state = 0;
							}
							else
							{
								state = 0;
								dwt_auto_tx_power_state = SIGNAL_RSSI_FRAME;
							}
							
							break;
						} 
						
						ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
						if(!((ret == RET_SUCCESS) && (cmd == AUTO_CHOOSE_TX_POWER) && buf_len == 52 &&\
							(src_device_type == ANCHOR) && (src_device_sub_type == SUB_ANCHOR) && (src_id == 0xfffe) && (target_id == 0xffff)))
						{
							if(dwt_auto_tx_power_state == START_AUTO_CHOOSE_TX_POWER)
							{
								state = 0;
							}						
							break;
						}

						sub_uwb_rg_ssi = g_uwb_rg_ssi;
						memcpy((u8*)&sub_tx_power, (u8*)&buf[1], 4);
						DBG_PRINT("sub device tx_power:0x%x  RSSI:%2.2f\r\n",sub_tx_power, sub_uwb_rg_ssi);
						if(g_uwb_rg_ssi < sub_device_rssi_threshold && g_uwb_rg_ssi > (sub_device_rssi_threshold - sub_device_rssi_rang))
						{
							cnt++;
							uwb_rg_ssi += g_uwb_rg_ssi;
							if(cnt >= 100)
							{		
								sub_uwb_rg_ssi = uwb_rg_ssi /100;
								DBG_PRINT("slave device tx_power choose is finished, slave device tx_power:0x%x  RSSI:%2.2f\r\n", sub_tx_power, sub_uwb_rg_ssi);
								dwt_auto_tx_power_state = SIGNAL_RSSI_OK;
								cnt = 0;
								state = 2;
								break;
							}
						}
						else
						{
							uwb_rg_ssi = 0;
							cnt = 0;
						}	

						dwt_auto_tx_power_state = SIGNAL_RSSI_FRAME;
						state = 0;
				}
				break;
						
				case 2:	//告知从设备tx_power选择ok，或发送信号帧
				{
						buf[0] = dwt_auto_tx_power_state;
						len = 1;
						if(dwt_auto_tx_power_state == SIGNAL_RSSI_OK)
						{
							memcpy((u8*)&buf[1], (u8*)&uwb_rg_ssi, 4);
							len += 4;
						}
						else if(dwt_auto_tx_power_state == SIGNAL_FRAME)
						{
							memcpy((u8*)&buf[1], (u8*)&r_reg_value, 4);
							len += 4;
						}
						
						make_package(AUTO_CHOOSE_TX_POWER, 0xffff, 0xfffe, ANCHOR, MAIN_ANCHOR, buf, len, package_buf, &package_buf_len);	
						
						ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
						if(ret != RET_SUCCESS)
						{
							state = 1;
							break;
						}

						if(dwt_auto_tx_power_state == SIGNAL_RSSI_OK)
						{
						//	tx_power_db = 0.0;	
							state = 3;
						}
						else
						{
							state = 1;
						}
				}
				break;

				//自动选择主设备tx_power
				case 3:	//接受信号强度帧，并配置tx_power
				{
						ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
						if(ret != RET_SUCCESS)
						{
							if(dwt_auto_tx_power_state == SIGNAL_RSSI_OK)
							{						
								state = 4;
								break;
							}
							
							tx_power_db += 0.5; 
							if(tx_power_db >33.5)
											tx_power_db = 33.5;							
						}	
						else
						{		
							ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
							if(!((ret == RET_SUCCESS) && (cmd == AUTO_CHOOSE_TX_POWER) && (buf_len == 5) &&\
								(src_id == 0xfffe) && (target_id == 0xffff)))
							{
								if(dwt_auto_tx_power_state == SIGNAL_RSSI_OK)
								{
									state = 4;
								}

								break;
							}

							if(buf[0] == SIGNAL_RSSI_OK)
							{
								state = 5;
								memcpy((u8*)&uwb_rg_ssi, (u8*)&buf[1], 4);
								DBG_PRINT("main device tx_power choose is finished,tx_power:0x%x  RSSI:%2.2f\r\n",r_reg_value, uwb_rg_ssi);
								
								g_device_config.tx_power = tx_power;
								save_device_para();	
								DBG_PRINT("tx_power config is suc, config value: 0x%x\r\n",  g_device_config.tx_power);								
								break;
							}

							memcpy((u8*)&uwb_rg_ssi, (u8*)&buf[1], 4);
							DBG_PRINT("main device tx_power:0x%x  RSSI:%2.2f\r\n", r_reg_value, uwb_rg_ssi);
							if(uwb_rg_ssi < main_device_rssi_threshold && uwb_rg_ssi > (main_device_rssi_threshold - main_device_rssi_rang))
							{
								state = 4;
								break;
							} 
							else
							{
								if(uwb_rg_ssi <= (main_device_rssi_threshold - main_device_rssi_rang))
								{
									tx_power_db += 0.5;
									if(tx_power_db >33.5)
											tx_power_db = 33.5;									
								}
								else if(uwb_rg_ssi >= main_device_rssi_threshold)
								{ 					
									tx_power_db -= 0.5;
									if(tx_power_db < 0)
										tx_power_db = 0;
								}
							}							

						}
					

						sprintf(tx_power_str,"%2.1f %2.1f %2.1f %2.1f",tx_power_db,tx_power_db,tx_power_db,tx_power_db);							
						tx_power = calc_tx_power_config_value(tx_power_str);	
						dwt_write32bitreg(TX_POWER_ID, tx_power);
						r_reg_value = dwt_read32bitreg(TX_POWER_ID);
						if(tx_power != r_reg_value)
						{			
							DBG_PRINT("tx_power config is failed\r\n");			
						}
						else
						{
							DBG_PRINT("main device tx_power:0x%x  RSSI:%2.2f\r\n",r_reg_value, uwb_rg_ssi);
						}

						dwt_auto_tx_power_state = SIGNAL_FRAME;
									
						state = 4;

				}
				break;

				case 4:	//发送信号帧
				{
						buf[0] = dwt_auto_tx_power_state;
						len = 1;
						
						make_package(AUTO_CHOOSE_TX_POWER, 0xffff, 0xfffe, ANCHOR, MAIN_ANCHOR, buf, 52, package_buf, &package_buf_len);	
						
						ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
						if(ret != RET_SUCCESS)
						{
							state = 3;
							break;
						}

						state = 3;
						
				}
				break;				


				case 5:	//循环
				{	
						DBG_PRINT("tx_power choose is finished, slave device tx_power:%2.1f  RSSI:%2.2f,  main device tx_power:%2.1f  RSSI:%2.2f\r\n",calc_tx_power_config_value_to_db(sub_tx_power) , sub_uwb_rg_ssi, calc_tx_power_config_value_to_db(tx_power), uwb_rg_ssi);
						DBG_PRINT("auto choose tx_power is finished, please stop auto choose tx_power\r\n");	

						if(g_dwt_auto_tx_power_config.auto_choose_tx_power == 0)
						{
							buf[0] = STOP_AUTO_CHOOSE_TX_POWER;
							make_package(AUTO_CHOOSE_TX_POWER, 0xffff, 0xfffe, ANCHOR, MAIN_ANCHOR, buf, 1, package_buf, &package_buf_len);

							for(i=0; i<5; i++)
							{
								ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
								if(ret != RET_SUCCESS)
								{						 
									break;
								}
								delay_ms(1);
							}
						
							return;
						}

						delay_ms(2000);
				}
				break;
			

				default:
					break;
			}

		}
		
}




ret_t anchor_test_111byte_ranging(void)
{
		uint32 i;
		u8 package_buf[PACKAGE_BUF_SIZE];
		u8 buf[BUF_SIZE] = {0};
		u16 package_buf_len;
		u16 buf_len;
		u8 dynamic_slot;
		u8 slot;
		u16 target_id;
		u16 src_id;
		u16 tag_id;
		u8 src_device_type;
		u8 src_device_sub_type; 
		u8 cmd;
		u8 slot_num;
		u32 time_start, time_end, nus, timeout; 	
		ret_t ret = RET_SUCCESS;
		u32 time_out;
		u32 T2, T3, T6;
		u32 delay_tx_time;		
		u32 state = 0;
		float uwb_rg_ssi;

		buf[55] = 3;
		buf[99] = 5;

		while(1)
		{					
				switch(state)
				{
						//0.接受标签POLL帧
						case 0:
						{
								g_get_rx_timestamp_flag = 1;
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, osWaitForever);
								if(ret != RET_SUCCESS)
								{
									state=0;
									continue;
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == POLL) && \
									(src_device_type == TAG) && (src_device_sub_type == TAG)))
								{
									state=0;
									continue;
								}						

								T2 = g_rx_timestamp;	//获得接受到标签POLL帧时间T2
								g_recv_tag_id = src_id;
								state=1;

								uwb_rg_ssi = g_uwb_rg_ssi;

								DBG_PRINT("short data RSSI:%2.2f\r\n", uwb_rg_ssi);

								break;
						}


						//1.发送RESP帧
						case 1: 
						{
								//printf("%d\n",SYS_Calculate_ACTIVE_FLAG); 
								buf[0] = T2 >> 24;
								buf[1] = T2 >> 16;
								buf[2] = T2 >> 8;
								buf[3] = T2 & 0xff;
								ret = make_package(RESP, Flash_Device_ID, g_recv_tag_id, ANCHOR, g_anchor_type, buf, 4, package_buf, &package_buf_len);
								if(ret != RET_SUCCESS)
								{
									state=0;
									continue;

								}
								g_get_tx_timestamp_flag = 1;
								ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
								if(ret != RET_SUCCESS)
								{
									state=0;
									continue;
								}

								T3 = g_tx_timestamp;										//获得RESP帧接受时间T3
								state=2;

								break;
								
						}						

						//2.接受FINAL帧
						case 2: 
						{
								g_get_rx_timestamp_flag = 1;
								ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
								if(ret != RET_SUCCESS)
								{
									state=0;
									continue;
								}
								
								ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
								if(!((ret == RET_SUCCESS) && (cmd == FINAL) && \
									(src_device_type == TAG) && (src_device_sub_type == TAG)))
								{
									state=0;
									continue;
								}						

								T6 = g_rx_timestamp;	//获得FINAL帧接受时间T6
								state=3;

								break;

						}

						//3.发送ACK帧
						case 3:
						{
								//printf("%d\n",SYS_Calculate_ACTIVE_FLAG); 
								buf[0] = T3 >> 24;
								buf[1] = T3 >> 16;
								buf[2] = T3 >> 8;
								buf[3] = T3 & 0xff;
								buf[4] = T6 >> 24;
								buf[5] = T6 >> 16;
								buf[6] = T6 >> 8;
								buf[7] = T6 & 0xff;
								
								ret = make_package(ACK, Flash_Device_ID, g_recv_tag_id, ANCHOR, g_anchor_type, buf, 100, package_buf, &package_buf_len);
								if(ret != RET_SUCCESS)
								{
									state=0;
									continue;
								}
										
								ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);		
								if(ret != RET_SUCCESS)
								{
									state=0;
									continue;
								}

								ret = RET_SUCCESS;
								state=0;
								
								break;

						}
						
						default:
								break;

				}

		}

end:
		DBG_PRINT("exit, state :%d\r\n", state);
		return ret;	
}




ret_t s_anchor_ranging(u32 time_out_us)
{
		uint32 i;
		u8 package_buf[PACKAGE_BUF_SIZE];
		u8 buf[BUF_SIZE] = {0};
		u16 package_buf_len;
		u16 buf_len;
		u8 dynamic_slot;
		u8 slot;
		u16 target_id;
		u16 src_id;
		u16 tag_id;
		u8 src_device_type;
		u8 src_device_sub_type; 
		u8 cmd;
		u8 slot_num;
		u32 time_start, time_end, nus; 			
		u32 time_out;
		u32 delay_tx_time, delay_rx_time, cur_dw1000_time;
		static u32 T2, T3, T6;			
		static u32 state = 0;
		ret_t ret = RET_SUCCESS;

		time_start = get_cur_time();
	
		while(1)
		{
			time_end = get_cur_time();
			nus = get_count_time(time_start, time_end);
			if(nus > time_out_us) 
			{
				ret = RET_SUCCESS;
				break;
			}	

			if((time_out_us - nus) > 0xffff)
			{
					time_out = 0xffff;
			}
			else
			{
					time_out = (time_out_us - nus);
			}
		
			switch(state)
			{
					//0.接受标签POLL帧
					case 0:
					{
							g_get_rx_timestamp_flag = 1;
							ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, time_out);
							if(ret != RET_SUCCESS)
							{
								state=0;
								DBG_PRINT("3-0 SA poll data rcv failed0! ret=%s\r\n",ret_type_str[ret]);
								break;
							}
							
							ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
							if(((ret == RET_SUCCESS) && (cmd == POLL) && \
								(target_id == (Flash_Device_ID-1)) && \
								(src_device_type == TAG) && (src_device_sub_type == TAG)))
							{
								DBG_PRINT("3-0 SA poll data rcv suc: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
														ret_type_str[ret],
														dm_comm_type_str[cmd],
														target_id,src_id,
														device_type_str[src_device_type],
														anchor_type_str[src_device_sub_type],buf_len);
								T2 = g_rx_timestamp;	//获得接受到标签POLL帧时间T2
								g_recv_tag_id = src_id;
								state=1;
							}
							else if(((ret == RET_SUCCESS) && (cmd == ACTIVATION_ALL_TAG_CMD) && \
								(target_id == Flash_Device_ID) && (src_id == (Flash_Device_ID-1))&& \
								(src_device_type == ANCHOR) && (src_device_sub_type == MAIN_ANCHOR) && (buf_len == 17)))
							{	
								DBG_PRINT("3-0 SA poll data rcv suc: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
													ret_type_str[ret],
													dm_comm_type_str[cmd],
													target_id,src_id,
													device_type_str[src_device_type],
													anchor_type_str[src_device_sub_type],buf_len);
								//主基站可能未收到从基站的激活信号，一直在发送激活帧，因此，这里发送从基站的激活信号
								make_package(ACTIVATION_ALL_TAG_CMD, src_id, target_id, ANCHOR, SUB_ANCHOR, buf, 12, package_buf, &package_buf_len);
								dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);									
							}
							else
							{
								DBG_PRINT("3-0 SA poll data rcv fail: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
													ret_type_str[ret],
													dm_comm_type_str[cmd],
													target_id,src_id,
													device_type_str[src_device_type],
													anchor_type_str[src_device_sub_type],buf_len);
								state=0;
							}
							
							break;
					}			


					//1.发送RESP帧
					case 1: 
					{
							cur_dw1000_time = dwt_readsystimestamphi32();
							delay_tx_time = cur_dw1000_time + 7*DW1000_100US_TIMER_CNT;

							buf[0] = T2 >> 24;
							buf[1] = T2 >> 16;
							buf[2] = T2 >> 8;
							buf[3] = T2 & 0xff;
							ret = make_package(RESP, Flash_Device_ID, g_recv_tag_id, ANCHOR, g_anchor_type, buf, 4, package_buf, &package_buf_len);
							if(ret != RET_SUCCESS)
							{
								state=0;
								break;

							}
							g_get_tx_timestamp_flag = 1;
						//	ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
							ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_DELAYED, delay_tx_time, TX_NORMAL_TIMEOUT+700);
							if(ret != RET_SUCCESS)
							{
								state=0;
								DBG_ERR_PRINT("3-1 SA sa_resp data send fail! sid = %d , tid=%d \r\n",Flash_Device_ID,g_recv_tag_id);
								break;
							}

							T3 = g_tx_timestamp;										//获得RESP帧接受时间T3
							state=2;
							DBG_PRINT("3-1 SA sa_resp data send suc! sid = %d , tid=%d \r\n",Flash_Device_ID,g_recv_tag_id);

							break;
							
					}					

					//2.接受FINAL帧
					case 2: 
					{
							g_get_rx_timestamp_flag = 1;
							ret = dm9000_recv_data(package_buf, &package_buf_len, OPEN_RECV, 0, DM9000_RX_TIMEOUT);
							if(ret != RET_SUCCESS)
							{
								state=0;
								DBG_PRINT("3-2 SA final data rcv failed0! ret=%s\r\n",ret_type_str[ret]);
								break;
							}
							
							ret = parsing_package(package_buf, package_buf_len, &cmd, &src_id,&target_id, &src_device_type, &src_device_sub_type, buf, &buf_len);
							if(!((ret == RET_SUCCESS) && (cmd == FINAL) && \
								(target_id == (Flash_Device_ID-1)) && \
								(src_device_type == TAG) && (src_device_sub_type == TAG)))
							{
								state=0;
								DBG_ERR_PRINT("3-2 SA final data rcv failed1: %s , %s , tid:%d , sid:%d , %s , %s , len:%d\r\n", 
														ret_type_str[ret],
														dm_comm_type_str[cmd],
														target_id,src_id,
														device_type_str[src_device_type],
														anchor_type_str[src_device_sub_type],buf_len);
								break;
							}						

							T6 = g_rx_timestamp;	//获得FINAL帧接受时间T6
							state=3;
							DBG_PRINT("3-2 SA final data rcv suc! sid=%d ,tid=%d , rx_stamp=%u\r\n",src_id,target_id,T6);

							break;

					}				

					//3.发送ACK帧
					case 3:
					{
							cur_dw1000_time = dwt_readsystimestamphi32() ;
							delay_tx_time = cur_dw1000_time + 7*DW1000_100US_TIMER_CNT;
					
							buf[0] = T3 >> 24;
							buf[1] = T3 >> 16;
							buf[2] = T3 >> 8;
							buf[3] = T3 & 0xff;
							buf[4] = T6 >> 24;
							buf[5] = T6 >> 16;
							buf[6] = T6 >> 8;
							buf[7] = T6 & 0xff;
							
							ret = make_package(ACK, Flash_Device_ID, g_recv_tag_id, ANCHOR, g_anchor_type, buf, 8, package_buf, &package_buf_len);
							if(ret != RET_SUCCESS)
							{
								state=0;
								break;
							}	
						 	
							//ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_IMMEDIATE, 0, TX_NORMAL_TIMEOUT);	
							ret = dm9000_send_data(package_buf, package_buf_len, DWT_START_TX_DELAYED, delay_tx_time, TX_NORMAL_TIMEOUT+700);	
														
							if(ret != RET_SUCCESS)
							{	
								state=0;
								DBG_ERR_PRINT("%s 3-3 SA sa_ack data send fail! sid = %d , tid=%d %s \r\n",ANSI_BG_RED,Flash_Device_ID,g_recv_tag_id, ANSI_NONE);
								break;
							}

							ret = RET_SUCCESS;
							DBG_PRINT("3-3 SA sa_ack data send suc! sid = %d , tid=%d \r\n",Flash_Device_ID,g_recv_tag_id);
							state=0;
							DBG_PRINT("%d tag meas success\r\n", g_recv_tag_id);
							
							break;

					}

					default:
							break;

					

			}			

		}

end:
		
	
		return ret;	
}


u32 major_anchor_timeout_cnt=0;

ret_t major_anchor(void)
{
	ret_t ret;
	u32 time_start, time_end, nus, timeout;
	u32 time_start1, time_end1, nus1, timeout1;
	m_archor_state_t m_archor_state = MA_NET_ACTIVATION_ALL_TAG;

	g_pos_info.anchor_sub_state = m_archor_state;
	

	time_start1 = get_cur_time();
	g_time_start = dwt_readsystimestamphi32();
	
	if(g_time_start >= g_m_anchor_last_time_start)
	{
		nus = g_time_start - g_m_anchor_last_time_start;
	}
	else
	{
		nus = (0xffffffff - g_m_anchor_last_time_start) + g_time_start + 1;
	}	

	g_m_anchor_last_time_start = g_time_start;
	nus = (nus / DW1000_100US_TIMER_CNT)*100;
	DBG_PRINT("major_anchor interval %d\r\n", nus);
		
	
	while(1)
	{	
		g_time_end = dwt_readsystimestamphi32();	
		if(g_time_end >= g_time_start)
		{
			nus = g_time_end - g_time_start;
		}
		else
		{
			nus = (0xffffffff - g_time_start) + g_time_end + 1;
		}

		//DBG_PRINT("nus is %8x, g_timer_cnt_ms is %8x\r\n", nus, g_timer_cnt_ms);

		nus = nus / DW1000_100US_TIMER_CNT; //dw1000计时器高32位每0x6180个计数大概为100us
		if(nus > NETWORK_CYCLE_TIME)
		{	
			time_end1 = get_cur_time();
			nus1 = get_count_time(time_start1, time_end1);
			major_anchor_timeout_cnt++;
			DBG_PRINT("major_anchor timout: %d/%d (unit:100us) - to_cnt:%d\r\n", nus1,NETWORK_CYCLE_TIME,major_anchor_timeout_cnt);
			
			dwt_forcetrxoff(); //this will clear all events
			dwt_rxreset();
			ret = NETWORK_TIME_OUT;
			g_anchor_type = IDLE_ANCHOR;
			g_pos_info.anchor_state = g_anchor_type;
			
			break;
		}	

		g_pos_info.anchor_sub_state = m_archor_state;
			
		switch(m_archor_state)
		{
			case MA_NET_ACTIVATION_ALL_TAG:					//2.激活标签
			{ 
				DBG_PRINT("M4-1 MA_NET_ACTIVATION_ALL_TAG START!\r\n");
				
				ret = m_activation_all_tag();
				if(ret == RET_SUCCESS)
				{
					DBG_PRINT("M4-1 MA_NET_ACTIVATION_ALL_TAG OK！\r\n");
					m_archor_state = MA_NET_TAG_REGISTER;
				}	
				else
				{
					DBG_ERR_PRINT("M4-1 MA_NET_ACTIVATION_ALL_TAG FAIL！\r\n");
					m_archor_state = MA_NET_WAITE;
					g_pos_info.anchor_sub_state = m_archor_state;
					
				}

				break;
			}

			case MA_NET_TAG_REGISTER:								//3.标签注册
			{ 
				DBG_PRINT("M4-2 MA_NET_TAG_REGISTER START!\r\n");
				
				ret = m_wait_tag_register();
				if(ret == RET_SUCCESS)
				{
					DBG_PRINT("M4-2 MA_NET_TAG_REGISTER OK！\r\n");
					m_archor_state = MA_NET_SLOT_ALLOCATION;
				}	
				else
				{
					DBG_ERR_PRINT("M4-2 MA_NET_TAG_REGISTER FAIL!\r\n");
					m_archor_state = MA_NET_WAITE;
					g_pos_info.anchor_sub_state = m_archor_state;
					
				}
				break;
			}

			case MA_NET_SLOT_ALLOCATION:						//4.分配时隙
			{ 
				DBG_PRINT("M4-3 MA_NET_SLOT_ALLOCATION START!\r\n");
				
				ret = m_slot_allocation();
				if(ret == RET_SUCCESS)
				{
					DBG_PRINT("M4-3 MA_NET_SLOT_ALLOCATION OK！\r\n");
					m_archor_state = MA_NET_RANGING;
				}	
				else
				{
					DBG_ERR_PRINT("M4-3 MA_NET_SLOT_ALLOCATION FAIL\r\n");
					m_archor_state = MA_NET_WAITE;
					g_pos_info.anchor_sub_state = m_archor_state;
					
				}			
				break;
			}
			
			case MA_NET_RANGING:										//5.测距
			{ 
				DBG_PRINT("M4-4 MA_NET_RANGING START!\r\n");
				
				m_anchor_ranging();	
				m_archor_state = MA_NET_WAITE;
				g_pos_info.anchor_sub_state = m_archor_state;
				
				DBG_PRINT("M4-4 MA_NET_RANGING DONE!\r\n");
				break;
			}

			case  MA_NET_WAITE:  		//等待
			{			
			//	DBG_PRINT("MA_NET_WAITE\r\n");
				break;
			}

			default:
				ret = RET_FAILED;
				break;
		}

	}

end:
	return ret;
		

}



u32 sub_anchor_timeout_cnt=0;
ret_t sub_anchor(void)
{
	ret_t ret;
	u16 slot;
	u32 time_start, time_end, nus, timeout;
	u32 time_start1, time_end1, nus1, timeout1;
	u16 sub_active_fail_cnt = 0;
	s_archor_state_t s_archor_state = SA_NET_ACTIVATION_ALL_TAG;

	g_pos_info.anchor_sub_state = s_archor_state;

	time_start1 = get_cur_time();
	g_time_start = dwt_readsystimestamphi32();


	while(1)
	{

		if((g_dwt_auto_tx_power_config.auto_choose_tx_power == 1) || (g_ranging_flag == 1) || g_detection_signal_flag || \
			(g_interference_signal_flag == 1))
		{
			return ret;
		}		

		
		//g_time_end = g_timer_cnt_ms;
		g_time_end = dwt_readsystimestamphi32();	
		if(g_time_end >= g_time_start)
		{
			nus = g_time_end - g_time_start;
		}
		else
		{
			nus = (0xffffffff - g_time_start) + g_time_end + 1;
		}

	//	DBG_PRINT("nus is %8x, g_timer_cnt_ms is %8x\r\n", nus, g_timer_cnt_ms);
		nus = nus / DW1000_100US_TIMER_CNT; //dw1000计时器高32位每0x6180个计数大概为100us
		if(nus > NETWORK_CYCLE_TIME)
		{	

		  if(first_acitve_flag == 1 || Flash_Device_ID == 0)
		  {
		  	time_end1 = get_cur_time();
				nus1 = get_count_time(time_start1, time_end1);
				sub_anchor_timeout_cnt++;
				DBG_PRINT("sub_anchor timout: %d/%d (unit:100us) - to_cnt:%d\r\n", nus1,NETWORK_CYCLE_TIME,sub_anchor_timeout_cnt);
				
				dwt_forcetrxoff(); //this will clear all events
				dwt_rxreset();
				ret = NETWORK_TIME_OUT;			
				g_anchor_type = MAIN_ANCHOR;
				g_pos_info.anchor_state = g_anchor_type;
				g_pos_info.anchor_sub_state = 0;
			
				break;
		  }
		}	

		g_pos_info.anchor_sub_state = s_archor_state;
		switch(s_archor_state)
		{
			case SA_NET_ACTIVATION_ALL_TAG:				//2.激活标签
			{
				DBG_PRINT("S2-1 SA_NET_ACTIVATION_ALL_TAG START!\r\n");
				
				ret = s_wait_activation_all_tag();
				if(ret == RET_SUCCESS)
				{	
					DBG_PRINT("S2-1 SA_NET_ACTIVATION_ALL_TAG OK!\r\n");
					s_archor_state = SA_NET_RANGING;
					first_acitve_flag = 1;
					g_pos_info.anchor_sub_state = s_archor_state;
					
				}
				else
				{
					DBG_ERR_PRINT("S2-1 A_NET_ACTIVATION_ALL_TAG FAIL!\r\n");
					first_acitve_flag = 0; //让一直等待激活
					//sub_active_fail_cnt++;
					
					#if 0
					if(first_acitve_flag == 1 || Flash_Device_ID == 0)
					{
							s_archor_state = SA_NET_WAITE;
							g_pos_info.anchor_sub_state = s_archor_state;							
					}
					#endif
				}
				break;
			}
	
			case SA_NET_RANGING:										//3.测距
			{ 
				DBG_PRINT("S2-2 SA_NET_RANGING START!\r\n");
				s_anchor_ranging((NETWORK_CYCLE_TIME - nus - FREE_TIME)*100);
				DBG_PRINT("S2-2 SA_NET_RANGING SINGLE DONE!\r\n");
				s_archor_state = SA_NET_WAITE;
				g_pos_info.anchor_sub_state = s_archor_state;
				
				break;				

			}

			case SA_NET_WAITE:  		//等待
			{	
			//	DBG_PRINT("SA_NET_WAITE\r\n");
				break;
			}	
	
			default:
				ret = RET_FAILED;
				break;

				
		}
	}

end:		
	return ret;

}



ret_t idle_anchor(void)
{
	ret_t ret = RET_SUCCESS;

	u32 time_start, time_end, nus, timeout;
	u32 time_start1, time_end1, nus1, timeout1;
	u32 network_time_out;

	time_start1 = get_cur_time();
	g_time_start = dwt_readsystimestamphi32();

	
	
	while(1)
	{
		g_time_end = dwt_readsystimestamphi32();
		if(g_time_end >= g_time_start)
		{
			nus = g_time_end - g_time_start;
		}
		else
		{
			nus = (0xffffffff - g_time_start) + g_time_end + 1;
		}

		nus = nus / DW1000_100US_TIMER_CNT; //dw1000计时器高32位每0x6180个计数大概为100us
		if(nus < (NETWORK_CYCLE_TIME - FREE_TIME))
		{
			osDelay(10);				
		}
	
		network_time_out = NETWORK_CYCLE_TIME * g_device_config.anchor_idle_num;		

		if(nus > network_time_out)
		{	
		  time_end1 = get_cur_time();
			nus1 = get_count_time(time_start1, time_end1);
			DBG_PRINT("idle_anchor timout: %d/%d(unit:100us)\r\n", nus1,network_time_out);		
			
			dwt_forcetrxoff(); //this will clear all events
			dwt_rxreset();
			ret = NETWORK_TIME_OUT;
			g_anchor_type = SUB_ANCHOR;
			g_pos_info.anchor_state = g_anchor_type;
			break;
		}	

		
	}
		
	return ret;

}




void init_anchor_type(void)
{
	if(Flash_Device_ID == 0)
	{
			g_anchor_type = MAIN_ANCHOR;
	}
	else
	{		
			g_anchor_type = SUB_ANCHOR;
	}
	DBG_PRINT("Device Id[%d] ,Init Anchor Type : %s\r\n",Flash_Device_ID,anchor_type_str[g_anchor_type]);
}



u8 first_enter_flag = 0;


//void MODE_ANCHOR(void)
static void mode_anchor(void)
{
		/* 初次进入本函数*/
		if(first_enter_flag == 0)
		{
				first_enter_flag = 1;
							
				/*初始化基站类型*/
				init_anchor_type();	
		}
		
		if(g_dwt_auto_tx_power_config.auto_choose_tx_power == 1)
		{
			auto_choose_main_tx_power();
		}		
		else if(g_ranging_flag == 1)
		{
			tag_test_ranging();
		}		
		else if(g_detection_signal_flag == 1)
		{
			detection_signal();
		}
		else if(g_interference_signal_flag == 1)
		{
			interference_signal();
		} 	

		
	
		switch(g_anchor_type)
		{
				case MAIN_ANCHOR:
				{		
						DBG_PRINT("\r\n3-1 now is main anchor[%d]\r\n",Flash_Device_ID);
						major_anchor();
						break;
				}	
									
				case SUB_ANCHOR:
				{	
						DBG_PRINT("\r\n3-3 now is sub anchor[%d]\r\n",Flash_Device_ID);
						sub_anchor();
						break;
				}						
				
				case IDLE_ANCHOR:
				{
						DBG_PRINT("\r\n3-2 now is idle anchor[%d]\r\n",Flash_Device_ID);
						idle_anchor();
						
						break;
				}	
				
		}				

}

u32 tag_active_suc_cnt=0;
u32 tag_register_suc_cnt=0;
u32 tag_slot_suc_cnt=0;
u32 tag_range_suc_cnt=0;
static ret_t mode_tag(void)
{
	ret_t ret;
	u8 slot = 0;

	
	tag_state = TAG_WAITE_ACTIVATION;


	while(1)
	{
		if(g_device_config.device_type != TAG)
		{
			return RET_SUCCESS;
		}

		g_pos_info.tag_state = tag_state;
	
		switch(tag_state)
		{
			case TAG_WAITE_ACTIVATION:									//1.等待被激活
			{	
				DBG_PRINT("\r\n\r\n\r\n-----------------------------------------------------------\r\n");
				DBG_PRINT("all : active_cnt: %d , regi_cnt: %d, slot_cnt: %d , range_cnt : %d \r\n",tag_active_suc_cnt,tag_register_suc_cnt,tag_slot_suc_cnt,tag_range_suc_cnt);
				DBG_PRINT("4-1 TAG_WAITE_ACTIVATION START!\r\n");
				
				ret = tag_wait_activation();
				if(ret == RET_SUCCESS)
				{
					DBG_PRINT("4-1 TAG_WAITE_ACTIVATION SUC：ANCHOR_ID[%3d / %3d]\r\n", g_locate_net_info.main_anchor_id,g_locate_net_info.sub_anchor_id);
					tag_state = TAG_REGISTER;
					tag_active_suc_cnt++;
				}
				else
				{
					g_pos_info.tag_position[0] = 0;
					g_pos_info.tag_position[1] = 0;
					g_pos_info.tag_position[2] = 0; 
					g_pos_info.D0 = 0;
					g_pos_info.D1 = 0;
					g_pos_info.main_anchor_id = 0;
					g_pos_info.sub_anchor_id = 0;
					g_pos_info.tag_position_valid_flag = POS_INVALID;
			
					DBG_ERR_PRINT("4-1 TAG_WAITE_ACTIVATION FAIL!\r\n");
					tag_state = TAG_WAITE_ACTIVATION;			
				}
				
				break;
			}


			case TAG_REGISTER:													//2.注册
			{ 
				DBG_PRINT("\r\n4-2 TAG_REGISTER START!\r\n");
				
				ret = tag_register();
				if(ret == RET_SUCCESS)
				{
					DBG_PRINT("4-2 TAG_REGISTER SUC：ANCHOR_ID[%3d / %3d] \r\n", g_locate_net_info.main_anchor_id,g_locate_net_info.sub_anchor_id);
					tag_state = TAG_RECV_SLOT_ALLOCATION;
					tag_register_suc_cnt++;
				}	
				else
				{
					g_pos_info.tag_position[0] = 0;
					g_pos_info.tag_position[1] = 0;
					g_pos_info.tag_position[2] = 0; 
					g_pos_info.D0 = 0;
					g_pos_info.D1 = 0;
					g_pos_info.tag_position_valid_flag = POS_INVALID;				
					DBG_ERR_PRINT("4-2 TAG_REGISTER FAIL!\r\n");
					tag_state = TAG_WAITE_ACTIVATION;
				}
				break;
			}


			case TAG_RECV_SLOT_ALLOCATION:							//3.接受测距时隙表
			{ 
				DBG_PRINT("\r\n4-3 TAG_RECV_SLOT_ALLOCATION START!\r\n");
				
				ret = tag_recv_slot_allocation(&slot);
				if(ret == RET_SUCCESS)
				{
					DBG_PRINT("4-3 AG_SLOT_ALLOCATION SUC：ANCHOR_ID[%3d / %3d] ,slot:%d\r\n", g_locate_net_info.main_anchor_id,g_locate_net_info.sub_anchor_id,slot);
					tag_state = TAG_RANGING;
					tag_slot_suc_cnt++;
					
				}
				else
				{
					g_pos_info.tag_position[0] = 0;
					g_pos_info.tag_position[1] = 0;
					g_pos_info.tag_position[2] = 0; 
					g_pos_info.D0 = 0;
					g_pos_info.D1 = 0;
					g_pos_info.tag_position_valid_flag = POS_INVALID;			
					DBG_ERR_PRINT("4-3 TAG_RECV_SLOT_ALLOCATION FAIL!\r\n");
					tag_state = TAG_WAITE_ACTIVATION;
				}
				
				break;
			}

			case TAG_RANGING:												//4.测距与定位解算
			{ 
				DBG_PRINT("\r\n4-4 TAG_RANGING START!\r\n");
				
				ret = tag_ranging(slot);
				if(ret == RET_SUCCESS)
				{
					tag_range_suc_cnt++;
					DBG_PRINT("4-4 TAG_RANGING SUC! \r\n");
					tag_state = TAG_WAITE_ACTIVATION;
				}
				else
				{
					DBG_ERR_PRINT("4-4 TAG_RANGING FAIL!\r\n");
					tag_state = TAG_WAITE_ACTIVATION;
				}
			
				break;
			}	

			default:
				ret = RET_FAILED;
				break;			
		}	
	}

		return ret;

}


	
/* 测距	*/
void meas_distance(void)
{		
		// dwt_setcallbacks(NULL, &rx_ok_cb);
		switch(g_device_config.device_type)
		{
				case TAG:  //标签
				{		
						mode_tag();
						//tag_test_ranging();
				}	
				break;
						
				case ANCHOR:  //基站
				{								
						mode_anchor();
						//anchor_test_ranging();
				}						
				break;
		}										
}


/**********************************************************************************************************************/
