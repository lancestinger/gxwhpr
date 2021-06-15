/*
 ******************************************************************************
 * @file    Imu.c
 * @author  Sensor Solutions Software Team
 * @brief   This file show the simplest way to read LIS2MDL mag
 *          connected to LSM6DSO I2C master interface (with FIFO support).
 *
 ******************************************************************************
*/
// Includes ------------------------------------------------------------------//

#include "ism330dhcx_reg.h"
#include "iis2mdc_reg.h"
#include "Imu.h"
#include <string.h>
#include <stdio.h>
#include "spi/spi_drv.h"
#include "gpio/gpio_drv.h"
#include "fml/gnss/nmea.h"
#include "drv/uart/uart_drv.h"
#include "comm/project_def.h"
#include "ins_navigation.h"
#include "Coordi_transfer.h"
#include "ins_baseparams.h"


typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;


#define SENSOR_BUS spi4_handle

/* Private macro -------------------------------------------------------------*/
#define TX_BUF_DIM          10
#define OUT_XYZ_SIZE		6

/* Private variables ---------------------------------------------------------*/
//static uint8_t tx_buffer[TX_BUF_DIM];

int First_flag = TRUE;


static stmdev_ctx_t ag_ctx;
static stmdev_lis2_ctx_t mag_ctx;

static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_mG[3];
//static float temperature_degC;

static uint8_t whoamI;

//三项数据分别计时
static U32 Accel_count = 0;
static U32 Gy_count = 0;
static U32 Mag_count = 0;

//三项数据准备完毕标志
static U8 XL_DRDY = 0;
static U8 GY_DRDY = 0;
static U8 MAG_DRDY = 0;

//三项数据raw_data结构体声明
static axis3bit16_t data_raw_angular_rate;
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_magnetic;

//三项数据的数据量
static int Accel_num = 0;
static int Gy_num = 0;
static int Mag_num = 0;

//三项数据存储数组
static imu_rawdata_t Imu_Accel[IMU_DATA_MAX_NUM];
static imu_rawdata_t Imu_Gy[IMU_DATA_MAX_NUM];
static imu_rawdata_t Imu_Mag[IMU_DATA_MAX_NUM];

//融合定位位置与速度入参
static ins_geopos_t Nav_location;//惯导与GPS融合定位，位置信息结构体
static ins_vel_t Nav_veloc;//惯导与GPS融合定位，速度信息结构体

//融合定位位置与速度出参
static ins_geopos_t Out_location;//惯导与GPS融合定位，位置信息结构体
static ins_vel_t Out_veloc;//惯导与GPS融合定位，速度信息结构体


//姿态输出
static ins_atti_t Attid;

//自校准
static Self_calib Imu_self_cali;
static int SELF_CALI_OVER = FALSE;

//----------------------------thread initial-----------------------------------//
static U64 thread_Imu_stk[SIZE_4K / 4];
static const osThreadAttr_t thread_Imu_attr = {
  .stack_mem  = &thread_Imu_stk[0],
  .stack_size = sizeof(thread_Imu_stk),
  .priority = osPriorityNormal,
};

static U64 thread_Imu_data_stk[SIZE_4K / 8];
static const osThreadAttr_t thread_Imu_data_attr = {
  .stack_mem  = &thread_Imu_data_stk[0],
  .stack_size = sizeof(thread_Imu_data_stk),
  .priority = osPriorityNormal,
};

//-----------------------------------------------------------------------------//

static F32 XL_bias[3] = {0,0,0};//-5.7   -10.5   997.0
static F32 GY_bias[3] = {0,0,0};//304  -712 -304

//-----------------------------------------------------------------------------//

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

static int32_t Ism330dhcx_write_Iis2mdc_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len);

static int32_t Ism330dhcx_read_Iis2mdc_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len);

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write_hub_fifo(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read_hub_fifo(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

/* Main Example --------------------------------------------------------------*/

/*
 * Main Example
 *
 * Configure low level function to access to external device
 * Check if LIS2MDL connected to Sensor Hub
 * Configure lis2mdl in CONTINUOUS_MODE with 20 ODR
 * Configure Sensor Hub to read one slave with XL trigger
 * Set FIFO watermark
 * Set FIFO mode to Stream mode
 * Enable FIFO batching of Slave0 + ACC + Gyro samples
 * Poll for FIFO watermark interrupt and read samples
 */

/*****************************************************************************
 函 数 名  : _Imu_Thread
 功能描述  : _Imu_Thread线程
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年12月24日
    作    者   : zxf
    修改内容   : 新生成函数

*****************************************************************************/

static void _Imu_thread(void * arg)
{
	uint32_t IMU_Time = 0;
	uint32_t locat_Time = 0;
	uint32_t Fusion_Time = 0;
	uint32_t Time_old = 0;
	uint32_t Time_out = 0;
	U32 Old_1PPS = 0;
	U32 Delta_1PPS = 0;
	F32 Heading = 0;
	uint8_t  Locat_valid = 0;
	uint8_t  Angle_valid = 0;
	uint8_t  Veloc_valid = 0;
	static U8 flag = FALSE;
	//U8 Time_out_flag = 0;
/*
	while(sysup_seconds_g<30)
	{
		DBG_IMU_Print("IMU NOW Self Calibriation!!\r\n");
		delay_ms(1000);
	}
*/
	SELF_CALI_OVER = TRUE;//自校准结束标志
	
	//设置加速度计和陀螺仪的三轴零偏
	ins_set_ag_bias(XL_bias,GY_bias);
	
	//DBG_IMU_Print("\r\n加速度计三轴零偏:%6.4f,%6.4f,%6.4f\r\n",XL_bias[0],XL_bias[1],XL_bias[2]);
	//DBG_IMU_Print("\r\n陀螺仪三轴零偏:%6.4f,%6.4f,%6.4f\r\n",GY_bias[0],GY_bias[1],GY_bias[2]);
	
	while(1)
	{
		Time_old = osKernelGetTickCount();
		
		while(NMEA_Data_ptr.GGA_DATA_READY_IMU !=TRUE || NMEA_Data_ptr.RMC_DATA_READY_IMU !=TRUE)
		{
			//waiting for flag of NMEA parse Over//
			delay_ms(20);
			Time_out = osKernelGetTickCount();
			if((Time_out-Time_old) >20000)
			{
				Time_out = 0;
				DBG_IMU_Print("IMU waiting for NMEA Time out!!\r\n");
				break;
			}
		}
		NMEA_Data_ptr.GGA_DATA_READY_IMU = FALSE;
		NMEA_Data_ptr.RMC_DATA_READY_IMU = FALSE;
		
		IN_OUT_Init();//每次惯导计算清空输入输出结构体
		
		//DBG_IMU_Print("\r\n加速度计三轴零偏:%6.4f,%6.4f,%6.4f\r\n",XL_bias[0],XL_bias[1],XL_bias[2]);
		//DBG_IMU_Print("\r\n陀螺仪三轴零偏:%6.4f,%6.4f,%6.4f\r\n",GY_bias[0],GY_bias[1],GY_bias[2]);

		Delta_1PPS = UBX_1PPS_time - Old_1PPS;
		
		Old_1PPS = UBX_1PPS_time;

		//判断位置信息是否有效
		if(NMEA_Data_ptr.RTK_mode < 1 || Delta_1PPS <=0 || NMEA_OPEN_SWITCH == 0)
		{
			Locat_valid = FALSE;//位置无效
			First_flag = TRUE;//方差误差等待第一帧有效
			DBG_IMU_Print("位置无效!!");
		}else{
			Locat_valid = TRUE;//位置有效
		}

		if(VELOC_VALID == FALSE || NMEA_OPEN_SWITCH == 0)
		{
			Veloc_valid = FALSE;
			First_flag = TRUE;//方差误差等待第一帧有效
			DBG_IMU_Print("速度无效!!");
		}else{
			Veloc_valid = TRUE;
		}
		
		//判断角度是否有效
		if(FIRST_ANGLE_GET == FALSE || NMEA_OPEN_SWITCH == 0)
		{
			Heading = 0;
			Angle_valid = FALSE;//角度无效
			DBG_IMU_Print("角度无效!!");
		}else{
			Heading = NMEA_Data_ptr.angle;
			Angle_valid = TRUE;//角度有效
		}
		
		if(Locat_valid == TRUE && Veloc_valid == TRUE)
		{
			////获取当前定时器计数
			locat_Time = osKernelGetTickCount();
			
			//坐标倒推计算与坐标系转换
			if(Lat_Lon_ext(locat_Time,&Nav_location,&Nav_veloc)!=0)
			{
				DBG_IMU_Print("Lat_Lon_ext ERROR!!\r\n");
				Locat_valid = FALSE;//位置无效
			}

			
			if(Std_Manage(&Nav_location,&Nav_veloc)!=0)
			{
				DBG_IMU_Print("STD CALC ERROR!!\r\n");
				Locat_valid = FALSE;//位置无效
			}
			
			Nav_location.std_h = 1;
			Nav_veloc.std_vu = 1;
			
			First_flag = FALSE;
		}
				
		DBG_IMU_Print("\r\nINPUT=%d,%d,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%d,%4.1f,\r\n",\
			locat_Time,\
			Locat_valid,\
			Nav_location.lat,Nav_location.lng,Nav_location.h,\
			Nav_location.std_e,Nav_location.std_n,Nav_location.std_h,\
			Veloc_valid,\
			Nav_veloc.ve,Nav_veloc.vn,Nav_veloc.vu,\
			Nav_veloc.std_ve,Nav_veloc.std_vn,Nav_veloc.std_vu,\
			Angle_valid,\
			Heading);

		////获取IMU调用时刻定时器计数
		IMU_Time = osKernelGetTickCount();
		
		DBG_IMU_Print("\r\nIMU_Update_time = %d\r\n",IMU_Time);
		
		////惯导计算
		insNaviUpdate(IMU_Time, Imu_Accel, Accel_num, Imu_Mag, Mag_num, Imu_Gy, Gy_num);
		
		//根据数据有效性判断是否调用算法
		if(Locat_valid == TRUE && Angle_valid == TRUE && Angle_valid == TRUE)
		{
			////获取融合算法调用时刻定时器计数	
			Fusion_Time = osKernelGetTickCount();

			///融合定位计算
			insNaviFusion(Fusion_Time,&Nav_location,&Nav_veloc,&Heading);
			
		}else if(Angle_valid == TRUE){
		
			////获取融合算法调用时刻定时器计数	
			Fusion_Time = osKernelGetTickCount();

			///融合定位计算
			insNaviFusion(Fusion_Time,NULL,NULL,&Heading);

		}else{
		
			////获取融合算法调用时刻定时器计数	
			Fusion_Time = osKernelGetTickCount();

			///融合定位计算
			insNaviFusion(Fusion_Time,NULL,NULL,NULL);
		}
		
		////输出定位结果
		ins_getFusionRslt(&Out_location,&Out_veloc,&Attid);
		
		DBG_IMU_Print("\r\nFusion_output = %f,%f,%f,%f,%f,%f\r\n",Out_location.lat,Out_location.lng,Out_location.h,Out_veloc.ve,Out_veloc.vn,Out_veloc.vu);

		DBG_IMU_Print("\r\nAttid = %f,%f,%f,%f,%f,%f\r\n\r\n",Attid.p,Attid.r,Attid.y,Attid.std_p,Attid.std_r,Attid.std_y);
		
		Delta_1PPS = 0;
		IMU_Time = 0;
		locat_Time = 0;
		Fusion_Time = 0;
		delay_ms(5);
		if(!flag){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET);
			flag = TRUE;
		}else{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET);
			flag = FALSE;
		}
	}
}

/*****************************************************************************
 函 数 名  : _Imu_data_Thread
 功能描述  : _Imu_data_Thread线程
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年1月11日
    作    者   : zxf
    修改内容   : 新生成函数

*****************************************************************************/


static void _Imu_data_thread(void * arg)
{
	//static int miss_flag=0;
	while(1)
	{
		if(XL_DRDY)
		{
			Imu_Acce_data_get();
			XL_DRDY = FALSE;
		}
		
		if(GY_DRDY)
		{
			Imu_Gy_data_get();
			GY_DRDY = FALSE;
		}

		if(MAG_DRDY)
		{
			Imu_Mag_data_get();
			MAG_DRDY = FALSE;
		}

		delay_ms(5);
	}
}


void IN_OUT_Init(void)
{

	GLOBAL_MEMSET(&Nav_location,0x0,sizeof(ins_geopos_t));
	GLOBAL_MEMSET(&Nav_veloc,0x0,sizeof(ins_vel_t));
	GLOBAL_MEMSET(&Out_location,0x0,sizeof(ins_geopos_t));
	GLOBAL_MEMSET(&Out_veloc,0x0,sizeof(ins_vel_t));


}

void Self_cali_Init(void)
{
	GLOBAL_MEMSET(&Imu_self_cali,0x0,sizeof(Self_calib));
	GLOBAL_MEMSET(&XL_bias,0x0,sizeof(float)*3);
	GLOBAL_MEMSET(&GY_bias,0x0,sizeof(float)*3);

	XL_bias[0] = ( -5.7/1000)*NORMAL_G;
	XL_bias[1] = (-10.5/1000)*NORMAL_G;
	XL_bias[2] = (997/1000)*NORMAL_G;
	GY_bias[0] = 304*DEG2RAD/1000;
	GY_bias[1] = -712*DEG2RAD/1000;
	GY_bias[2] = -304*DEG2RAD/1000;
}


/*****************************************************************************
 函 数 名  : Imu_apl_init
 功能描述  : Imu_apl初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年12月24日
    作    者   : zxf
    修改内容   : 新生成函数

*****************************************************************************/

void Imu_apl_init(void)
{
	osThreadId_t thread_Imu_id = 0;
	osThreadId_t thread_Imu_data_id = 0;

	uint8_t rst=1;
	uint8_t	time_out=0;
	int32_t ret = 0;
	ism330dhcx_sh_cfg_read_t sh_cfg_read;
	ism330dhcx_pin_int1_route_t *ISM330DHCX_pin_int1 = NULL;
	ism330dhcx_pin_int2_route_t *ISM330DHCX_pin_int2 = NULL;
	
	ISM330DHCX_pin_int1 = (ism330dhcx_pin_int1_route_t*)GLOBAL_MALLOC(sizeof(ism330dhcx_pin_int1_route_t));
	ISM330DHCX_pin_int2 = (ism330dhcx_pin_int2_route_t*)GLOBAL_MALLOC(sizeof(ism330dhcx_pin_int2_route_t));
	GLOBAL_MEMSET(ISM330DHCX_pin_int1,PROPERTY_DISABLE,sizeof(ism330dhcx_pin_int1_route_t));
	GLOBAL_MEMSET(ISM330DHCX_pin_int2,PROPERTY_DISABLE,sizeof(ism330dhcx_pin_int2_route_t));

//------------Initialize LSM330DHCX & IIS2MDC driver interface--------------//

	ag_ctx.write_reg = platform_write_hub_fifo;
	ag_ctx.read_reg = platform_read_hub_fifo;
	ag_ctx.handle = &spi4_handle;

	//Initialize lis2mdc driver interface
	mag_ctx.Lis2_write_reg = Ism330dhcx_write_Iis2mdc_cx;
	mag_ctx.Lis2_read_reg = Ism330dhcx_read_Iis2mdc_cx;
	mag_ctx.handle = &spi4_handle;

//---------------------------Struct Initial---------------------------------//

	IN_OUT_Init();//输入输出结构体初始化
	Self_cali_Init();//自校准初始化
	Coordi_initial();//坐标转换初始化

	NMEA_OPEN_SWITCH = 1;//开启惯导部分的GPS位置有效开关

//-------------------LSM330DHCX & IIS2MDC DRDY Initial--------------------//

	//关闭加速度计在INT1上的DRDY中断
	ret = ism330dhcx_pin_int1_route_get(&ag_ctx, ISM330DHCX_pin_int1);
	ISM330DHCX_pin_int1->int1_ctrl.int1_drdy_xl = PROPERTY_DISABLE;	
	ret = ism330dhcx_pin_int1_route_set(&ag_ctx, ISM330DHCX_pin_int1);

	//关闭陀螺仪在INT2上的DRDY中断
	ret = ism330dhcx_pin_int2_route_get(&ag_ctx, ISM330DHCX_pin_int2);
	ISM330DHCX_pin_int2->int2_ctrl.int2_drdy_g = PROPERTY_DISABLE;
	ret = ism330dhcx_pin_int2_route_set(&ag_ctx, ISM330DHCX_pin_int2);

//--------------------------Check LSM330DHCX ID---------------------------//

#if 1
	whoamI = 0; 

	ism330dhcx_device_id_get(&ag_ctx, &whoamI);
	while(whoamI != ISM330DHCX_ID)
	{
		time_out++;
		if(time_out>=100)
		{
			ERR_PRINT(("ISM330DHCX ID获取失败!!\r\n"));
			ERR_PRINT(("ISM330DHCX ERROR ID = %x\r\n",whoamI));
			time_out=0;
			break;
		}
		ism330dhcx_device_id_get(&ag_ctx, &whoamI);
		delay_ms(50);
	}
	NOTE_PRINT(("ISM330DHCX ID = %x 获取成功!!!\r\n",whoamI));
#endif

//-------------------------LSM330DHCX RESET-------------------------------//

	while(ism330dhcx_reset_set(&ag_ctx, PROPERTY_ENABLE))
	{
		GLOBAL_PRINT(("ism330dhcx is reseting!!\r\n"));
		delay_ms(50);
	}

	do {
		ism330dhcx_reset_get(&ag_ctx, &rst);
	} while (rst);

	NOTE_PRINT(("ism330dhcx 设备初始化完成!!\r\n"));

//--------------------------Check IIS2MDC ID---------------------------//

	whoamI = 0;

	/* Check if LIS2MDC connected to Sensor Hub. */
	while(whoamI != IIS2MDC_ID)//whoamI != IIS2MDC_ID
	{
		time_out++;
		if(time_out>=100)
		{
			ERR_PRINT(("iis2mdc ID获取失败!!\r\n"));
			ERR_PRINT(("iis2mdc ERROR ID = %d\r\n",whoamI));
			time_out=0;
			break;
		}
		ret = iis2mdc_device_id_get(&mag_ctx, &whoamI);
		delay_ms(50);
	}
	NOTE_PRINT(("iis2mdc ID = %x 获取成功!!!\r\n",whoamI));
	
//-------------------------IIS2MDC RESET-------------------------------//

	ret = iis2mdc_reset_set(&mag_ctx, PROPERTY_ENABLE);
	delay_ms(20);
	do{
		ret = iis2mdc_reset_get(&mag_ctx, &rst);
		delay_ms(20);
		if(rst)
		{
			GLOBAL_PRINT(("iis2mdc is Reseting!\r\n"));
		}
	}while(rst);
	NOTE_PRINT(("iis2mdc 设备初始化完成!!\r\n"));
	
//----------------------------IIS2MDC Configure---------------------------------//

	ret = iis2mdc_block_data_update_set(&mag_ctx, PROPERTY_ENABLE);//BDU OPEN
	ret = iis2mdc_data_rate_set(&mag_ctx, IIS2MDC_ODR_50Hz);//IIS2MDC_ODR_20Hz
	ret = iis2mdc_set_rst_mode_set(&mag_ctx, IIS2MDC_SENS_OFF_CANC_EVERY_ODR);//IIS2MDC_SENS_OFF_CANC_EVERY_ODR
	ret = iis2mdc_offset_temp_comp_set(&mag_ctx, PROPERTY_ENABLE);//启动温度补偿
	ret = iis2mdc_drdy_on_pin_set(&mag_ctx, PROPERTY_ENABLE);//DRDY ON
	ret = iis2mdc_operating_mode_set(&mag_ctx, IIS2MDC_CONTINUOUS_MODE);//IIS2MDC_CONTINUOUS_MODE

//----------------------LSM330DHCX Configure I2C Master--------------------//

#if 1  
	sh_cfg_read.slv_add = (IIS2MDC_I2C_ADD & 0xFEU) >> 1; 
	sh_cfg_read.slv_subadd = IIS2MDC_OUTX_L_REG;
	sh_cfg_read.slv_len = 3 * sizeof(int16_t);
	ret = ism330dhcx_sh_slv0_cfg_read(&ag_ctx, &sh_cfg_read);
	// Configure Sensor Hub to read one slave.
	ret = ism330dhcx_sh_slave_connected_set(&ag_ctx, ISM330DHCX_SLV_0);
	// Enable I2C Master.
	ret = ism330dhcx_sh_master_set(&ag_ctx, PROPERTY_ENABLE);
#endif

//------------------------LSM330DHCX & IIS2MDC DRDY Configure-------------------//
	//开启加速度计在INT1上的DRDY中断
	ret = ism330dhcx_pin_int1_route_get(&ag_ctx, ISM330DHCX_pin_int1);
	ISM330DHCX_pin_int1->int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;	
	ret = ism330dhcx_pin_int1_route_set(&ag_ctx, ISM330DHCX_pin_int1);

	//开启陀螺仪在INT2上的DRDY中断
	ret = ism330dhcx_pin_int2_route_get(&ag_ctx, ISM330DHCX_pin_int2);
	ISM330DHCX_pin_int2->int2_ctrl.int2_drdy_g = PROPERTY_ENABLE;
	ret = ism330dhcx_pin_int2_route_set(&ag_ctx, ISM330DHCX_pin_int2);

//--------------------------------------LSM330DHCX XL&GY Configure---------------------------------------------------//

	ret = ism330dhcx_block_data_update_set(&ag_ctx, PROPERTY_ENABLE);//PROPERTY_ENABLE
	ret = ism330dhcx_data_ready_mode_set(&ag_ctx, ISM330DHCX_DRDY_PULSED);//ISM330DHCX_DRDY_PULSED
	ret = ism330dhcx_xl_full_scale_set(&ag_ctx, ISM330DHCX_2g);
	ret = ism330dhcx_gy_full_scale_set(&ag_ctx, ISM330DHCX_500dps);//ISM330DHCX_2000dps
	ret = ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_52Hz);//ISM330DHCX_XL_ODR_12Hz5//ISM330DHCX_XL_ODR_26Hz
	ret = ism330dhcx_gy_data_rate_set(&ag_ctx, ISM330DHCX_GY_ODR_52Hz);//ISM330DHCX_GY_ODR_12Hz5//ISM330DHCX_GY_ODR_26Hz

//--------------------------------------FIFO ENABLE/DISABLE----------------------------------------------------------//
#if 0 //FIFO ENABLE/DISABLE

	/*
	* Configure LSM6DSO FIFO.
	*
	*
	* Set FIFO watermark (number of unread sensor data TAG + 6 bytes
	* stored in FIFO) to 15 samples. 5 * (Acc + Gyro + Mag)
	*/
	ret = ism330dhcx_fifo_watermark_set(&ag_ctx, 15);

	/* Set FIFO mode to Stream mode (aka Continuous Mode). */
	ret = ism330dhcx_fifo_mode_set(&ag_ctx, ISM330DHCX_STREAM_MODE);

	/* Enable latched interrupt notification. */
	ret = ism330dhcx_int_notification_set(&ag_ctx, ISM330DHCX_ALL_INT_LATCHED);//ISM330DHCX_ALL_INT_LATCHED

	/* Enable drdy 75 us pulse: uncomment if interrupt must be pulsed. */
	//ret = ism330dhcx_data_ready_mode_set(&ag_ctx, ISM330DHCX_DRDY_LATCHED);

	/*
	* FIFO watermark interrupt routed on INT1 pin.
	* Remember that INT1 pin is used by sensor to switch in I3C mode.
	*/
	//ism330dhcx_pin_int1_route_get(&ag_ctx, &int1_route);
	//int1_route.int1_ctrl.int1_fifo_th = PROPERTY_ENABLE;
	//ism330dhcx_pin_int1_route_set(&ag_ctx, &int1_route);

	/*
	* Enable FIFO batching of Slave0.
	* ODR batching is 13 Hz.
	*/

	ret = ism330dhcx_sh_batch_slave_0_set(&ag_ctx, PROPERTY_ENABLE);
	ret = ism330dhcx_sh_data_rate_set(&ag_ctx, ISM330DHCX_SH_ODR_104Hz);//ISM330DHCX_SH_ODR_13Hz

	/* Set FIFO batch XL/Gyro ODR to 12.5Hz. */
	ret = ism330dhcx_fifo_xl_batch_set(&ag_ctx, ISM330DHCX_XL_BATCHED_AT_104Hz);//ISM330DHCX_XL_BATCHED_AT_12Hz5
	ret = ism330dhcx_fifo_gy_batch_set(&ag_ctx, ISM330DHCX_GY_BATCHED_AT_104Hz); 
	ret = ism330dhcx_gy_lp1_bandwidth_set(&ag_ctx,ISM330DHCX_AGGRESSIVE);
#endif
//-----------------------------------------------------------------------------------------------------------------//

	free(ISM330DHCX_pin_int1);
	free(ISM330DHCX_pin_int2);
	NOTE_PRINT(("ISM330DHCX & IIS2MDC Initial OK!!\r\n"));

	thread_Imu_id = osThreadNew(_Imu_thread, NULL, &thread_Imu_attr);
	GLOBAL_HEX(thread_Imu_id);
	
	thread_Imu_data_id = osThreadNew(_Imu_data_thread, NULL, &thread_Imu_data_attr);
	GLOBAL_HEX(thread_Imu_data_id);
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_hub_fifo(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
	int32_t ret=0;
	if (handle == &spi4_handle){
		gpio_drv_set(GPIO_ISM330_SS, LEVEL_LOW);
		ret = HAL_SPI_Transmit(handle, &Reg, 1, 3000);
		if(ret!=0)
		{
			ERR_PRINT(("Imu SendTransmit ERROR!!\r\n"));
			ERR_PRINT(("Register is: %d\r\n",Reg));
		}
		delay_ms(2);
		ret = HAL_SPI_Transmit(handle, Bufp, len, 3000);
		
		if(ret!=0)
		{
			ERR_PRINT(("Imu Send ERROR!!ret =%d\r\n",ret));
			ERR_PRINT(("Register is: %d\r\n",Reg));
			ERR_PRINT(("Bufp is: %d\r\n",Bufp));
		}
		gpio_drv_set(GPIO_ISM330_SS, LEVEL_HIGH);
	}else{
		HAL_I2C_Mem_Write(handle, ISM330DHCX_I2C_ADD_H, Reg,
	                  I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
	}
	return ret;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_hub_fifo(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
	int32_t ret=0;
	
	if (handle == &spi4_handle){
	    /* Read command */
	    Reg |= 0x80;
		gpio_drv_set(GPIO_ISM330_SS, LEVEL_LOW);
	    ret = HAL_SPI_Transmit(handle, &Reg, 1, 3000);
		if(ret!=0)
		{
			ERR_PRINT(("Imu ReadTransmit ERROR!!\r\n"));
			ERR_PRINT(("Register is: %d\r\n",Reg));
		}
		delay_ms(2);
	    ret = HAL_SPI_Receive(handle, Bufp, len, 3000);
		if(ret!=0)
		{
			ERR_PRINT(("Imu READ ERROR!!ret = %d\r\n",ret));
			ERR_PRINT(("Register is: %d\r\n",Reg));
			ERR_PRINT(("Bufp is: %d\r\n",Bufp));
		}
		gpio_drv_set(GPIO_ISM330_SS, LEVEL_HIGH);
  	}
  
  	return ret;
}


/*
 * @brief  Write lsm2mdc device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t Ism330dhcx_write_Iis2mdc_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len)
{
  axis3bit16_t data_raw_acceleration;
  int32_t ret;
  uint8_t drdy;
  ism330dhcx_status_master_t master_status;
  ism330dhcx_sh_cfg_write_t sh_cfg_write;
  
  /* Disable accelerometer. */
  ret = ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_OFF);

  /* Configure Sensor Hub to read LIS2MDL. */
  sh_cfg_write.slv0_add = (IIS2MDC_I2C_ADD & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_write.slv0_subadd = reg,
  sh_cfg_write.slv0_data = *data,
  ret = ism330dhcx_sh_cfg_write(&ag_ctx, &sh_cfg_write);

  /* Enable I2C Master. */
  ret = ism330dhcx_sh_master_set(&ag_ctx, PROPERTY_ENABLE);

  /* Enable accelerometer to trigger Sensor Hub operation. */
  ret = ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_104Hz);

  /* Wait Sensor Hub operation flag set. */
  ret = ism330dhcx_acceleration_raw_get(&ag_ctx, data_raw_acceleration.u8bit);
  do
  {
    delay_ms(20);
	ret = ism330dhcx_xl_flag_data_ready_get(&ag_ctx, &drdy);
  } while(!drdy);

  do
  {
    delay_ms(20);
    ret = ism330dhcx_sh_status_get(&ag_ctx, &master_status);
  } while(!master_status.sens_hub_endop);

  /* Disable I2C master and XL (trigger). */
  ret = ism330dhcx_sh_master_set(&ag_ctx, PROPERTY_DISABLE);
  ret = ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_OFF);

  return ret;
}

/*
 * @brief  Read lsm2mdc device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t Ism330dhcx_read_Iis2mdc_cx(void *ctx, uint8_t reg, uint8_t *data,
        uint16_t len)
{
  ism330dhcx_sh_cfg_read_t sh_cfg_read;
  axis3bit16_t data_raw_acceleration;
  int32_t ret;
  uint8_t drdy;
  //uint8_t i=0;
  ism330dhcx_status_master_t master_status;

  //Test by Zhangxf
  static uint8_t PTR_DATA[144]={0};

  /* Disable accelerometer. */
  ret = ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_OFF);

  /* Configure Sensor Hub to read LIS2MDL. */
  sh_cfg_read.slv_add = (IIS2MDC_I2C_ADD & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_read.slv_subadd = reg;
  sh_cfg_read.slv_len = len;
  ret = ism330dhcx_sh_slv0_cfg_read(&ag_ctx, &sh_cfg_read);//Configure slave 0 for perform a write/read
  ret = ism330dhcx_sh_slave_connected_set(&ag_ctx, ISM330DHCX_SLV_0);//Number of external sensors to be read by the sensor hub
  
  /* Enable I2C Master and I2C master. */
  ret = ism330dhcx_sh_master_set(&ag_ctx, PROPERTY_ENABLE);

  /* Enable accelerometer to trigger Sensor Hub operation. */
  ret = ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_104Hz);//
  /* Wait Sensor Hub operation flag set. */
  ret = ism330dhcx_acceleration_raw_get(&ag_ctx, data_raw_acceleration.u8bit);

  do {
    delay_ms(20);
	ret = ism330dhcx_xl_flag_data_ready_get(&ag_ctx, &drdy);
  } while (!drdy);

  do {
    delay_ms(20);
    ret = ism330dhcx_sh_status_get(&ag_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL(trigger). */
  ret = ism330dhcx_sh_master_set(&ag_ctx, PROPERTY_DISABLE);
  ret = ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_OFF);
  
  //memset(&TEST_DATA,0x0,sizeof(ism330dhcx_emb_sh_read_t));

  /* Read SensorHub registers */
  ret = ism330dhcx_sh_read_data_raw_get(&ag_ctx,(ism330dhcx_emb_sh_read_t*)&PTR_DATA);

  GLOBAL_MEMCPY((uint8_t *)data,(uint8_t *)&PTR_DATA,len);
/*
  printf("After Data/bin = ");
  for(i=0;i<144;i++)
  {
  	  if(i%8 ==0 )
	  printf("\r\n%04x:",i);
	  printf("%02x",PTR_DATA[i]);
  }
  //printf("DATA = %d",data);
*/
  return ret;
}


void Imu_Mag_data_get(void)
{
	static uint8_t k=0;
	uint8_t i=0;
	static uint8_t emb_sh[18]={0};

	GLOBAL_MEMSET(emb_sh, 0x0, 18);
	if(k>IMU_DATA_MAX_NUM-1)
	{ 
	  for(i=0;i<k-1;i++)
	  {
		  Imu_Mag[i].x = Imu_Mag[i+1].x;
		  Imu_Mag[i].y = Imu_Mag[i+1].y;
		  Imu_Mag[i].z = Imu_Mag[i+1].z;
		  Imu_Mag[i].t = Imu_Mag[i+1].t;
	  }   
	  k--;
	}
	
	Imu_Mag[k].t = Mag_count;

	ism330dhcx_sh_read_data_raw_get(&ag_ctx, (ism330dhcx_emb_sh_read_t*)&emb_sh);
	GLOBAL_MEMCPY((uint8_t *)&data_raw_magnetic,(uint8_t *)&emb_sh[0],OUT_XYZ_SIZE);
	magnetic_mG[0] = IIS2MDC_FROM_LSB_TO_mG(data_raw_magnetic.i16bit[0]);
	magnetic_mG[1] = IIS2MDC_FROM_LSB_TO_mG(data_raw_magnetic.i16bit[1]);
	magnetic_mG[2] = IIS2MDC_FROM_LSB_TO_mG(data_raw_magnetic.i16bit[2]);
	DBG_IMU_RAW_Print("Mag[mG]:%d,%4.2f,%4.2f,%4.2f\r\n",Mag_count,magnetic_mG[0],magnetic_mG[1],magnetic_mG[2]);

	Imu_Mag[k].x = magnetic_mG[0]*0.1;//单位转换为 μT
	Imu_Mag[k].y = magnetic_mG[1]*0.1;
	Imu_Mag[k].z = magnetic_mG[2]*0.1;
	k++;
	Mag_num = k;
}


void Imu_Gy_data_get(void)
{
	static uint8_t k=0;
	uint8_t i=0;
	
	if(k>IMU_DATA_MAX_NUM-1)
	{ 
	  for(i=0;i<k-1;i++)
	  {
		  Imu_Gy[i].x = Imu_Gy[i+1].x;
		  Imu_Gy[i].y = Imu_Gy[i+1].y;
		  Imu_Gy[i].z = Imu_Gy[i+1].z;
		  Imu_Gy[i].t = Imu_Gy[i+1].t;
	  }   
	  k--;
	}
	
	Imu_Gy[k].t = Gy_count;

	GLOBAL_MEMSET(data_raw_angular_rate.u8bit, 0x0, 3 * sizeof(int16_t));
	ism330dhcx_angular_rate_raw_get(&ag_ctx, data_raw_angular_rate.u8bit);
	angular_rate_mdps[0] = ism330dhcx_from_fs500dps_to_mdps(data_raw_angular_rate.i16bit[0]);
	angular_rate_mdps[1] = ism330dhcx_from_fs500dps_to_mdps(data_raw_angular_rate.i16bit[1]);
	angular_rate_mdps[2] = ism330dhcx_from_fs500dps_to_mdps(data_raw_angular_rate.i16bit[2]);
	
	DBG_IMU_RAW_Print("GY[mdps]:%d,%4.2f,%4.2f,%4.2f\r\n",Gy_count,angular_rate_mdps[0],angular_rate_mdps[1],angular_rate_mdps[2]);

	Imu_Gy[k].x = angular_rate_mdps[0]*DEG2RAD/1000;//单位转换为 rad/s
	Imu_Gy[k].y = angular_rate_mdps[1]*DEG2RAD/1000;
	Imu_Gy[k].z = angular_rate_mdps[2]*DEG2RAD/1000;

	//For self init//
	/*
	if(sysup_seconds_g<30 && SELF_CALI_OVER == FALSE)
	{
		Imu_self_cali.SUM_Gy_x += Imu_Gy[k].x;
		Imu_self_cali.SUM_Gy_y += Imu_Gy[k].y;
		Imu_self_cali.SUM_Gy_z += Imu_Gy[k].z;
		Imu_self_cali.Num_Gy++;
		GY_bias[0] = Imu_self_cali.SUM_Gy_x/Imu_self_cali.Num_Gy;
		GY_bias[1] = Imu_self_cali.SUM_Gy_y/Imu_self_cali.Num_Gy;
		GY_bias[2] = Imu_self_cali.SUM_Gy_z/Imu_self_cali.Num_Gy;
	}
	*/
	k++;
	Gy_num = k;	
}



void Imu_Acce_data_get(void)
{
	static uint8_t k=0;
	uint8_t i=0;
	
	if(k>IMU_DATA_MAX_NUM-1)
	{ 
	  for(i=0;i<k-1;i++)
	  {
		  Imu_Accel[i].x = Imu_Accel[i+1].x;
		  Imu_Accel[i].y = Imu_Accel[i+1].y;
		  Imu_Accel[i].z = Imu_Accel[i+1].z;
		  Imu_Accel[i].t = Imu_Accel[i+1].t;
	  }   
	  k--;
	}
	
	Imu_Accel[k].t = Accel_count;

	GLOBAL_MEMSET(data_raw_acceleration.u8bit, 0x0, 3 * sizeof(int16_t));
	ism330dhcx_acceleration_raw_get(&ag_ctx, data_raw_acceleration.u8bit);
	acceleration_mg[0] = ism330dhcx_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
	acceleration_mg[1] = ism330dhcx_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
	acceleration_mg[2] = ism330dhcx_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);
	
	DBG_IMU_RAW_Print("Accel[mg]:%d,%4.2f,%4.2f,%4.2f\r\n",Accel_count,acceleration_mg[0],acceleration_mg[1],acceleration_mg[2]);

	Imu_Accel[k].x = (acceleration_mg[0]/1000)*NORMAL_G;//单位转换为 m/s^2
	Imu_Accel[k].y = (acceleration_mg[1]/1000)*NORMAL_G;
	Imu_Accel[k].z = (acceleration_mg[2]/1000)*NORMAL_G;

	//For self init//
	/*
	if(sysup_seconds_g<30 && SELF_CALI_OVER == FALSE)
	{
		Imu_self_cali.SUM_Accel_x += Imu_Accel[k].x;
		Imu_self_cali.SUM_Accel_y += Imu_Accel[k].y;
		Imu_self_cali.SUM_Accel_z += Imu_Accel[k].z;
		Imu_self_cali.Num_Accel++;
		XL_bias[0] = Imu_self_cali.SUM_Accel_x/Imu_self_cali.Num_Accel;
		XL_bias[1] = Imu_self_cali.SUM_Accel_y/Imu_self_cali.Num_Accel;
		XL_bias[2] = Imu_self_cali.SUM_Accel_z/Imu_self_cali.Num_Accel;
	}
	*/
	k++;
	Accel_num = k;
}

void Imu_Time_Get(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_12)
	{
		Gy_count = osKernelGetTickCount();
		GY_DRDY = TRUE;
	}
	else if(GPIO_Pin == GPIO_PIN_13)
	{
		Accel_count = osKernelGetTickCount();
		XL_DRDY = TRUE;
	}
	else if(GPIO_Pin == GPIO_PIN_14)
	{
		Mag_count = osKernelGetTickCount();
		MAG_DRDY = TRUE;
	}
}

