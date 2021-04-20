#include "EH_uwb.h"
#include "EH_uwb_parse.h"
#include "uart/uart_fml.h"
#include "project_def.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "Matrix_functions.h"
#include "kalman/kalman_filter.h"

//extern osMessageQueueId_t uwbQueueHandle;
uint8_t UWB_SWICH = 0;
uint8_t Least_fail = 0;

static U64 thread_EH_uwb_stk[SIZE_4K/8];
static const osThreadAttr_t thread_EH_uwb_attr = {
  .stack_mem  = &thread_EH_uwb_stk[0],
  .stack_size = sizeof(thread_EH_uwb_stk),
  .priority = osPriorityNormal,
};

/*------------------------------文件变量------------------------------*/

apdist aplist[]={

#if 0
    //zfg
	0x1145,{-11.5018,-7.6924,3.24},//{-9.2225,-7.5942,3.24}
	0x118c,{-9.2716,-34.9638,3.54},//{-6.377,-38.1664,3.54}
	0x1955,{14.7298,-23.8052,3.44},//{18.7103,-25.6092,3.44}
	0xf4f9,{0,0,3.72},//{0,0,3.72}
	0xce92,{-3.0846,-32.8713,9.15},//{-4.3683,-33.3934,9.15}
	0x0acb,{-9.097, -6.3824,8.957},//{-12.1930, -4.2554,8.957}
	0xcdc5,{3.0932, 1.10476,9.129},//{-0.3068, 3.0868,9.129}
	0xcc01,{16.946, -22.5812,9.013},//{18.0609, -21.6199,9.013}

#else
    //phone point
	0x1145,{-9.2225,-7.5942,3.24},//
	0x118c,{-6.377,-38.1664,3.54},//
	0x1955,{18.7103,-25.6092,3.44},//
	0xf4f9,{0,0,3.72},//
	0xce92,{-2.4501,-36.4987,9.15},//{-4.3683,-33.3934,9.15}
	0x0acb,{-10.2748,-7.3607,8.957},//{-12.1930, -4.2554,8.957}
	0xcdc5,{1.6113,-0.0184,9.129},//{-0.3068, 3.0868,9.129}
	0xcc01,{19.9791,-24.7252,9.013},//{18.0609, -21.6199,9.013}

#endif

//0x1138,{5,4,3},
   //0x1137,{4,3,5},
   //0x1136,{3,4,5},
   //0x1135,{0,0,0},

};
	char teststring[]={0x02,0x91,0x11,0x02,
								0x01,0x30,
								0x00,0x00,
								0x00,0x00,0x00,0x00,//帧长度
								0x09,0x09,0x09,0x09,//卡号
								0x09,0x09,0x09,//序号
								0x04,//基站数目
							0x35,0x11,//地址
								0xf4,0x01,//距离
								0x35,0x35,//RSSI AND REG
							0x36,0x11,//地址
								0xf4,0x01,//距离
								0x35,0x35,//RSSI AND REG
							0x37,0x11,//地址
								0x08,0x02,//距离
								0x35,0x35,//RSSI AND REG								
							0x38,0x11,//地址
								0x69,0x01,//距离
								0x35,0x35,//RSSI AND REG								
								0x01};//目标坐标3,4,0
/*------------------------------文件变量------------------------------*/
								
int tag_xyz(float *left_parameter,float *right_parameter, float *result,const int dimension, const int num)
{
	float *matrix_a = (float *)GLOBAL_MALLOC(sizeof(float)*(num-1)*dimension);
	float *matrix_b = (float *)GLOBAL_MALLOC(sizeof(float)*(num-1));
	float matrix_f[DNUM] = {0};
	float matrix_g[DNUM] = {0};
	for(int i = 0;i < num ; i++){

		swap(&left_parameter[0],&left_parameter[i*dimension]);
		swap(&left_parameter[1],&left_parameter[i*dimension+1]);
#if DNUM == 3
		swap(&left_parameter[2],&left_parameter[i*dimension+2]);
#endif
		swap(&right_parameter[0],&right_parameter[i]);
		
		arguments_transformer1(left_parameter,right_parameter,matrix_a,matrix_b,dimension,num);//换算成基本线性方程组形态。
		
		least_square(matrix_a,matrix_b,matrix_f,dimension,num-1);//最小二乘法

		matrix_g[0] += matrix_f[0];
		matrix_g[1] += matrix_f[1];
#if DNUM == 3
		matrix_g[2] += matrix_f[2];
#endif
	}
		result[0] += matrix_g[0]/num;
		result[1] += matrix_g[1]/num;
#if DNUM == 3
		result[2] += matrix_g[2]/num;
#endif
	GLOBAL_FREE(matrix_a);
	GLOBAL_FREE(matrix_b);
	return 0;
}
								
								
int least_square(float *matrix_a,float *matrix_b, float *result,const int dimension, const int num)
{
	float *matrix_d = (float *)GLOBAL_MALLOC(sizeof(float)*(num)*dimension);
	float *matrix_c = (float *)GLOBAL_MALLOC(sizeof(float)*(num));	
	float matrix_e[DNUM][DNUM] = {0};
	float matrix_g[DNUM][DNUM] = {0};
	//x=(JTrJr)-1JTrb 
	
	Multi_transpose(matrix_a,matrix_d,num,dimension);//置换JTr
	Multi(matrix_d,matrix_a,matrix_e[0],dimension,num,num,dimension);//乘法	//JTrJr
	//printf("Matrix_E: %6.3f \r\n",matrix_e);
	Matrix_inversion(matrix_e,dimension,matrix_g); //取逆 (JTrJr)-1
	Multi(matrix_d,matrix_b,matrix_c,dimension,num,num,1);//JTrb 
	Multi(matrix_g[0],matrix_c,result,dimension,dimension,dimension,1);//(JTrJr)-1JTrb
	
	GLOBAL_FREE(matrix_d);
	GLOBAL_FREE(matrix_c);
	return 0;
}

//解析基站坐标
int aplistparse(UWBagree uwbagree,Vector3 *ap,float *matrix_x,float *matrix_y)
{
	//double point[3] = {2,9,1};//test
	//int k=0;
	
		for(int i = 0; i < uwbagree.data.num; i++){
			for(int j = 0; j < sizeof(aplist)/sizeof(aplist[0]); j++){
				if(uwbagree.data.info[i].addr == aplist[j].addr){
					/*
					ap[i].x = aplist[j].zxy.x;
					ap[i].y = aplist[j].zxy.y;
					ap[i].z = aplist[j].zxy.z;
					*/
					if(DNUM == 3){
						matrix_x[i*DNUM+0] = aplist[j].zxy.x;
						matrix_x[i*DNUM+1] = aplist[j].zxy.y;
						matrix_x[i*DNUM+2] = aplist[j].zxy.z;
						matrix_y[i] = uwbagree.data.info[i].dist/100.0;
						//matrix_y[i] = sqrt(pow((aplist[j].zxy.x - point[0]),2) + pow((aplist[j].zxy.y - point[1]),2) + pow((aplist[j].zxy.z - point[2]),2));
					}else{
						matrix_x[i*DNUM+0] = aplist[j].zxy.x;
						matrix_x[i*DNUM+1] = aplist[j].zxy.y;
						matrix_y[i] = sqrt(pow(((uwbagree.data.info[i].dist)/100.0),2)-pow((aplist[j].zxy.z-High),2));
					}
					//printf("%f,%f,%f %f %d %d %d %d %d\r\n",aplist[j].zxy.x,aplist[j].zxy.y,aplist[j].zxy.z,matrix_y[i],aplist[j].addr,uwbagree.data.num,i,j,sizeof(aplist)/sizeof(aplist[0]));
				}
			}
		}
		/*
	printf("Distance: ");
	for(k=0;k<4;k++){
		printf("%6.3f   ",matrix_y[k]);
	}
	printf("\r\n");
	*/
	return 0;
}

float Station_Z_Sort(float* coord_x, uint8_t stat_num)
{
	float sort = 0,result=0;
	uint8_t i=0,j=1;

	float *coord_p = (float *)GLOBAL_MALLOC(sizeof(float)*(stat_num)*DNUM);
	memcpy(coord_p,coord_x,sizeof(float)*(stat_num)*DNUM);
	
	for(i=0;i<3;i++)
	{
		for(j=1;j<stat_num;j++)
		{
			//z或hih排序
			if(coord_p[i*DNUM+2]<coord_p[j*DNUM+2])
			{
				sort = coord_p[i*DNUM+2];
				coord_p[i*DNUM+2] = coord_p[j*DNUM+2];
				coord_p[j*DNUM+2] = sort;
			}
		}
		result += coord_p[i*DNUM+2];
	}	

	result = result/4;

	GLOBAL_FREE(coord_p);

	return result;
}



kalman1_state kalman1_x={ .x=0,.p=0,.A=1,.H=1,.q=1,.r=10,.B = 0,.u = 0};
kalman1_state kalman1_y={ .x=0,.p=0,.A=1,.H=1,.q=1,.r=10,.B = 0,.u = 0};

kalman2_state kalman2_x = {.x = {4.01,4.01},.p = {{10e-3,0}, {0,10e-3}},.A = {{1, 0.1}, {0, 1}},.H = {1,0},.q = {10e-3,10e-3},.r = 1};
kalman2_state kalman2_y = {.x = {2.01,2.01},.p = {{10e-3,0}, {0,10e-3}},.A = {{1, 0.1}, {0, 1}},.H = {1,0},.q = {10e-3,10e-3},.r = 1};


int uwb_parse(UWBParse *uwbParse,char *buf)
{
	//int i;
	float Avr_tagz=0;
	if(uwb_uart_parse(&(uwbParse->uwbdata),buf) != 0 || uwbParse->uwbdata.data.num < (DNUM+1)){
		return -1;
	}
	float *matrix_y = (float *)GLOBAL_MALLOC(sizeof(float)*uwbParse->uwbdata.data.num);
	float *matrix_x = (float *)GLOBAL_MALLOC(sizeof(float)*(uwbParse->uwbdata.data.num)*DNUM);	
	float matrix_f[DNUM] = {0};

	aplistparse(uwbParse->uwbdata,uwbParse->ap,matrix_x,matrix_y);//解析基站坐标和距离。
	Avr_tagz = Station_Z_Sort(matrix_x, uwbParse->uwbdata.data.num);
	NOTE_PRINT(("AVR_Z = %6.3f\r\n",Avr_tagz));
	tag_xyz(matrix_x,matrix_y,matrix_f,DNUM,uwbParse->uwbdata.data.num);//计算tag的xyz。
	Matrix_print(matrix_f,DNUM,1);
	NOTE_PRINT(("\r\n"));
	NOTE_PRINT(("\r\n"));
	
	if(1){
		uwbParse->tag.x = matrix_f[0];
		uwbParse->tag.y = matrix_f[1];
#if DNUM == 3
		uwbParse->tag.z = Avr_tagz;//matrix_f[2];
#endif
		uwbParse->uwbdata.data.id ++ ;
	}
	else if(0){
		if(kalman1_x.x == 0 && kalman1_y.x == 0){
		kalman1_x.x = matrix_f[0];
		kalman1_y.x = matrix_f[1];
		}
		uwbParse->tag.x = kalman1_filter(&kalman1_x,matrix_f[0]);
		uwbParse->tag.y = kalman1_filter(&kalman1_y,matrix_f[1]);
#if DNUM == 3
		uwbParse->tag.z = matrix_f[2];
#endif
	}else if(0){
		uwbParse->tag.x = kalman2_filter(&kalman2_x,matrix_f[0]);
		uwbParse->tag.y = kalman2_filter(&kalman2_y,matrix_f[1]);
#if DNUM == 3
	  uwbParse->tag.z = matrix_f[2];
#endif
	}
/*
	if(uwbParse->tag.x>=6||uwbParse->tag.x<=-1||uwbParse->tag.y>=7||uwbParse->tag.y<=-1)
	{
		UWB_SWICH = 1;
	}else if(Least_fail == 1){
		UWB_SWICH = 1;
	}else{
		UWB_SWICH = 0;
	}
	*/
	GLOBAL_FREE(matrix_x);
	GLOBAL_FREE(matrix_y);
	return 0;
}

/*****************************************************************************
 函 数 名  : _EH_uwb_thread
 功能描述  : EH_uwb线程
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年12月21日
    作    者   : zxf
    修改内容   : 新生成函数

*****************************************************************************/
static void _EH_uwb_thread(void* arg)
{
	char *buf = NULL;
	uint16_t num = 0;
	uwbStruct *outbuf = NULL;
	netStruct *out =NULL;
	UWBParse uwbParse = {0};
	
	UwbUart.RX_flag = 0;
	UwbUart.RX_Size = 0;
	
	while(1){
		if(UwbUart.RX_flag == 1 && UwbUart.RX_Size != 0)
		{
			NOTE_PRINT(("Uwb_get_messege!!\r\n"));
			num = UwbUart.RX_Size;
		  	buf = (char *)GLOBAL_MALLOC(num);
			GLOBAL_MEMSET(buf,0,num);
			GLOBAL_MEMCPY(buf,UwbUart.RX_pData,num);
			UwbUart.RX_flag = 0;
			UwbUart.RX_Size = 0;
			NOTE_PRINT(("Uwb num = %d\r\n",num));
        	if(uwb_parse(&uwbParse,buf)<0)
			{
				GLOBAL_FREE(buf);
				continue;
			}
			out = (netStruct *)GLOBAL_MALLOC(sizeof(netStruct));
			GLOBAL_MEMSET(out,0,sizeof(netStruct));
			outbuf = (uwbStruct *)GLOBAL_MALLOC(sizeof(uwbStruct));
			GLOBAL_MEMSET(outbuf,0,sizeof(uwbStruct));
			out->type = 1;
			out->uwb = outbuf;
			GLOBAL_MEMCPY(&(outbuf->uwbParse),&uwbParse,sizeof(UWBParse));
			//if(my_QueuePut(uwbQueueHandle,&out,NULL,NULL)<0){
			GLOBAL_FREE(out);
			//}
			GLOBAL_FREE(buf);
		}
		delay_ms(5);
		
	}
}

/*****************************************************************************
 函 数 名  : EH_uwb_apl_init
 功能描述  : EH_uwb线程初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年12月21日
    作    者   : zxf
    修改内容   : 新生成函数

*****************************************************************************/

void EH_uwb_apl_init(void)
{
	osThreadId_t thread_EH_uwb_id = 0;

	thread_EH_uwb_id = osThreadNew(_EH_uwb_thread, NULL, &thread_EH_uwb_attr);
	GLOBAL_HEX(thread_EH_uwb_id);
}


