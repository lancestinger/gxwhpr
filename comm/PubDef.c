
// Header File

//#include <stm32f7xx.h>


#include "PubDef.h"
//#include "timer/timer.h"
//#include "multi_debug.h"
#include "project_def.h"
//#include "sys_time.h"

// Function List


int sign(double x)
{
	int y;
	if(x>=0) 
		y=1;
	else 
		y=-1;
	return(y);
}

uint16_t U16Swap(uint16_t *pt16DataIn)
{
	UN16INT un16DataTmp;

	un16DataTmp.u16Val = *pt16DataIn;
	
	un16DataTmp.u8Val[0] ^= un16DataTmp.u8Val[1];
	un16DataTmp.u8Val[1] ^= un16DataTmp.u8Val[0];
	un16DataTmp.u8Val[0] ^= un16DataTmp.u8Val[1];

	return(un16DataTmp.u16Val);
}
uint32_t U32Swap(uint32_t *pt32DataIn)
{
	UN32INT un32DataTmp;

	un32DataTmp.u32Val = *pt32DataIn;
	
	un32DataTmp.u8Val[0] ^= un32DataTmp.u8Val[3];
	un32DataTmp.u8Val[3] ^= un32DataTmp.u8Val[0];
	un32DataTmp.u8Val[0] ^= un32DataTmp.u8Val[3];

	un32DataTmp.u8Val[1] ^= un32DataTmp.u8Val[2];
	un32DataTmp.u8Val[2] ^= un32DataTmp.u8Val[1];
	un32DataTmp.u8Val[1] ^= un32DataTmp.u8Val[2];

	return(un32DataTmp.u32Val);
}

F32 W2dbm(F32 w) 
{
	if(w < (F32)FLOAT_CMP_CALIB)	
	{						
		return 0;			
	}						
	else 					
	{						
		return (10*log10(w*1000));
	}						\
}

void HexShow(char * Buf ,int Len)
{
	int i;
	for(i = 0;i < Len;i++)
	{
		GLOBAL_PRINT(("%2.2x ",Buf[i]));
	}
	GLOBAL_PRINT(("\r\n"));
}

// void delay_us(U32 x)
// {
// 	extern void timer2_wait_us(U32 us);
// 	/*u8 i = 0;
// 	while(x--)
// 	{
// 		for(i = 0; i < 120; i++);
// 	}*/
// 	//TIM3_Delay_us(x);
// 	timer2_wait_us(x);
// }

char* itoa(int num,char*str,int radix)
{
	/*索引表*/
	char index[]="0123456789ABCDEF";
	unsigned unum;/*中间变量*/
	int i=0,j,k;
	/*确定unum的值*/
	if(radix==10&&num<0)/*十进制负数*/
	{
		unum=(unsigned)-num;
		str[i++]='-';
	}
	else unum=(unsigned)num;/*其他情况*/
	/*转换*/
	do{
		str[i++]=index[unum%(unsigned)radix];
		unum/=radix;
	}while(unum);
	str[i]='\0';
	/*逆序*/
	if(str[0]=='-')k=1;/*十进制负数*/
	else k=0;
	char temp;
	for(j=k;j<=(i-1)/2;j++)
	{
		temp=str[j];
		str[j]=str[i-1+k-j];
		str[i-1+k-j]=temp;
	}
	return str;
}

//用于搜寻帧头
int indexOfArray(U8 container[], int len1, U8 target[] ,int len2) 
{
	int start = 0;
	if (len2 > 0) 
    {
		if (len2 > len1) 
        {
			return -1;
		}

		while (1) 
        {
			for (; start < len1; start++) 
            {
				if (container[start] == target[0])
					break;
			}

			if (start == len1 || len2 + start > len1) 
            {
				return -1;
			}
			int o1 = start, o2 = 0;
			while (o1 < len1 && o2 < len2) 
            {
				if (container[o1] != target[o2]) 
                {
					break;
				}
				o1++;
				o2++;
			}
			if (o2 == len2) 
            {
				return start;
			}
			start++;
		}
	}
	return -1;
}

U8 isLeapYear(U32 year)
{
	U8 ret = 0;
	if((year%4==0&&year%100!=0)||(year%400==0)) 
	{
		ret=1;
	}else
	{
		ret=0; 
	}
	return ret;
}


//4个字节按照排列顺序凑成整数
static int fourValue(char a, char b,char c,char d)
{
	union fourValue{
		char val[4];
		int x;
	} value;
	value.val[0] = a;
	value.val[1] = b;
	value.val[2] = c;
	value.val[3] = d;
	return value.x;
}


int makeChecksum(char data[], int lenght)
{
	int total = 0;
	int num = 0;
	#if 1
	while (num < lenght){
		total = total + data[num];//fourValue(data[num + 3], data[num + 2], data[num + 1], data[num + 0]);
		num ++;
	}
	#else
	while (num < lenght - 4){
		total = total + fourValue(data[num + 3], data[num + 2], data[num + 1], data[num + 0]);
		num += 4;
	}
	#endif
//	total = total % 4294967296;
	int ck = fourValue(data[num + 0], data[num + 1], data[num + 2], data[num + 3]);
	if (ck == total)
		return 1;
	else
	{
		ERR_PRINT(("rcv_check = %08x, calc_check = %08x\r\n", ck, total));
		return 0;
	}
}

int getChecksum(char data[], int lenght)
{
	int total = 0;
	int num = 0;
	#if 1
	while (num < lenght){
		total = total + data[num];//fourValue(data[num + 3], data[num + 2], data[num + 1], data[num + 0]);
		num ++;
	}
	#else
	while (num < lenght - 4){
		total = total + fourValue(data[num + 3], data[num + 2], data[num + 1], data[num + 0]);
		num += 4;
	}
	#endif
//	total = total % 4294967296;
	//int ck = fourValue(data[num + 0], data[num + 1], data[num + 2], data[num + 3]);
	return total;
	/*if (ck == total)
		return 1;
	else
	{
		ERR_PRINT(("rcv_check = %08x, calc_check = %08x\r\n", ck, total));
		return 0;
	}*/
}

const char s_cMonth[]   = "JanFebMarAprMayJunJulAugSepOctNovDec";

/**
  * @brief  convert __DATE__ to yymmdd
  * @param  const char* p_str,Pre-Processing __DATE__ 
  *         data as "Aug 27 2013" "Aug  1 2013"
  * @param  char* yymmdd,lenght 3 Bytes for year,month,day
  * @retval None
  */
void getYYMMDDFromStr(const char* p_str,char* yymmdd)
{
	char month = 0;
	char lenght = GLOBAL_STRLEN(p_str);

	if (lenght != 11)
	{
		return;
	}

	for (month = 0; month < 12;month++)
	{
		if (0 == memcmp(&s_cMonth[month*3], p_str, 3))
		{
			month++;
			break;
		}
	}

	*yymmdd++ = (p_str[9]-'0')*10+(p_str[10]-'0');
	*yymmdd++ = month;
	if ((p_str[4] < '0') || (p_str[4] > '3'))//over 01~31
	{
		*yymmdd = p_str[5]-'0';
	}
	else
	{
		*yymmdd = (p_str[4]-'0')*10+(p_str[5]-'0');
	}
}

/*****************************************************************************
 函 数 名  : compare_data_continue_not_equal
 功能描述  : 连判两个数是否不相等
 输入参数  : S32 data1  
             S32 data2  
             U32* cnt   
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2018年9月19日 星期三
    作    者   : wenzhiquan
    修改内容   : 新生成函数

*****************************************************************************/
U8 compare_data_continue_not_equal(S32 data1, S32 data2, U32* cnt, U32 cpm_times)
{
	U8 ret = FALSE;

	if(data1 != data2)
	{
		(*cnt)++;
		if(*cnt > cpm_times)
		{
			ret = TRUE;
		}
	}
	else
	{
		*cnt = 0;
		ret = FALSE;
	}
	return ret;
}

FILE_CHECK_Enum file_upload_check(U8* file_name, FILE_UPLOAD_Enum* file_type)
{
	FILE_CHECK_Enum ret = FILE_CHECK_NULL;
	FILE *fd = NULL;
	U32 i,j;
	U8 Data;
	U32 FileSize;
	U32 version = 0;
	U16 FileCrc16,crc;
	U8 str_buf[10], type = FILE_OTHER;

	GLOBAL_PRINT(("ENTERED file_upload_check!!\r\n"));
	if(!file_name)
	{
		return ret;
	}
	
	GLOBAL_PRINT(("Ready to fopen!!\r\n"));

	if((fd = fopen ((const char *)file_name, "r")) != NULL)
	{
		/* 读取文件信息 */
		GLOBAL_TRACE((KERN_INFO"\r\nRead File Size ... "));
		fread(&FileSize, sizeof(FileSize), 1, fd);
		GLOBAL_TRACE(("OK\r\n"));
		GLOBAL_TRACE(("Read File CRC ... "));
		fread(&FileCrc16, sizeof(FileCrc16), 1, fd);
		GLOBAL_TRACE(("OK\r\n"));
		GLOBAL_TRACE(("Read File Version ... "));
		fread(&version, sizeof(version), 1, fd);
		GLOBAL_TRACE(("OK\r\n"));
		GLOBAL_TRACE(("Read File Discription ... "));
		fread(str_buf, sizeof(str_buf), 1, fd);
		GLOBAL_TRACE(("OK\r\n"));
		if(!GLOBAL_STRNCASECMP((const char *)str_buf, "armhpr", 6))
		{
			type = FILE_ARM;
		}
		else
		{
            fclose(fd);
			WARN_PRINT(("未知文件类型!!!\r\n"));
			return FILE_CHECK_ERROR;
		}
		/* 文件大小检测 */
		if((type == FILE_ARM) && (FileSize > (STM32H7XX_FLASH_SIZE - JUMP_APP_OFFSET)))
		{
			fclose(fd);
			GLOBAL_TRACE((KERN_ERR"FileSize Is Too Large! (%d)\r\n"KERN_INFO,FileSize));
			return FILE_CHECK_ERROR;
		}
		
		/* 文件校验 */
		GLOBAL_TRACE(("Check The File ... "));
		fseek(fd, 0, SEEK_SET);	//重新定义文件头为新位置
		fseek(fd, 1024, SEEK_SET);	//重新定义文件头偏移1024字节处为新位置
		for(i = 0,crc = 0;i < FileSize;i++)
		{
			fread(&Data, sizeof(Data), 1, fd);
			crc = crc ^ (Data << 8);   
			for(j = 8; j != 0; j--) 
			{
				if(crc & 0x8000) 
				{
					crc = (crc << 1) ^ 0x1021; 
				}
				else
				{
					crc = crc << 1;
				}
			}    
		}
		if(crc != FileCrc16)
		{
			fclose(fd);
			GLOBAL_TRACE((KERN_ERR"Failure!\r\n"KERN_INFO));
			return FILE_CHECK_ERROR;
		}
		fclose(fd);
		*file_type = (FILE_UPLOAD_Enum)type;
		GLOBAL_TRACE(("OK \r\n"));
		ret = FILE_CHECK_SUC;
	}
	return ret;
}

//CRC-16生成多项式为：X16+X15+X2+1
//参数：字节数组指针，数组长度
//返值：CRC-16计算值
U16 countCRC16(U8 *addr, int num)
{
	U16 crc = 0xFFFF;
	U16 i;
	while (num--)
	{
		crc ^= *(addr++);
		for (i = 0; i < 8; i ++)
		{
			if (crc & 1)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
			{
				crc >>= 1;
			}
		}
	}
	return crc;
}

//冒泡排序
void bubble_sort_int(S32* buf, U32 len)
{
	U32 i = 0, j = 0;

	for(i = 0; i < len; i++)
	{
		for(j = i; j < len; j++)
		{
			if(buf[i] > buf[j])
			{
				SWAP(S32, buf[i], buf[j]);
			}
		}
	}
}

void bubble_sort_float(F32* buf, U32 len)
{
	U32 i = 0, j = 0;

	for(i = 0; i < len; i++)
	{
		for(j = i; j < len; j++)
		{
			if(buf[i] > buf[j])
			{
				SWAP(F32, buf[i], buf[j]);
			}
		}
	}
}





/*****************************************************************************
 函 数 名  : error_handler
 功能描述  : 错误处理
 输入参数  : char *file  
             int line    
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年5月29日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void error_handler(char *file, int line)
{
  while(1)
  {
  }
}

/*****************************************************************************
 函 数 名  : rounding
 功能描述  : 浮点数四舍五入转整数
 输入参数  : F64 x  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年7月18日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
S64 rounding(F64 x)
{
	S64 y;
	if(x>=0)
	{
		if(x-(S64)x>=0.5)
		    y=(S64)x+1;
	    else
		    y=(S64)x;
	}
	else
	{
      if((S64)x-x>=0.5)
		    y=(S64)x-1;
	    else
		    y=(S64)x;
	}
	return(y);
}

/*****************************************************************************
 函 数 名  : reboot_cmd
 功能描述  : 重启
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年9月15日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void reboot_cmd(void)
{
	WARN_PRINT(("system is rebooting now!!!!!!!!\r\n"));
	delay_ms(1000);
    NVIC_SystemReset();
}


//判断IP地址是否合法
U8 net_ip_is_legal(U8* buf)
{
	if((buf[0] == 0) && (buf[1] == 0) && (buf[2] == 0) && (buf[3] == 0))
	{
		return 0;
	}
	if((buf[0] == 0xFF) && (buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] ==0xFF))
	{
		return 0;
	}
	return 1;
}

//判断MAC地址是否合法
U8 net_mac_is_legal(U8* buf)
{
	if(buf[0] & 0x01)
	{
		return 0;
	}
	if((buf[0] == 0) && (buf[1] == 0) && (buf[2] == 0)
		&& (buf[3] == 0) && (buf[4] == 0) && (buf[5] == 0))
	{
		return 0;
	}
	return 1;
}

void update_flag_write(U16 boot_type)
{
	FILE* fd = 0;
	u16 crc;
	u16 len;
	U8 buf[10] = {0};

	len = 2;
	GLOBAL_MEMCPY(buf, (U8*)&boot_type, len);
	crc = crc16(buf, 2);
	buf[len] = crc/256;
	buf[len+1] = crc%256; 
	len += 2;

	if((fd = fopen(UPDATE_FLAG_FILE, "w")) != NULL)
	{
		fwrite(buf, len, 1, fd);
		fclose(fd);
	}
	else
	{
		ERR_PRINT(("升级标志保存失败!\r\n"));
	}

}



//写RTC备份寄存器
void rtc_bakup_write(U16 boot_type)
{
	RTC_HandleTypeDef RTC_Handler;  //RTC句柄
	RTC_Handler.Instance=RTC;
	__HAL_RCC_RTC_CLK_ENABLE();//使能电源时钟PWR
	HAL_PWR_EnableBkUpAccess();//取消备份区域写保护
	HAL_RTCEx_BKUPWrite(&RTC_Handler,RTC_BKP_DR1,boot_type);
}

/*eof*/
