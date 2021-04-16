

#ifndef __PUBDEF_H__
#define __PUBDEF_H__

#ifdef __cplusplus
    #if __cplusplus
extern "C"
{
    #endif
#endif /* __cplusplus */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include<string.h>
#include <stdarg.h>
//#include "usart.h"
#include<stdlib.h>
#include <stdio.h>
#include "time.h"
#include "rtx_os.h"
#include <rl_net.h>
#include <rl_net_lib.h>
#include <rl_fs.h>
//#include "delay.h"
//#include "SimpleShell.h"
//#include "core_cm7.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "sys_debug.h"

#if defined(VERSION_OURUAN) 
    #ifdef ANT_SUP
        #define PROJECT_NAME		"GXWS33C_01_OuRuan_ant_supply"
    #else
        #define PROJECT_NAME		"GXWS33C_01_OuRuan_ant_not_supply"
    #endif
#elif defined(VERSION_AB)  
    #if defined(VERSION_ANZE)
        #define PROJECT_NAME		"GXWS33C_01_AB_ANZE"
    #elif defined(VERSION_KY)
        #define PROJECT_NAME		"GXWS33C_01_AB_KY"
    #else 
        #define PROJECT_NAME		"GXWS33C_01_AB"
    #endif
#elif defined(VERSION_HUANUO) 
    #ifdef BDS_INCLUDE
        #define PROJECT_NAME		"GXWS33C_01_HuaNuo_GPS_BDS_GLO"
    #else
        #define PROJECT_NAME		"GXWS33C_01_HuaNuo_GPS_GLO"
    #endif
#else
    #define PROJECT_NAME		"GXWHPR_v1.9.8.1"
#endif

//��������ANSI��ɫ����
#define ANSI_NONE			"\033[0m"		//ȡ����ɫ
#define ANSI_BLACK			"\033[0;30m"	//��ɫ
#define ANSI_DARK_BLACK		"\033[1;30m"	//���ɫ
#define ANSI_BLUE			"\033[0;34m"	//��ɫ
#define ANSI_DARK_BLUE		"\033[1;34m"	//����ɫ
#define ANSI_GREEN			"\033[0;32m"	//��ɫ
#define ANSI_DARK_GREEN		"\033[1;32m"	//����ɫ
#define ANSI_CRAN			"\033[0;36m"	//����ɫ
#define ANSI_DARK_CRAN		"\033[1;36m"	//������ɫ
#define ANSI_RED			"\033[0;31m"	//��ɫ
#define ANSI_DARK_RED		"\033[1;31m"	//���ɫ
#define ANSI_PURPLE			"\033[0;35m"	//��ɫ
#define ANSI_DARK_PURPLE	"\033[1;35m"	//����ɫ
#define ANSI_YELLOW			"\033[0;33m"	//��ɫ
#define ANSI_DARK_YELLOW	"\033[1;33m"	//���ɫ
#define ANSI_GRAY			"\033[0;37m"	//��ɫ
#define ANSI_WHITE			"\033[1;37m"	//��ɫ

//����ɫ
#define ANSI_BG_BLACK		"\033[40;37m"	//��ɫ
#define ANSI_BG_RED			"\033[41;37m"	//��ɫ
#define ANSI_BG_GREEN		"\033[42;37m"	//��ɫ
#define ANSI_BG_YELLOW		"\033[43;37m"	//��ɫ
#define ANSI_BG_BLUE		"\033[44;37m"	//��ɫ
#define ANSI_BG_PURPLE		"\033[45;37m"	//��ɫ
#define ANSI_BG_DARK_GREEN	"\033[46;37m"	//����ɫ
#define ANSI_BG_WHITE		"\033[47;30m"	//��ɫ

#define ANSI_CURSOR_HIDE	"\033[?25l"		//���ع��
#define ANSI_CURSOR_DIS		"\033[?25h"		//��ʾ���

#define ANSI_CLEAR			"\033[H\033[J"		//����

#define KERN_ERR				ANSI_RED	/* error conditions */
#define KERN_WARNING			ANSI_PURPLE	/* warning conditions */
#define KERN_NOTICE				ANSI_BLUE	/* normal but significant condition */
#define KERN_INFO				ANSI_NONE/* informational */


#ifndef NULL
#define NULL		0
#endif

#ifndef	U8
#define	U8			unsigned char
#endif

#ifndef S8
#define S8			signed char			
#endif

#ifndef CHAR_T
#define CHAR_T		char			
#endif

#ifndef U16
#define U16			unsigned short
#endif

#ifndef S16
#define S16			short
#endif

#ifndef U32
#define U32			unsigned int
#endif

#ifndef S32
#define S32			int
#endif

#ifndef U64
#define U64			unsigned long long	
#endif

#ifndef S64
#define S64			long long
#endif

#ifndef BOOL
#define BOOL		long
#endif

#ifndef BOOL8
#define BOOL8		S8
#endif

#ifndef F32
#define F32			float
#endif

#ifndef F64
#define F64			double
#endif

//����һЩ���õ��������Ͷ̹ؼ��� 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  
typedef const int16_t sc16;  
typedef const int8_t sc8;  

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  
typedef __I int16_t vsc16; 
typedef __I int8_t vsc8;   

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  
typedef const uint16_t uc16;  
typedef const uint8_t uc8; 

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  
typedef __I uint16_t vuc16; 
typedef __I uint8_t vuc8;  

#define SIZE_1		(1U)
#define SIZE_8		(8U)
#define SIZE_16		(16U)
#define SIZE_32		(32U)
#define SIZE_64		(64U)
#define SIZE_128	(128U)
#define SIZE_256	(256U)
#define SIZE_512	(512U)
#define SIZE_1K		(1024U)
#define SIZE_2K		(2048U)
#define SIZE_3K		(3072U)
#define SIZE_4K		(4096U)
#define SIZE_5K		(5120U)
#define SIZE_6K		(6144U)
#define SIZE_7K		(7168U)
#define SIZE_8K		(8192U)
#define SIZE_9K		(9216U)
#define SIZE_10K	(10240U)
#define SIZE_20K	(20480U)

#ifndef TRUE
#define TRUE		1
#endif

#ifndef FALSE
#define FALSE		0
#endif

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#ifndef INOUT
#define INOUT
#endif

#define SECONDS_20180101	1514736000	// 2018��1��1��0ʱ0��0�룬����ʱ��
#define SECONDS_19800106	315964800	//1980��1��6��8ʱ0��0�룬����ʱ��	
#define TRIAL_TOTAL_SECONDS		(15*86400)	//����ʱ��

#define PI (F64)3.14159265358979323846	//Բ����
#define C  299792458.0				//���٣�m/s
#define recip_C 3.335640951981520e-09	//���ٵ�����s/m
#define F -4.442807633e-10 		//�����ЧӦ��������
#define Iz 5e-9						//�������ʱ������ ��
#define e_Earth 0.08181919104281579

/*WGS84����ϵ����*/
#define WGS84_A		(F64)6378137.0				//���뾶
#define WGS84_B		6356752.314			//�̰���
#define WGS84_F		1/298.257223563		//������
#define WGS84_WE	7.2921151467E-5		//������ת���ٶ�
#define WGS84_GM	3.986004418E+014			//������������������˻�
#define WGS84_ECC 	6.69437999014E-003		//ƫ����

/*CGCS2000����ϵ����*/
#define CGCS2000_A		6378137.0				//���뾶
#define CGCS2000_F		1/298.257222101		//������
#define CGCS2000_WE	7.2921150E-5			//������ת���ٶ�
#define CGCS2000_GM	3.986004418E14			//������������������˻�
#define CGCS2000_ECC 	0.00669438002290		//ƫ����

/*PZ-90����ϵ����*/
#define	 PZ90_A			(6378137.0)				//������
#define	 PZ90_F			(1/298.257839303)		//������
#define	 PZ90_WE		7.2921150E-5			//������ת���ٶ�
#define  PZ90_GM		(3.986004418E+14)			//������������������˻�
#define  PZ90_ECC 		6.69437999014E-003		//ƫ����
#define  PZ90_J02		(1.0826257E-3)			/*PZ-90������ʵĶ��״�гϵ��*/

#define M_DEG2RAD(x)	(((F64)x)*0.017453292519943)				//�Ƕ�ת����
#define M_RAD2DEG(x)	(((F64)x)*57.29577951308232)				//����ת�Ƕ�

#define KNToMS          (0.51444)                //��(����/Сʱ)ת M/S

#ifndef SWAP
#define SWAP(type,a,b){type tmp; tmp=a; a=b; b=tmp;}
#endif

#define FLOAT_CMP_CALIB		(1e-6)

#ifndef BigLittleSwap16
#define BigLittleSwap16(A)        ((((U16)(A) & 0xff00) >> 8) | \
                                                       (((U16)(A) & 0x00ff) << 8))
#endif

#ifndef BigLittleSwap32
#define BigLittleSwap32(A)        ((((U32)(A) & 0xff000000) >> 24) | \
                                                       (((U32)(A) & 0x00ff0000) >> 8) | \
                                                       (((U32)(A) & 0x0000ff00) << 8) | \
                                                       (((U32)(A) & 0x000000ff) << 24))
#endif
extern U8 printf_type;
extern void print_def(CHAR_T * fmt ,...);
#define PRINT_IMM print_def
#define GPRINT	print_def

//#define print printf
#ifndef GLOBAL_INTVAL
#define GLOBAL_INTVAL(x) GPRINT("[%s][%d] ", __FUNCTION__, __LINE__);GPRINT("%s = %ld\r\n", #x, x)
#endif

#ifndef GLOBAL_UINTVAL
#define GLOBAL_UINTVAL(x) GPRINT("[%s][%d] ", __FUNCTION__, __LINE__);GPRINT("%s = %lu\r\n", #x, x)
#endif


#ifndef GLOBAL_HEX
#define GLOBAL_HEX(x) GPRINT("[%s][%d] ", __FUNCTION__, __LINE__);GPRINT("%s = 0x%X\r\n", #x, x)
#endif


#ifndef GLOBAL_FLTVAL
#define GLOBAL_FLTVAL(x) GPRINT("[%s][%d] ", __FUNCTION__, __LINE__);GPRINT("%s = %.3lf\r\n", #x, x)
#endif

#ifndef GLOBAL_DBFLTVAL
#define GLOBAL_DBFLTVAL(x) GPRINT("[%s][%d] ", __FUNCTION__, __LINE__);GPRINT("%s = %.12lE\r\n", #x, x)
#endif


#ifndef GLOBAL_TRACE
#define GLOBAL_TRACE(x) GPRINT("[%s][%d] ", __FUNCTION__, __LINE__);GPRINT x;
#endif

#ifndef GLOBAL_PRINT
#define GLOBAL_PRINT(x) GPRINT x;
#endif

#ifndef GLOBAL_TIME_PRINT
#define GLOBAL_TIME_PRINT(x) GPRINT("[%02dd %02d:%02d:%02d]",sysup_seconds_g/86400, sysup_seconds_g%86400/3600, sysup_seconds_g%3600/60, sysup_seconds_g%60); GPRINT x;
#endif

#ifndef ERR_PRINT
#define ERR_PRINT(x)	if(sys_debug_get_type(SYS_DEBUG_ERROR))\
						{GPRINT("\r\n "ANSI_BG_RED"ERROR "ANSI_NONE); \
						GPRINT("[%s][%d] ", __FUNCTION__, __LINE__);GPRINT x;}
#endif

#ifndef ERR_LICENSE
#define ERR_LICENSE(x)	GPRINT("\r\n "ANSI_BG_RED"ERROR "ANSI_NONE);GPRINT x
#endif


#ifndef WARN_PRINT
#define WARN_PRINT(x)	if(sys_debug_get_type(SYS_DEBUG_WARN))\
						{GPRINT("\r\n "ANSI_BG_PURPLE"WRANNING "ANSI_NONE); \
						GPRINT("[%s][%d] ", __FUNCTION__, __LINE__);GPRINT x;}
#endif

#ifndef NOTE_PRINT
#define NOTE_PRINT(x)	if(sys_debug_get_type(SYS_DEBUG_NOTE))\
						{GPRINT("\r\n "ANSI_BG_BLUE"NOTE: "ANSI_NONE); \
						GPRINT("[%s][%d] ", __FUNCTION__, __LINE__);GPRINT x;}
#endif

#ifndef APP_IS_BIT_SET
#define APP_IS_BIT_SET(value, bit)			((value & (1 << bit)) ? 1 : 0)
#endif
#ifndef APP_SET_BIT
#define APP_SET_BIT(value, bit)				(value |= 1<<bit)
#endif
#ifndef APP_CLEAR_BIT
#define APP_CLEAR_BIT(value, bit)			(value &= (~(1<<bit)))
#endif

#ifndef CALC_VECTOR_MOD
#define CALC_VECTOR_MOD(x,y,z) sqrt(x*x+y*y+z*z)
#endif

/*C ��׼�����꣬Ϊ�˿��Է�����滻����Ϊ�Զ���ĺ���*/
#ifndef  GLOBAL_MALLOC
#define	GLOBAL_MALLOC(x) malloc(x)
#endif

#ifndef  GLOBAL_CALLOC
#define	GLOBAL_CALLOC(count, sizeof) calloc(count, sizeof)
#endif

#ifndef  GLOBAL_FREE
#define	GLOBAL_FREE(x) free(x)
#endif

#ifndef  GLOBAL_MEMCPY
#define	GLOBAL_MEMCPY(des,src,sizeof) memcpy(des,src,sizeof)
#endif

#ifndef  GLOBAL_MEMMOVE
#define	GLOBAL_MEMMOVE(des,src,sizeof) memmove(des,src,sizeof)
#endif

#ifndef  GLOBAL_MEMSET
#define	GLOBAL_MEMSET(des,value,sizeof) memset(des,value,sizeof)
#endif

#ifndef  GLOBAL_MEMCMP
#define	GLOBAL_MEMCMP(a,b,sizeof) memcmp(a,b,sizeof)
#endif

#ifndef  GLOBAL_MEMMOVE
#define	GLOBAL_MEMMOVE(d,s,sizeof) memmove(d,s,sizeof)
#endif

#ifndef  GLOBAL_STRCPY
#define	GLOBAL_STRCPY(des,src) strcpy((char*)des,(char*)src)
#endif

#ifndef  GLOBAL_STRNCPY
#define	GLOBAL_STRNCPY(des,src, size) strncpy((char*)des,(char*)src, size)
#endif

#ifndef  GLOBAL_STRCAT
#define	GLOBAL_STRCAT(des,src) strcat(des,src)
#endif


#ifndef  GLOBAL_STRCMP
#define	GLOBAL_STRCMP(a,b) strcmp((const char*)a,b)
#endif

#ifndef  GLOBAL_STRSTR
#define	GLOBAL_STRSTR(a,b) strstr((const char*)a,b)
#endif

#ifndef  GLOBAL_STRNCMP
#define	GLOBAL_STRNCMP(a,b,s) strncmp((const char*)a,b,s)
#endif

#ifndef  GLOBAL_STRLEN
#define	GLOBAL_STRLEN(a) strlen((char*)a)
#endif

#ifndef  GLOBAL_MEMCPYSTR
#define	GLOBAL_MEMCPYSTR(des,src) memcpy((const char*)des,(const char*)src,GLOBAL_STRLEN(src))
#endif

#ifndef  GLOBAL_MEMSTRCMP
#define	GLOBAL_MEMSTRCMP(des,src) memcmp(des,src,GLOBAL_STRLEN(src))
#endif


#ifndef GLOBAL_STRCASECMP
#define GLOBAL_STRCASECMP(a,b) strcasecmp((const char*)a, (const char*)b)
#endif

#ifndef GLOBAL_STRNCASECMP
#define GLOBAL_STRNCASECMP(a,b,n) strncasecmp((const char*)a, (const char*)b, n)
#endif

#ifndef GLOBAL_SSCANF
#define GLOBAL_SSCANF sscanf
#endif

#ifndef GLOBAL_ATOI
#define GLOBAL_ATOI atoi
#endif


#ifndef GLOBAL_STRTOL
#define GLOBAL_STRTOL strtol
#endif


#ifndef GLOBAL_STRTOUL
#define GLOBAL_STRTOUL strtoul
#endif

#ifndef GLOBAL_STRTOD
#define GLOBAL_STRTOD strtod
#endif










#ifndef SECONDS_UNIX_GPS
#define SECONDS_UNIX_GPS		315964800		//UNIXʱ���ת����GPSʱ����Ĳ���ֵ,1970��1��1����1980���1��6��0ʱ,
#endif

#ifndef TOTAL_SECONDS_WEEK
#define TOTAL_SECONDS_WEEK		604800
#endif

#ifndef GPS_BDS_DIFF_SECS
#define GPS_BDS_DIFF_SECS	(14)//GPS�뱱��ʱ���������ֵ
#endif

#ifndef TOTAL_SECONDS_DAY
#define TOTAL_SECONDS_DAY		86400
#endif

#define GLONASS_OFFSET_UTC		10800

#define Gps_To_1996 (504489618)


#ifndef GPS_WEEK_MOD
#define GPS_WEEK_MOD		1024	//GPS������ģֵ
#endif

#ifndef NO_WARNING
#define NO_WARNING(x)	 	(void)(x)		/* no warning */
#endif


#define STM32H7XX_FLASH_SIZE                (0x200000) /* 2MB */
#define JUMP_APP_OFFSET                     (0x20000)
#define SYS_PARA_FILE                       "para.bin"
#define NET_PARA_FILE                       "net.bin"
#define LICENSE_FILE                        "license.bin"
#define HN_CFG_FILE                         "hn_cfg.bin"
#define ARM_UPLOAD_FILE_NAME                "arm.bin"
#define FPGA_UPLOAD_FILE_NAME               "fpga.bin"
#define ARM_UPLOAD_FILE_NAME_TEMP           "arm_temp.bin"
#define FPGA_UPLOAD_FILE_NAME_TEMP          "fpga_temp.bin"
#define UPLOAD_FILE_NAME_TEMP               "upload_temp.bin"
#define POS_PARA_FILE                       "position_para.bin"
#define UPDATE_FLAG_FILE                    "update_flag.bin"


// �ߵ͵�ƽ
#define LEVEL_HIGH 1
#define LEVEL_LOW  0 

// wzp 2016-06-21 add 
typedef union
{
	U32 u32Val;
	U16 u16Val[2];
	U8 u8Val[4];
}UN32INT;

typedef union
{
	U16 u16Val;
	U8 u8Val[2];	
}UN16INT;

typedef enum
{
	FILE_CHECK_NULL = 0x0,
	FILE_CHECK_ERROR,
	FILE_CHECK_SUC,
}FILE_CHECK_Enum;

typedef enum
{
	FILE_ARM = 0x0,
	FILE_FPGA,
	FILE_OTHER,
}FILE_UPLOAD_Enum;

typedef enum
{
	BKUP_BOOT_NORMAL = 0x0000,
	BKUP_BOOT_UPGRADE = 0x1111,
}BKUP_BOOT_TYPE_Enum;


#define dbm2W(dbm) (pow(10,(dbm)/10.0))/1000.0

#define BCD2BIN16(x)	(((x>>12)&0xF)*1000+((x>>8)&0xF)*100+((x>>4)&0xF)*10 + (x&0x0F))
#define BIN2BCD16(x)	((((x/1000)%10)<<12) |(((x/100)%10)<<8) | (((x%100)/10)<<4) | (x%10))

#define BCD2BIN(x)	(((x>>4)&0xF)*10 + (x&0x0F))
#define BIN2BCD(x)	((((x%100)/10)<<4) | (x%10))
#define ABS_DATA(x)							(((x) >= 0) ? (x) : (-(x)))
#define	MAX(a,b)							(((a) > (b)) ? (a) : (b))

#define GPIO_LOW	GPIO_PIN_RESET
#define GPIO_HIGH	GPIO_PIN_SET

U16 U16Swap(U16 *pt16DataIn);
U32 U32Swap(U32 *pt32DataIn);
void HexShow(char * Buf ,int Len);
F32 W2dbm(F32 w);
#define delay_ms(x)	osDelay(x*10)
extern void delay_us(U32 x);
extern char* itoa(int num,char*str,int radix);
extern U8 isLeapYear(U32 year);
extern void getYYMMDDFromStr(const char* p_str,char* yymmdd);
extern U8 compare_data_continue_not_equal(S32 data1, S32 data2, U32* cnt, U32 cpm_times);
extern FILE_CHECK_Enum file_upload_check(U8* file_name, FILE_UPLOAD_Enum* file_type);
extern int indexOfArray(U8 container[], int len1, U8 target[] ,int len2);
extern U16 countCRC16(U8 *addr, int num);
extern void bubble_sort_int(S32* buf, U32 len);
extern void bubble_sort_float(F32* buf, U32 len);
extern int makeChecksum(char data[], int lenght);
extern int getChecksum(char data[], int lenght);
extern int sign(double x);
extern void reboot_cmd(void);
extern U8 net_ip_is_legal(U8* buf);
extern U8 net_mac_is_legal(U8* buf);

extern void rtc_bakup_write(U16 boot_type);
extern void update_flag_write(U16 boot_type);


//void glo_calc_nav_time(S32 gps_time, S32* N4, S32* Nt);
//#define BCD2BIN16(x)	(((x>>12)&0xF)*1000+((x>>8)&0xF)*100+((x>>4)&0xF)*10 + (x&0x0F))

extern void error_handler(char *file, int line);

extern S64 rounding(F64 x);

#ifndef GPS_LEAP_SECONDS
#define GPS_LEAP_SECONDS 18
#endif





//#define delay_us(x)	{int i= 120;while(x--){while(i--)}}
//extern void delay_ms(U32 nTime);
//extern void delay_us(U32 nTime);

#ifdef __cplusplus
    #if __cplusplus
}
    #endif
#endif /* __cplusplus */

#endif /* __PUBDEF_H__ */


