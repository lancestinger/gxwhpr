/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : crc.c
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年9月14日
  最近修改   :
  功能描述   : CRC模块
  函数列表   :
  修改历史   :
  1.日    期   : 2019年9月14日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/


/*------------------------------头文件------------------------------*/
#include "crc/crc.h"
/*------------------------------头文件------------------------------*/


/*------------------------------文件宏------------------------------*/

#define P_16 					0xA001
#define P_32 					0xEDB88320L
#define P_CCITT 				0x1021
#define P_DNP 					0xA6BC
#define P_KERMIT 				0x8408
#define P_SICK 					0x8005
/*------------------------------文件宏------------------------------*/


/*------------------------------文件变量------------------------------*/

static S32 crc_tab16_init = false;
static S32 crc_tab32_init = false;
static S32 crc_tabccitt_init = false;
static S32 crc_tabdnp_init = false;
static S32 crc_tabkermit_init = false;

static U16 crc_tab16[256];
static U32 crc_tab32[256];
static U16 crc_tabccitt[256];
static U16 crc_tabdnp[256];
static U16 crc_tabkermit[256];
/*------------------------------文件变量------------------------------*/


/*------------------------------函数声明------------------------------*/

/*------------------------------函数声明------------------------------*/








/* Function List*/

//创建CRC-16 表
static void _init_crc16_tab(void)
{
	U16 i, j;
	U16 crc, c;
	for(i = 0; i < 256; i++)
	{
		crc = 0;
		c = i;
		for(j = 0;j < 8;j++)
		{
			if((crc ^ c) & 0x0001)
			{
				crc =(crc >> 1) ^ P_16;
			}
			else
			{
				crc = crc >> 1;
			}
			c = c >> 1;
		}
		crc_tab16[i] = crc;
	}
	crc_tab16_init = true;
}


static void _init_crckermit_tab(void)
{
	U16 i, j;
	U16 crc, c;
	for(i = 0;i < 256;i++)
	{
		crc = 0;
		c =  i;
		for (j = 0;j < 8;j++)
		{
			if((crc ^ c) & 0x0001)
			{
				crc = (crc >> 1) ^ P_KERMIT;
			}
			else
			{
				crc = crc >> 1;
			}
			c = c >> 1;
		}
		crc_tabkermit[i] = crc;
	}
	crc_tabkermit_init = true;
}



static void _init_crcdnp_tab(void)
{
	U16 i,j;
	U16 crc, c;
	for (i = 0;i < 256;i++)
	{
		crc = 0;
		c = i;
		for (j = 0;j < 8;j++)
		{
			if((crc ^ c) & 0x0001)
			{
				crc = ( crc >> 1 ) ^ P_DNP;
			}
			else
			{
				crc = crc >> 1;
			}
			c = c >> 1;
		}
		crc_tabdnp[i] = crc;
	}
	crc_tabdnp_init = true;
}



static void _init_crc32_tab(void)
{
	U32 i, j;
	U32 crc;
	for (i = 0;i < 256;i++)
	{
		crc = i;
		for (j = 0;j < 8;j++)
		{
			if (crc & 0x00000001L)
			{
				crc = (crc >> 1) ^ P_32;
			}
			else
			{
				crc = crc >> 1;
			}
		}
		crc_tab32[i] = crc;
	}
	crc_tab32_init = true;
}


static void _init_crcccitt_tab(void)
{
	U16 i, j;
	U16 crc, c;
	for (i = 0;i < 256;i++)
	{
		crc = 0;
		c = i << 8;
		for (j = 0;j < 8;j++)
		{
			if ((crc ^ c) & 0x8000 )
			{
				crc = (crc << 1) ^ P_CCITT;
			}
			else
			{
				crc = crc << 1;
			}
			c = c << 1;
		}
		crc_tabccitt[i] = crc;
	}
	crc_tabccitt_init = true;
} 



static U16 _crc_ccitt(U16 crc, U8 c)
{
	U16 u16DataTmp;
	UN16INT un16ShortC;

	un16ShortC.u16Val = 0;
	un16ShortC.u8Val[0] = c;
	if(false == crc_tabccitt_init)
	{
		_init_crcccitt_tab();
	}
	u16DataTmp = (crc >> 8) ^ un16ShortC.u16Val;
	crc = (crc << 8) ^ crc_tabccitt[u16DataTmp];
	return(crc);
}

static U16 _crc_sick(U16 crc,U8 c,U8 prev_byte)
{
	UN16INT un16DataTmp;
	
	un16DataTmp.u16Val = 0;
	un16DataTmp.u8Val[0] = c;
	un16DataTmp.u8Val[1] = prev_byte;
	if (crc & 0x8000)
	{
		crc = (crc << 1) ^ P_SICK;
	}
	else
	{
		crc = crc << 1;
	}
	crc ^= un16DataTmp.u16Val;
	return(crc);
} 


static U16 _crc_16(U16 crc,U8 c)
{
	UN16INT un16DataTmp;

	un16DataTmp.u16Val = 0;
	un16DataTmp.u8Val[0] = c;
	if(false == crc_tab16_init)
	{
		_init_crc16_tab();
	}
	un16DataTmp.u16Val ^= crc;
	crc = (crc >> 8) ^ crc_tab16[un16DataTmp.u8Val[0]];
	return(crc);
}

static U16 _crc_kermit(U16 crc,U8 c)
{
	UN16INT un16DataTmp;

	un16DataTmp.u16Val = 0;
	un16DataTmp.u8Val[0] = c;
	
	if(false == crc_tabkermit_init )
	{
		_init_crckermit_tab();
	}
	un16DataTmp.u16Val ^= crc;
	crc = (crc >> 8) ^ crc_tabkermit[un16DataTmp.u8Val[0]];
	return(crc);
}


static U16 _crc_dnp(U16 crc,U8 c)
{
	UN16INT un16DataTmp;

	un16DataTmp.u16Val = 0;
	un16DataTmp.u8Val[0] = c;
	if(false == crc_tabdnp_init)
	{
		_init_crcdnp_tab();
	}
	un16DataTmp.u16Val ^= crc;
	crc = (crc >> 8) ^ crc_tabdnp[un16DataTmp.u8Val[0]];
	return(crc);
}


static U32 _crc_32(U32 crc,U8 c)
{
	UN32INT un32DataTmp;

	un32DataTmp.u32Val = 0;
	un32DataTmp.u8Val[0] = c;
	if(false == crc_tab32_init)
	{
		_init_crc32_tab();
	}
	un32DataTmp.u32Val ^= crc;
	crc = (crc >> 8) ^ crc_tab32[un32DataTmp.u8Val[0]];
	return(crc);
}


U16 crc16(U8 * Data,U32 Len)
{
    U32 i;
	U16 crc;
	for(i = 0,crc = 0;i < Len;i++)
	{
		crc = _crc_16(crc,Data[i]);
	}
	return(crc);
}

//MODBUS CRC16校验
U16 crc16_modbus(U8 * Data ,U32 Len)
{
    U32 i;
	U16 crc;
	for(i = 0,crc = 0xFFFF;i < Len;i++)
	{
		crc = _crc_16(crc,Data[i]);
	}
	return(crc);
}


//DNP CRC16校验
U16 crc16_dnp(U8 * Data ,U32 Len)
{
    U32 i;
	UN16INT un16CRCTmp;
	UN16INT un16ReVal;
	
	for(i = 0,un16CRCTmp.u16Val = 0;i < Len;i++)
	{
		un16CRCTmp.u16Val = _crc_dnp(un16CRCTmp.u16Val,Data[i]);
	}
	un16CRCTmp.u16Val = ~un16CRCTmp.u16Val;
	un16ReVal.u8Val[0] = un16CRCTmp.u8Val[1];
	un16ReVal.u8Val[1] = un16CRCTmp.u8Val[0];
	return(un16ReVal.u16Val);
}

//SICK CRC-16
U16 crc16_sick(U8 * Data ,U32 Len)
{
    U32 i;
	U16 prev_byte;
	UN16INT un16CRCTmp;
	UN16INT un16ReVal;
	
	for(i = 0,un16CRCTmp.u16Val = 0,prev_byte = 0;i < Len;i++)
	{
		un16CRCTmp.u16Val = _crc_sick(un16CRCTmp.u16Val,Data[i],prev_byte);
		prev_byte = Data[i];
	}
	
    un16ReVal.u8Val[0] = un16CRCTmp.u8Val[1];
	un16ReVal.u8Val[1] = un16CRCTmp.u8Val[0];
	return(un16ReVal.u16Val);
}

// XMODEM CRC-16
U16 crc16_ccitt_xmodem(U8 * Data ,U32 Len)
{
    U32 i;
	U16 crc;
	for(i = 0,crc = 0;i < Len;i++)
	{
		crc = _crc_ccitt(crc,Data[i]);
	}
	
	return(crc);
}

//初始全1的校验
U16 crc16_ccitt_ffff(U8 * Data ,U32 Len)
{
    U32 i;
	U16 crc;
	for(i = 0,crc = 0xFFFF;i < Len;i++)
	{
		crc = _crc_ccitt(crc,Data[i]);
	}
	
	return(crc);
}



U16 crc16_ccitt_1d0f(U8 * Data ,U32 Len)
{
    U32 i;
	U16 crc;
	for(i = 0,crc = 0x1D0F;i < Len;i++)
	{
		crc = _crc_ccitt(crc,Data[i]);
	}
	
	return(crc);
}

U16 crc16_ccitt_kermit(U8 * Data ,U32 Len)
{
    U32 i;	
	UN16INT un16CRCTmp;
	UN16INT un16ReVal;
	
	for(i = 0,un16CRCTmp.u16Val = 0;i < Len;i++)
	{
		un16CRCTmp.u16Val = _crc_kermit(un16CRCTmp.u16Val,Data[i]);
	}
	
	un16ReVal.u8Val[0] = un16CRCTmp.u8Val[1];
	un16ReVal.u8Val[1] = un16CRCTmp.u8Val[0];
	return(un16ReVal.u16Val);
}

//CRC-32校验
U32 crc32(U8 * Data ,U32 Len)
{
    U32 i;
	U32 crc;
	for(i = 0,crc = 0xFFFFFFFF;i < Len;i++)
	{
		crc = _crc_32(crc,Data[i]);
	}
	
	crc ^= 0xFFFFFFFF;
	return(crc);
}

/*eof*/

