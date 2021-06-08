/******************************************************************************

                  版权所有 (C), 2015-2020, Winlab

 ******************************************************************************
  文 件 名   : gpio_drv.h
  版 本 号   : 初稿
  作    者   : Winlab
  生成日期   : 2019年6月22日
  最近修改   :
  功能描述   : GPIO头文件
  函数列表   :
  修改历史   :
  1.日    期   : 2019年6月22日
    作    者   : Winlab
    修改内容   : 创建文件

******************************************************************************/


#ifndef _GPIO_DRV_H_
#define _GPIO_DRV_H_

#include "pubdef.h"

typedef enum
{
    GPIO_1 = 0,
    GPIO_2,
    GPIO_3,
    GPIO_4,
    GPIO_5,
    GPIO_6,
    GPIO_7,
    GPIO_8,
    GPIO_9,
    GPIO_10,
    GPIO_11,
    GPIO_12,
    GPIO_13,//LED1
    GPIO_14,//LED10
    GPIO_15,//LED11
    GPIO_16,//LED12
    GPIO_17,//LED15
    GPIO_18,//LED9
    GPIO_NUM,
}gpio_type_num;

//#define GPIO_SZ157_S0	GPIO_3	//GPIO映射
//#define GPIO_SZ157_S1	GPIO_2
//#define GPIO_SZ157_S2	GPIO_1
//#define GPIO_TEST		GPIO_4
#define GPIO_PHY_RESET		  GPIO_1
#define GPIO_ISM330_SS		  GPIO_2
#define GPIO_UBX_RST		  GPIO_3
#define GPIO_PANT			  GPIO_4
#define GPIO_UBX_1PPS		  GPIO_5
#define GPIO_FLASH_SS		  GPIO_6
#define GPIO_SIM_RST_PWR	  GPIO_7
#define GPIO_SIM_DEF_STAU	  GPIO_8
#define GPIO_ISM330_INT1      GPIO_9
#define GPIO_ISM330_INT2      GPIO_10
#define GPIO_IIS2MDC_INTDRDY  GPIO_11
#define GPIO_UWB_IRQ          GPIO_12
//#define GPIO_UWB_RESET        GPIO_13
#define GPIO_AMP_CTRL		  0

#define GPIO_ENABLE_AMP(x)	{gpio_drv_set(GPIO_AMP_CTRL, x); U16 temp = fmc_read_reg(FMC_BASE_SYS, BASE_SYS_RESERVE);\
								if(x){temp |= (0x02);}else{temp &= (~0x02);}\
                                temp &= (~0x0400);\
								fmc_write_reg(FMC_BASE_SYS, BASE_SYS_RESERVE, temp)};

//管控模式
#define GPIO_ENABLE_MANCTRL(x)	{U16 temp = fmc_read_reg(FMC_BASE_SYS, BASE_SYS_RESERVE);\
                                if(x){temp |= (0x01);}else{temp &= (~0x01);}\
                                temp &= (~0x0400);\
                                fmc_write_reg(FMC_BASE_SYS, BASE_SYS_RESERVE, temp)};

                                

//fmc_write_reg(FMC_BASE_SYS, BASE_SYS_RESERVE, (x << 1));}

/*GPIO驱动数据结构*/
typedef struct
{
    GPIO_TypeDef*		gpio_port;				/* GPIO端口 */
    U32					gpio_pin;				/* GPIO PIN */
    U32					gpio_mode;				/* 模式设置 */

    U32					irq_enable;			    /* 中断使能 */
    IRQn_Type			irqn;					/* 中断号 */
    U32					prio;					/* 抢占优先级 */
    U32					sub_prio;				/* 子优先级 */
}gpio_drv_t;

extern void gpio_drv_set(gpio_type_num gpio_type, U8 val);
extern U8 gpio_drv_read(gpio_type_num gpio_type);
extern void gpio_drv_init(void);
extern void gpio_enable_irq(void);
extern void gpio_turn_led(GPIO_TypeDef* port, U32 pin, uint8_t mode);

#endif
/*eof*/

