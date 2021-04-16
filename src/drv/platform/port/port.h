/*! ----------------------------------------------------------------------------
 * @file	port.h
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <string.h>
#include <stdio.h>
//#include "stm32f40x.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include "platform/deca/deca_sleep.h"
#include "meas/meas_distance.h"
#include "pubDef.h"

/* Define our wanted value of CLOCKS_PER_SEC so that we have a millisecond
 * tick timer. */
#undef CLOCKS_PER_SEC
#define CLOCKS_PER_SEC 1000

void printf2(const char *format, ...);
	
void USART_puts(uint8_t *s,uint8_t len);
	
extern int writetospi_serial(uint16_t headerLength,
                             const uint8_t *headerBuffer,
                             uint32_t bodylength,
                             const uint8_t *bodyBuffer);

extern int readfromspi_serial(uint16_t	headerLength,
                              const uint8_t *headerBuffer,
                              uint32_t readlength,
                              uint8_t *readBuffer );

#define writetospi  writetospi_serial
#define readfromspi readfromspi_serial

typedef enum
{
    LED_PC6,
    LED_PC7,
    LED_PC8,
    LED_PC9,
    LED_ALL,
    LEDn
} led_t;

extern uint8_t SW1;  
extern uint8_t SW2;
extern uint8_t SW3;	
extern uint8_t SW4;
extern uint8_t SW5;	
extern uint8_t SW6;
extern uint8_t SW7;	
extern uint8_t SW8;	

#define SPIy_PRESCALER				SPI_BaudRatePrescaler_128

#define SPIy						SPI2
#define SPIy_GPIO					GPIOB
#define SPIy_CS						GPIO_Pin_12
#define SPIy_CS_GPIO				GPIOB
#define SPIy_SCK					GPIO_Pin_13
#define SPIy_MISO					GPIO_Pin_14
#define SPIy_MOSI					GPIO_Pin_15


#define SPIx_PRESCALER				SPI_BaudRatePrescaler_8

#define SPIx						SPI1
#define SPIx_GPIO					GPIOA
#define SPIx_CS						GPIO_Pin_4
#define SPIx_CS_GPIO				GPIOA
#define SPIx_SCK					GPIO_Pin_5
#define SPIx_MISO					GPIO_Pin_6
#define SPIx_MOSI					GPIO_Pin_7

#define DW1000_RSTn					GPIO_Pin_2
#define DW1000_RSTn_GPIO			GPIOA

#define DECARSTIRQ                  GPIO_Pin_3
#define DECARSTIRQ_GPIO             GPIOA
#define DECARSTIRQ_EXTI             EXTI_Line3
#define DECARSTIRQ_EXTI_PORT        GPIO_PortSourceGPIOA
#define DECARSTIRQ_EXTI_PIN         GPIO_PinSource3
#define DECARSTIRQ_EXTI_IRQn        EXTI3_IRQn

#define DECAIRQ                     GPIO_Pin_3
#define DECAIRQ_GPIO                GPIOA
#define DECAIRQ_EXTI                EXTI_Line3
#define DECAIRQ_EXTI_PORT           GPIO_PortSourceGPIOA
#define DECAIRQ_EXTI_PIN            GPIO_PinSource3
#define DECAIRQ_EXTI_IRQn           EXTI3_IRQn
#define DECAIRQ_EXTI_USEIRQ         ENABLE
#define DECAIRQ_EXTI_NOIRQ          DISABLE

#define TA_BOOT1                 	GPIO_Pin_14
#define TA_BOOT1_GPIO            	GPIOB

#define LED                 	GPIO_Pin_1
#define LED_GPIO            	GPIOB
      
#define KEY_MODE                 	GPIO_Pin_14
#define KEY_MODE_GPIO            	GPIOB

#define KEY_SET                 	GPIO_Pin_15
#define KEY_SET_GPIO            	GPIOB

#define Read_KEY_MODE() GPIO_ReadInputDataBit(KEY_MODE_GPIO,KEY_MODE)
#define Read_KEY_SET() GPIO_ReadInputDataBit(KEY_SET_GPIO,KEY_SET)


#define EXTON                	GPIO_Pin_0
#define EXTON_GPIO            	GPIOA

#define EXTON_close() GPIO_ResetBits(EXTON_GPIO,EXTON)
#define EXTON_open() GPIO_SetBits(EXTON_GPIO,EXTON)

//when switch (S1) is 'on' the pin is low
int is_switch_on(uint16_t GPIOpin);

#define port_IS_TAG_pressed()		is_switch_on(TA_SW1_4)
#define port_IS_LONGDLY_pressed()	is_dlybutton_low()

#define USARTx						USART2

#define port_USARTx_busy_sending()	0 //(USART_GetFlagStatus((USARTx),(USART_FLAG_TXE))==(RESET))
#define port_USARTx_no_data()		0 //(USART_GetFlagStatus((USARTx),(USART_FLAG_RXNE))==(RESET))
#define port_USARTx_send_data(x)	  //USART_SendData((USARTx),(uint8_t)(x))
#define port_USARTx_receive_data()	0 //USART_ReceiveData(USARTx)

#define port_SPIx_busy_sending()		(SPI_I2S_GetFlagStatus((SPIx),(SPI_I2S_FLAG_TXE))==(RESET))
#define port_SPIx_no_data()				(SPI_I2S_GetFlagStatus((SPIx),(SPI_I2S_FLAG_RXNE))==(RESET))
#define port_SPIx_send_data(x)			SPI_I2S_SendData((SPIx),(x))
#define port_SPIx_receive_data()		SPI_I2S_ReceiveData(SPIx)
#define port_SPIx_disable()				SPI_Cmd(SPIx,DISABLE)
#define port_SPIx_enable()              SPI_Cmd(SPIx,ENABLE)
#define port_SPIx_set_chip_select()		GPIO_SetBits(SPIx_CS_GPIO,SPIx_CS)
#define port_SPIx_clear_chip_select()	GPIO_ResetBits(SPIx_CS_GPIO,SPIx_CS)

#define port_SPIy_busy_sending()		(SPI_I2S_GetFlagStatus((SPIy),(SPI_I2S_FLAG_TXE))==(RESET))
#define port_SPIy_no_data()				(SPI_I2S_GetFlagStatus((SPIy),(SPI_I2S_FLAG_RXNE))==(RESET))
#define port_SPIy_send_data(x)			SPI_I2S_SendData((SPIy),(x))
#define port_SPIy_receive_data()		SPI_I2S_ReceiveData(SPIy)
#define port_SPIy_disable()				SPI_Cmd(SPIy,DISABLE)
#define port_SPIy_enable()              SPI_Cmd(SPIy,ENABLE)
#define port_SPIy_set_chip_select()		GPIO_SetBits(SPIy_CS_GPIO,SPIy_CS)
#define port_SPIy_clear_chip_select()	GPIO_ResetBits(SPIy_CS_GPIO,SPIy_CS)
#define port_LCD_RS_set()				GPIO_SetBits(SPIy_GPIO,LCD_RS)
#define port_LCD_RS_clear()				GPIO_ResetBits(SPIy_GPIO,LCD_RS)
#define port_LCD_RW_set()				GPIO_SetBits(SPIy_GPIO,LCD_RW)
#define port_LCD_RW_clear()				GPIO_ResetBits(SPIy_GPIO,LCD_RW)

#define port_GET_stack_pointer()		__get_MSP()
#define port_GET_rtc_time()				RTC_GetCounter()
#define port_SET_rtc_time(x)			RTC_SetCounter(x)

ITStatus EXTI_GetITEnStatus(uint32_t x);

#define port_GetEXT_IRQStatus()             EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn)
#define port_DisableEXT_IRQ()               NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn)
#define port_EnableEXT_IRQ()                NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn)
#define port_CheckEXT_IRQ()                 GPIO_ReadInputDataBit(DECAIRQ_GPIO, DECAIRQ)
int NVIC_DisableDECAIRQ(void);

//define LCD functions
#define port_LCD_Clear(x) 											0
#define port_LCD_SetBackColor(x) 									0
#define port_LCD_SetTextColor(x) 									0
#define port_LCD_DisplayString(line, column, font_index, buffer) 	0
int is_IRQ_enabled(void);

int is_button_low(uint16_t GPIOpin);

#define is_button_high(x)			0
void led_on(led_t led);
void led_off(led_t led);
#define gpio_set(x)				0
#define gpio_reset(x)				0
#define is_gpio_out_low(x)			0
#define is_gpio_out_high(x)			0
			
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn peripherals_init()
 *
 * @brief Initialise all peripherals.
 *
 * @param none
 *
 * @return none
 */
void peripherals_init (void);

void SPI_ChangeRate(uint16_t scalingfactor);
void SPI_ConfigFastRate(uint16_t scalingfactor);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_low()
 *
 * @brief Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
 *
 * @param none
 *
 * @return none
 */
extern void spi_set_rate_low (void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_high()
 *
 * @brief Set SPI rate as close to 20 MHz as possible for optimum performances.
 *
 * @param none
 *
 * @return none
 */
extern void spi_set_rate_high (void);

unsigned long portGetTickCnt(void);

#define portGetTickCount() 			portGetTickCnt()

extern void reset_DW1000(void);
extern void setup_DW1000RSTnIRQ(int enable);
extern double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,u8 db);
extern void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
extern void final_msg_get_dist(const uint8 *ts_field, uint32 *dist);
extern void GPIO_Toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
extern int LP(int tmp,uint8_t channel);
extern uint16_t Checksum_u16(uint8_t* pdata, uint32_t len) ;

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
