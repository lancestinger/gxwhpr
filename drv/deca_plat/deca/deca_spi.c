/*! ----------------------------------------------------------------------------
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include <string.h>

#include "deca_spi.h"
#include "deca_sleep.h"
#include "decadriver/deca_device_api.h"
#include "deca_plat/port/port.h"
#include "drv/spi/spi_drv.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal_spi.h"


#define SPI_UWB_CS_LOW		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define SPI_UWB_CS_HIGH 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)

int writetospi_serial( uint16_t headerLength,
			   	    const uint8_t *headerBuffer,
					uint32_t bodylength,
					const uint8_t *bodyBuffer
				  );

int readfromspi_serial( uint16_t	headerLength,
			    	 const uint8_t *headerBuffer,
					 uint32_t readlength,
					 uint8_t *readBuffer );
/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
	// done by port.c, default SPI used is SPI1

	return 0;

} // end openspi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
#if 0
	while (port_SPIx_busy_sending()); //wait for tx buffer to empty

	port_SPIx_disable();
#endif
	return 0;

} // end closespi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
#pragma GCC optimize ("O3")
int writetospi_serial
(
    uint16_t       headerLength,
    const uint8_t *headerBuffer,
    uint32_t       bodylength,
    const uint8_t *bodyBuffer
)
{

	int i=0;

	
	
	GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16U;
	
	//GLOBAL_PRINT(("Entered Writefromspi_serial!!\r\n"));
	
	spi_write_buffer(SPI_1, (U8*)headerBuffer, headerLength);
	
	spi_write_buffer(SPI_1, (U8*)bodyBuffer, bodylength);
	
	//GLOBAL_PRINT(("SPI1 WRITE 2 Over!!\r\n"));

	GPIOA->BSRR = GPIO_PIN_4;	
	
  return 0;
} // end writetospi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
#pragma GCC optimize ("O3")
int readfromspi_serial
(
    uint16_t       headerLength,
    const uint8_t *headerBuffer,
    uint32_t       readlength,
    uint8_t       *readBuffer
)
{

	int i=0;

	GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16U;
	
	//GLOBAL_PRINT(("Entered Readfromspi_serial!!\r\n"));
	
	spi_write_buffer(SPI_1, (U8*)headerBuffer, headerLength);
		
	//GLOBAL_PRINT(("SPI1 READ 1 Over!!\r\n"));
	
	spi_read_buffer(SPI_1, (U8*)readBuffer, readlength);
	
	//GLOBAL_PRINT(("SPI1 READ 2 Over!!\r\n"));

	GPIOA->BSRR = GPIO_PIN_4;		

    return 0;
} // end readfromspi()



