#include <string.h>
#include <stdio.h>
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include "platform/deca/deca_sleep.h"
#include "platform/port/port.h"
//#include "usart.h"
#include "meas/meas_distance.h"
#include "stmflash.h"

//#include "stm32f4xx_hal_flash.h"
//#include "stm32f4xx_hal_flash_ex.h"
#include "stm32h7xx_hal_flash.h"

#include "crc/crc.h"




#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08100000) /* Base @ of Sector 0, 16 Kbyte */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08120000) /* Base @ of Sector 1, 16 Kbyte */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 16 Kbyte */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x08160000) /* Base @ of Sector 3, 16 Kbyte */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08180000) /* Base @ of Sector 4, 64 Kbyte */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbyte */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbyte */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbyte */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x081FFFFF) /* Base @ of Sector 8, 128 Kbyte */
//#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbyte */
//#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbyte */
//#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbyte */


//#define FLASH_SAVE_ADDR  0X0800FB38 				//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
#define FLASH_SAVE_ADDR  ADDR_FLASH_SECTOR_6				//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)


//////////////////////////////////////////////////////////////////////////////////




/**
  * @brief  Gets the sector of a given address
  * @param  Address: Flash address
  * @retval The sector of a given address
  */
static uint32_t get_sector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;  
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;  
  }
  /*
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;  
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;  
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;  
  }
  else
  {
    sector = FLASH_SECTOR_11;  
  }
 */
  return sector;
}



//flash的初始化，解锁flash和清除一些flash的异常状态标识
uint16_t init_fs(void)
{
 
    HAL_FLASH_Unlock(); 
	//HAL_FLASH_OB_Unlock();
  //  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
  //                        FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
 return 0;
}


uint16_t de_init_fs(void) 
{
    HAL_FLASH_Lock();
	//HAL_FLASH_OB_Lock();
	return 0;
}


//flash 的檫除操作
uint16_t erase_fs(uint32_t start_Add,uint32_t end_Add)
{
    /* USER CODE BEGIN 3 */
    uint32_t UserStartSector;
    uint32_t SectorError;
    FLASH_EraseInitTypeDef pEraseInit;
 
    /* Unlock the Flash to enable the flash control register access *************/
    init_fs();
 	GLOBAL_PRINT(("step 3\r\n"));
    /* Get the sector where start the user flash area */
    UserStartSector = get_sector(start_Add);
 
    pEraseInit.TypeErase = TYPEERASE_SECTORS;
	pEraseInit.Banks = FLASH_BANK_2;
    pEraseInit.Sector = UserStartSector;
    pEraseInit.NbSectors = get_sector(end_Add)-UserStartSector+1 ;
    pEraseInit.VoltageRange = VOLTAGE_RANGE_3;

 
    if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK)
    {
    		de_init_fs();
        /* Error occurred while page erase */
        return (1);
    }
	GLOBAL_PRINT(("step 4\r\n"));

		de_init_fs();
    return (HAL_OK);
    /* USER CODE END 3 */
}



//flash写入操作
static uint16_t write_fs(u8 *src, uint32_t dest, uint32_t Len)
//uint16_t MEM_If_Write_FS(uint8_t *data,uint32_t len,uint32_t address)

{
    /* USER CODE BEGIN 3 */
    uint32_t i = 0;
	HAL_StatusTypeDef FlashStatus=HAL_OK;

	init_fs();
	GLOBAL_PRINT(("step 5\r\n"));

	FlashStatus = FLASH_WaitForLastOperation(FLASH_WAITETIME*10,FLASH_BANK_2);
	
 	GLOBAL_PRINT(("step 6\r\n"));
    //for(i = 0; i < Len; i++)
    //{
        /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
           be done by byte */
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (dest+i), (uint64_t)(src+i)) == HAL_OK)
        {
        	FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME*10,FLASH_BANK_2);
            /* Check the written value */
            if(*(uint8_t *)(src + i) != *(uint8_t*)(dest+i))
            {
            	
				
        		de_init_fs();
				GLOBAL_PRINT(("step 7\r\n"));
            	/* Flash content doesn't match SRAM content */
                return 2;
            }
			GLOBAL_PRINT(("step 8\r\n"));
        }
        else
        {
        	FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME*10,FLASH_BANK_2);
        	GLOBAL_PRINT(("step 8\r\n"));
        	de_init_fs();
            /* Error occurred while writing data in Flash memory */
            return 1;
        }
   // }

		de_init_fs();
    return (HAL_OK);
    /* USER CODE END 3 */
}




//flash读出操作
static uint8_t read_fs(uint8_t *src, uint8_t *dest, uint32_t Len)
{
    /* Return a valid address to avoid HardFault */
    /* USER CODE BEGIN 4 */
	  uint32_t i = 0;
    uint8_t *psrc = src;
 
    for(i = 0; i < Len; i++)
    {
        dest[i] = *psrc++;
    }
    return HAL_OK;
 
    /* USER CODE END 4 */
}



void flash_write(u8 *buf, u16 len)
{
	erase_fs(FLASH_SAVE_ADDR,ADDR_FLASH_SECTOR_7);
	write_fs(buf, FLASH_SAVE_ADDR, len);	
}



void flash_read(u8 *buf, u16 len)
{
	read_fs((u8*)FLASH_SAVE_ADDR, buf, len);	
}







