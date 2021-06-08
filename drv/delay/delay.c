#include "delay.h"
#include "drv/uart/uart_drv.h"


#include "cmsis_os2.h"
//延时nus
//nus为要延时的us数.		    								   
void delay_us(u32 nus)
{
  u32 start_count;
	u32 end_count;
	u32 count;
  
  u32 timeout_count = nus * (osKernelGetSysTimerFreq() / 1000000u);
  
  start_count = osKernelGetSysTimerCount();             // get start value of the Kernel system tick

	while(1)
	{
		end_count = osKernelGetSysTimerCount();
		if(end_count >= start_count)
		{
			count = end_count - start_count;
		}
		else
		{
			count = (0xffffffff - start_count) + end_count + 1;
		}
			
		if(count > timeout_count)
		{
			break;
		}
	}

}


void delay_ms_x(u32 nms)
{	 	
	u32 nus;
	nus = nms*10;
	osDelay(nus);		
} 


u32 get_cur_time(void)
{
	return osKernelGetSysTimerCount();

}


u32 get_count_time(u32 start_count, u32 end_count)
{

	u32 nus;
	u32 count;
	float freq;
	

	if(end_count >= start_count)
		count = end_count - start_count;
	else
		count = (0xffffffff - start_count) + end_count + 1;

	freq = osKernelGetSysTimerFreq();
	nus = (1000000u/freq)*count;

	return nus;
}



void test_time(void)
{
	u32 start;
	u32 end;
	u32 nus;

	start = get_cur_time();

	while(1)
	{
		end = get_cur_time();
		nus = get_count_time(start, end);
		if(nus >= 5000000u)
		{
			//printf("time is %d us\r\n", nus);
			uart_send_data(0, "111111111\r\n", strlen("111111111\r\n"));
			start = get_cur_time();			
		}			

	}
}
































