#include <math.h>
#include "MT.h"
#include "hal_drivers.h"
#include "Generic.h"

int mt_spi_exit = 0;
void MT_Components_Event(void)
{

#if (MT_DEBUG==TRUE)	
	
#if ((defined HAL_ADC_DEBUG) && (HAL_ADC_DEBUG == TRUE))

		float uwb_vol = (float)HalAdcValueGet(HAL_ADC1CH) / 4096 * 3.3 * 2;  //读取并返回ADC对应通道的AD转换值 		
		
		_dbg_printf("This is HalADC test device battery is: %d %.2fV\n", HalAdcValueGet(HAL_ADC1CH), uwb_vol);
#endif

#if (defined HAL_USART_DEBUG) && (HAL_USART_DEBUG == TRUE)
		_dbg_printf("This is HalUsart test\n");		
#endif		
	
#if (defined HAL_LED_DEBUG) && (HAL_LED_DEBUG == TRUE)	
		HalLedSet(HAL_LED_ALL, HAL_LED_MODE_ON);		    //点亮LED
		HaBeepSet(HAL_BEEP_ALL,HAL_BEEP_MODE_ON);
		HalDelay_nMs(1000);	//延时500ms
		HalLedSet(HAL_LED_ALL, HAL_LED_MODE_OFF);		    //点亮LED
		HaBeepSet(HAL_BEEP_ALL,HAL_LED_MODE_OFF);		
		HalDelay_nMs(1000);	//延时500ms			
#endif		
  /* HID */
#if (defined HAL_USB_DEBUG) && (HAL_USB_DEBUG == TRUE)	
		HalUsbWrite("This is HalUsb test\n",strlen("This is HalUsb test\n"));
#endif	

#if ((defined HAL_SPI_LIS3DH_DEBUG) && (HAL_SPI_LIS3DH_DEBUG == TRUE))
		mt_spi_exit = 1;

		AxesRaw_t data;		
		float Xg,Yg,Zg,G; 
		if(LIS3DH_GetAccAxesRaw(&data) == MEMS_SUCCESS){
			Xg = data.AXIS_X/16384.0;
			Yg = data.AXIS_Y/16384.0;
			Zg = data.AXIS_Z/16384.0;
			G=sqrt(Xg*Xg + Yg*Yg + Zg*Zg);	
			//_dbg_printf("X=%6d Y=%6d Z=%6d \r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
			_dbg_printf("This is HalSPI Lis3dh test mg x=%8.3fg,y=%8.3fg,z=%8.3fg,G=%8.3fg\r\n",Xg,Yg,Zg,G);
		}
#endif
		App_Module_Sys_Work_Mode_Event(Sys_Operate_Mode_WORK_DONE);


		HalIWDG_Feed();
		//HalPmu_Enter();//进入停机模式（唤醒部分见中断服务函数exti.c）	
#endif
}
