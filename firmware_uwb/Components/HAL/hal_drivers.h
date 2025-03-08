/*
 ********************************************************************************
 *
 *                                 hal_drivers.h
 *
 * File          : hal_drivers.h
 * Version       : V1.0
 * Author        : tony
 * Mode          : Thumb2
 * Toolchain     : 
 * Description   : 开发板硬件驱动 
 *                
 * History       :
 * Date          : 2013.07.22
 *******************************************************************************/
 
#ifndef __HAL_DRIVERS_H
#define __HAL_DRIVERS_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 * 																				INCLUDES
 **************************************************************************************************/	
#include "OSAL_Comdef.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_timer.h"
#include "hal_usart.h"
#include "hal_flash.h"
#include "hal_rtc.h"
#include "hal_adc.h"
#include "hal_usb.h"
#include "hal_spi.h"
#include "hal_pmu.h"
	
	
		
/**************************************************************************************************
 * 																				CONSTANTS
 **************************************************************************************************/	
#define HAL_LED 	TRUE
#define HAL_TIMER TRUE
#define HAL_KEY		TRUE//TRUE	
#define HAL_USART TRUE//TRUE	
#define HAL_FLASH TRUE//TRUE
#define HAL_RTC		FALSE//FALSE
#define HAL_SPI		TRUE
#define HAL_USB 	TRUE

#ifdef HW_BASE
	#define HAL_ADC		FALSE
	#define HAL_PMU		FALSE
#else
	#define HAL_ADC		FALSE
	#define HAL_PMU		FALSE
#endif


#define HAL_NVIC_GROUP_SN		NVIC_PriorityGroup_4
#define Hal_NVIC_Init(x)		NVIC_PriorityGroupConfig(x);

/***************************************************************************************************
 * 																				TYPEDEF
 ***************************************************************************************************/


/***************************************************************************************************
 * 																				GLOBAL VARIABLES
 ***************************************************************************************************/
	
	
	
/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/	
	
extern void Hal_Init ( uint8_t task_id );

/*
 * Initialize HW
 */
extern void Hal_Driver_Init (void);

#ifdef __cplusplus
}
#endif

#endif//__HAL_DRIVERS_H

/**************************************************************************************************
**************************************************************************************************/
