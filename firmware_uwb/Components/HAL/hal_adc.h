#ifndef __HAL_ADC_H
#define __HAL_ADC_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 * 																				INCLUDES
 **************************************************************************************************/
#include "stm32f10x.h"	
#include "OSAL_Comdef.h"

	
/**************************************************************************************************
 * 																				CONSTANTS
 **************************************************************************************************/			
#define HAL_LOW_BATTERY			3.50

//ADC¶Ë¿Ú¶¨Òå
#define HAL_ADC_RCC     		RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC1                            
#define HAL_ADC_PORT				GPIOB    
#define HAL_ADC1_PIN        GPIO_Pin_1  
#define HAL_ADC1_SN					ADC1	

#define HAL_ADC1_MAX_CH			1
#define HAL_ADC1CH					ADC_Channel_9
	
/***************************************************************************************************
 * 																				TYPEDEF
 ***************************************************************************************************/
enum Hal_ADC_Battery{
	Hal_ADC_ADC_Count_Init = 0,
	Hal_ADC_ADC_Count_MAX = 3//200
};

enum Hal_Battery_State{
	Hal_Battery_State_Invaild = 0,
	Hal_Battery_State_Low,
	Hal_Battery_State_OK,
	Hal_Battery_State_Full
};


/***************************************************************************************************
 * 																				GLOBAL VARIABLES
 ***************************************************************************************************/



/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/	

static void HalAdc_IO_Init(void);

static void HalAdc_Init(void);

static void HalAdcenable(void);

static void HalAdcDisable(void);



/*
 * Initialize adc Service.
 */
extern void HalAdcInit( void );



extern uint16_t HalAdcValueGet(uint8_t ch);  


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif
#endif//__HAL_ADC_H
