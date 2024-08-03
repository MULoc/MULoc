#ifndef __HAL_LED_H
#define __HAL_LED_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 * 																				INCLUDES
 **************************************************************************************************/
#include "stm32f10x.h"	
#include "string.h"	
#include "OSAL_Comdef.h"
/**************************************************************************************************
 * 																				CONSTANTS
 **************************************************************************************************/		
	
#ifdef HW_BASE
	//LED端口定义
	#define HAL_LED_RCC     		RCC_APB2Periph_GPIOA                           
	#define HAL_LED_PORT				GPIOA    
	#define HAL_LED1_PIN        GPIO_Pin_1    //LED_RUN
	#define HAL_LED2_PIN        NULL    
	#define HAL_LED3_PIN        NULL    
	#define HAL_LED4_PIN        NULL
		
	#define HAL_LED_POWER				NULL

	//BEEP端口定义
	#define HAL_BEEP_RCC     		NULL                           
	#define HAL_BEEP_PORT				NULL
	#define HAL_BEEP_PIN        NULL   
#else

	#define HAL_LED_RCC     		RCC_APB2Periph_GPIOB                           
	#define HAL_LED_PORT				GPIOB    
	#define HAL_LED1_PIN        GPIO_Pin_6    //LED RUN
	#define HAL_LED2_PIN        GPIO_Pin_7    //LED POWER
	#define HAL_LED3_PIN				NULL					//SOS
	#define HAL_LED4_PIN        NULL
	
	#define HAL_LED_POWER				GPIO_Pin_2		//复用boot1


	//BEEP端口定义
	#define HAL_BEEP_RCC     		RCC_APB2Periph_GPIOA//RCC_APB2Periph_GPIOA                            
	#define HAL_BEEP_PORT				GPIOA//GPIOA
	#define HAL_BEEP_PIN        GPIO_Pin_3//GPIO_Pin_3   
#endif
/***************************************************************************************************
 * 																				TYPEDEF
 ***************************************************************************************************/
typedef enum {
    HAL_LED1 = 0,			//RED
    HAL_LED2,					//GREEN
    HAL_LED3,
    HAL_LED4,
		HAL_LED_ALL
}HAL_LED_SN;

typedef enum {
		HAL_LED_MODE_ON = 0,
		HAL_LED_MODE_OFF = 1,
		HAL_LED_MODE_TOGGLE
}HAL_LED_MODE;

typedef enum {
    HAL_BEEP1 = 1,
		HAL_BEEP_ALL
}HAL_BEEP_SN;

typedef enum {
		HAL_BEEP_MODE_ON = 0,
		HAL_BEEP_MODE_OFF,
}HAL_BEEP_MODE;

typedef struct{
		uint8_t mode;				//工作状态 0：ON 	1：OFF 	2：TOGGLE
		uint16_t period;		//周期
}hal_led_t;

/***************************************************************************************************
 * 																				GLOBAL VARIABLES
 ***************************************************************************************************/

extern hal_led_t hal_led[HAL_LED_ALL];


/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/	

/*
 * Initialize LED Service.
 */
extern void HalLedInit( void );

/*
 * Set the LED ON/OFF/TOGGLE.
 */
extern void HalLedSet (HAL_LED_SN leds, HAL_LED_MODE mode);

/*
 * Set the BEEP ON/OFF/TOGGLE.
 */
extern void HaBeepSet (HAL_BEEP_SN beeps, HAL_BEEP_MODE mode);


extern int HalLed_Mode_Set(HAL_LED_SN led, HAL_LED_MODE mode, uint16_t period);

extern void HalDevice_On (void);

extern void HalDevice_Off (void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif
#endif
