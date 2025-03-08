#ifndef __HAL_KEY_H
#define __HAL_KEY_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 * 																				INCLUDES
 **************************************************************************************************/
#include <stdbool.h>
#include "stm32f10x.h"	
#include "OSAL_Comdef.h"

	
/**************************************************************************************************
 * 																				CONSTANTS
 **************************************************************************************************/		
#ifdef HW_BASE
	#define xHAL_KEY_POWER_ISR
	#define xHAL_KEY_ACT_ISR
#else
	#define HAL_KEY_POWER_ISR
	#define HAL_KEY_ACT_ISR
#endif
	
//是否电源输入
#define HAL_KEY1_RCC 						RCC_APB2Periph_GPIOA													
#define HAL_KEY1_PORT						GPIOA 	
#define HAL_KEY1_PIN						GPIO_Pin_1		
#define HAL_KEY1_PORT_SOURCE		GPIO_PortSourceGPIOA
#define HAL_KEY1_PIN_SOURCE 		GPIO_PinSource1
#define HAL_KEY1_EXTI_LIN 			EXTI_Line1
#define HAL_KEY1_EXIT_IRQN			EXTI1_IRQn

#define HAL_KEY2_RCC						RCC_APB2Periph_GPIOA													
#define HAL_KEY2_PORT 					GPIOA 
#define HAL_KEY2_PIN 						GPIO_Pin_2		
#define HAL_KEY2_PORT_SOURCE 		GPIO_PortSourceGPIOA
#define HAL_KEY2_PIN_SOURCE			GPIO_PinSource2
#define HAL_KEY2_EXTI_LIN				EXTI_Line2
#define HAL_KEY2_EXIT_IRQN 			EXTI2_IRQn


#define HAL_KEY3_RCC 						RCC_APB2Periph_GPIOB													
#define HAL_KEY3_PORT						GPIOB 	
#define HAL_KEY3_PIN 						GPIO_Pin_3	
#define HAL_KEY3_PORT_SOURCE 		GPIO_PortSourceGPIOB
#define HAL_KEY3_PIN_SOURCE			GPIO_PinSource3
#define HAL_KEY3_EXTI_LIN				EXTI_Line3
#define HAL_KEY3_EXIT_IRQN 			EXTI3_IRQn	

#ifdef HW_RTLS
#define HAL_KEY4_RCC 						RCC_APB2Periph_GPIOA													
#define HAL_KEY4_PORT						GPIOA 	
#define HAL_KEY4_PIN 						GPIO_Pin_9	
#define HAL_KEY4_PORT_SOURCE 		GPIO_PortSourceGPIOA
#define HAL_KEY4_PIN_SOURCE			GPIO_PinSource9
#define HAL_KEY4_EXTI_LIN				EXTI_Line9
#define HAL_KEY4_EXIT_IRQN 			EXTI9_5_IRQn	
#endif

/***************************************************************************************************
 * 																				TYPEDEF
 ***************************************************************************************************/
typedef void (*halKeyCBack_t) (uint8_t keys, uint8_t state);	


/***************************************************************************************************
 * 																				GLOBAL VARIABLES
 ***************************************************************************************************/
	
	
	
/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/	

#ifdef HAL_KEY_POWER_ISR
static void HalExitInit(void);
#endif

/*
 * Initialize the Key Service
 */
extern void HalKeyInit( void );

/*
 * Configure the Key Service
 */
extern void HalKeyConfig( bool interruptEnable, const halKeyCBack_t cback);

/*
 * Read the Key status
 */
extern uint8_t HalKeyRead( void);



/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif
#endif
