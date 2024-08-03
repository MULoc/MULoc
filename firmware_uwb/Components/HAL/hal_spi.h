#ifndef __HAL_SPI_H
#define __HAL_SPI_H

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
#include "lis3dh_driver.h"

/**************************************************************************************************
 * 																				CONSTANTS
 **************************************************************************************************/		

#define HAL_SPI_DECA

#ifdef HW_BASE
	#define xHAL_SPI_LIS3DH
	#define xHAL_SPI_LCD 			
#else
	#define HAL_SPI_LIS3DH
	#define HAL_SPI_LCD
#endif

//DECAWAVE SPI端口定义
#define SPI1_PRESCALER		SPI_BaudRatePrescaler_8	
	
#define SPI1_GPIO					GPIOA
#define SPI1_CS						GPIO_Pin_4
#define SPI1_CS_GPIO			GPIOA
#define SPI1_SCK					GPIO_Pin_5
#define SPI1_MISO					GPIO_Pin_6
#define SPI1_MOSI					GPIO_Pin_7

//DECAWAVE KEY端口定义
#define DECA_RCC										RCC_APB2Periph_GPIOB
#define DECAIRQ                     GPIO_Pin_5
#define DECAIRQ_GPIO                GPIOB
#define DECAIRQ_EXTI                EXTI_Line5
#define DECAIRQ_EXTI_PORT           GPIO_PortSourceGPIOB
#define DECAIRQ_EXTI_PIN            GPIO_PinSource5
#define DECAIRQ_EXTI_IRQn           EXTI9_5_IRQn
#define DECAIRQ_EXTI_USEIRQ         ENABLE
#define DECAIRQ_EXTI_NOIRQ          DISABLE



#define SPI1_LIS3DH_GPIO					GPIO_Pin_15
#define SPI1_LIS3DH_CS_GPIO				GPIOA

#define SPI_LIS3DH_CS_LOW()   		GPIO_ResetBits(SPI1_LIS3DH_CS_GPIO,SPI1_LIS3DH_GPIO)
#define SPI_LIS3DH_CS_HIGH()   		GPIO_SetBits(SPI1_LIS3DH_CS_GPIO,SPI1_LIS3DH_GPIO) 

//LIS3DH INT1端口定义
#define LIS3DH_RCC										RCC_APB2Periph_GPIOB
#define LIS3DIRQ                   	  GPIO_Pin_4
#define LIS3DHIRQ_GPIO                GPIOB
#define LIS3DHIRQ_EXTI                EXTI_Line4
#define LIS3DHIRQ_EXTI_PORT           GPIO_PortSourceGPIOB
#define LIS3DHIRQ_EXTI_PIN            GPIO_PinSource4
#define LIS3DHIRQ_EXTI_IRQn           EXTI4_IRQn
#define LIS3DHIRQ_EXTI_USEIRQ         ENABLE
#define LIS3DHIRQ_EXTI_NOIRQ          DISABLE



/***************************************************************************************************
 * 																				TYPEDEF
 ***************************************************************************************************/


enum LIS3DH_Threshold_VAL{
	LIS3DH_Threshold_Count_Init = 0,
	LIS3DH_Threshold_Count_MAX = 10//200
};


/***************************************************************************************************
 * 																				GLOBAL VARIABLES
 ***************************************************************************************************/



/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/	

/*
 * Initialize SPI Service.
 */
extern void HalSpiInit( void );

#ifdef HAL_SPI_LIS3DH
extern uint8_t SPI1_ReadWriteByte(uint8_t TxData);
#endif
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif
#endif
