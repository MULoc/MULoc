#ifndef __HAL_UART_H
#define __HAL_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

	
/**************************************************************************************************
 * 																				INCLUDES
 **************************************************************************************************/	
#include "stdio.h" 
#include "OSAL_Comdef.h"
#include "stm32f10x.h"	

	
	
/**************************************************************************************************
 * 																				CONSTANTS
 **************************************************************************************************/		
#define HAL_USART1					/*调试口*/
#define xHAL_USART2					/*待使用*/				
#define xHAL_USART3					/*uwb数据解析*/

#define HAL_USART1_DMA
#define xHAL_USART2_DMA



	
//LED端口定义
#define HAL_USART1_RCC     			RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA             	
#define HAL_USART1_PORT					GPIOA    
#define HAL_USART1_TXD_PIN    	GPIO_Pin_9    
#define HAL_USART1_RXD_PIN    	GPIO_Pin_10    
#define HAL_USART1_SN						USART1



/***************************************************************************************************
 * 																				TYPEDEF
 ***************************************************************************************************/
typedef void (*halUARTCBack_t) (uint8_t port, uint8_t event);


/***************************************************************************************************
 * 																				GLOBAL VARIABLES
 ***************************************************************************************************/
extern char USART1_DMA_RX_BYTE;
extern char USART2_DMA_RX_BYTE;


/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/	
	extern void _dbg_printf(const char *format,...);

	extern uint16_t USART1_SendBuffer(const char* buffer, uint16_t length, int flag);

	extern uint16_t USART2_SendBuffer(const char* buffer, uint16_t length);

	extern void HalUARTInit ( void ); 
	
	static void HalUASRT1_NVIC_Config(void);
	
	static void HalUASRT2_NVIC_Config(void);
	
	static void HalUASRT3_NVIC_Config(void);
	
	static void HalUSART1_Init(u32 bound);
	
	static void HalUSART2_Init(u32 bound);
	
	static void HalUSART3_Init(u32 bound);
	
	static void HalUSART1_IO_Init(void);
	
	static void HalUSART2_IO_Init(void);
	
	static void HalUSART3_IO_Init(void);


#ifdef __cplusplus
}
#endif

#endif	//__HAL_UART_H
