/*! ----------------------------------------------------------------------------
 * @file	port.h
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "compiler.h"
#include "stm32f10x.h"

//#define USB_SUPPORT

/*****************************************************************************************************************//*
 * To enable LDC update (i.e. printing/outputting range result to LCD set this to 1
 * NOTE: it takes 3.1 ms to output/write to LCD so this is limiting when performing fast ranging
 * (e.g. 6.8 with short slot times - wake up could be 2.25 ms, ranging exchange 1.65 ms and the LCD update 3.1 ms)
 */
#define LCD_UPDATE_ON (0)


/*****************************************************************************************************************//*
 * To enable Direct Memory Access for SPI set this option to (1)
 * This option will increase speed of spi transactions but it will use an extra RAM memory buffer
 */
#define DMA_ENABLE	(0)

/*****************************************************************************************************************//*
**/
#if (DMA_ENABLE == 1)
 #define writetospi		writetospi_dma
 #define readfromspi	readfromspi_dma
 void dma_init(void);
#else
#define writetospi		writetospi_serial
#define readfromspi	  readfromspi_serial
//extern int writetospi_serial( uint16_t headerLength,
// 			   	    const uint8_t *headerBuffer,
// 					uint32_t bodylength,
// 					const uint8_t *bodyBuffer
// 				  );

//extern int readfromspi_serial( uint16_t	headerLength,
// 			    	 const uint8_t *headerBuffer,
// 					 uint32_t readlength,
// 					 uint8_t *readBuffer );					 


#endif

typedef enum
{
    LED_PC6,
    LED_PC7,
    LED_PC8,
    LED_PC9,
    LED_ALL,
    LEDn
} led_t;

#define SPIy_PRESCALER				SPI_BaudRatePrescaler_128

#define SPIy						SPI2
#define SPIy_GPIO					GPIOB
#define SPIy_CS						GPIO_Pin_12
#define SPIy_CS_GPIO				GPIOB
#define SPIy_SCK					GPIO_Pin_13
#define SPIy_MISO					GPIO_Pin_14
#define SPIy_MOSI					GPIO_Pin_15

#define LCD_RW						GPIO_Pin_10
#define LCD_RS						GPIO_Pin_11

#define SPIx_PRESCALER				SPI_BaudRatePrescaler_8

#define SPIx						SPI1
#define SPIx_GPIO					GPIOA
#define SPIx_CS						GPIO_Pin_4
#define SPIx_CS_GPIO				GPIOA
#define SPIx_SCK					GPIO_Pin_5
#define SPIx_MISO					GPIO_Pin_6
#define SPIx_MOSI					GPIO_Pin_7

#define DW1000_RSTn					GPIO_Pin_0
#define DW1000_RSTn_GPIO			GPIOA

#define DECARSTIRQ                  GPIO_Pin_0
#define DECARSTIRQ_GPIO             GPIOA
#define DECARSTIRQ_EXTI             EXTI_Line0
#define DECARSTIRQ_EXTI_PORT        GPIO_PortSourceGPIOA
#define DECARSTIRQ_EXTI_PIN         GPIO_PinSource0
#define DECARSTIRQ_EXTI_IRQn        EXTI0_IRQn

#define DECAIRQ                     GPIO_Pin_5
#define DECAIRQ_GPIO                GPIOB
#define DECAIRQ_EXTI                EXTI_Line5
#define DECAIRQ_EXTI_PORT           GPIO_PortSourceGPIOB
#define DECAIRQ_EXTI_PIN            GPIO_PinSource5
#define DECAIRQ_EXTI_IRQn           EXTI9_5_IRQn
#define DECAIRQ_EXTI_USEIRQ         ENABLE
#define DECAIRQ_EXTI_NOIRQ          DISABLE

#define TA_BOOT1                 	GPIO_Pin_2
#define TA_BOOT1_GPIO            	GPIOB

#define TA_RESP_DLY                 GPIO_Pin_0
#define TA_RESP_DLY_GPIO            GPIOC

#define TA_SW1_3					GPIO_Pin_0
#define TA_SW1_4					GPIO_Pin_1
#define TA_SW1_5					GPIO_Pin_2
#define TA_SW1_6					GPIO_Pin_3
#define TA_SW1_7					GPIO_Pin_4
#define TA_SW1_8					GPIO_Pin_5
#define TA_SW1_GPIO                 GPIOC

#define S1_SWITCH_ON  (1)
#define S1_SWITCH_OFF (0)
//when switch (S1) is 'on' the pin is low
int is_switch_on(uint16_t GPIOpin);

#define port_IS_TAG_pressed()		is_switch_on(TA_SW1_4)
#define port_IS_LONGDLY_pressed()	is_dlybutton_low()

#define port_USARTx_busy_sending()	0 //(USART_GetFlagStatus((USARTx),(USART_FLAG_TXE))==(RESET))
#define port_USARTx_no_data()		0 //(USART_GetFlagStatus((USARTx),(USART_FLAG_RXNE))==(RESET))
//#define port_USARTx_send_data(x)	0 //USART_SendData((USARTx),(uint8_t)(x))
#define port_USARTx_send_data(x)	{ } //USART_SendData((USARTx),(uint8_t)(x))
#define port_USARTx_receive_data()	0 //USART_ReceiveData(USARTx)

#define port_SPIx_busy_sending()		(SPI_I2S_GetFlagStatus((SPIx),(SPI_I2S_FLAG_TXE))==(RESET))
#define port_SPIx_no_data()				(SPI_I2S_GetFlagStatus((SPIx),(SPI_I2S_FLAG_RXNE))==(RESET))
#define port_SPIx_send_data(x)			SPI_I2S_SendData((SPIx),(x))
#define port_SPIx_receive_data()		SPI_I2S_ReceiveData(SPIx)
#define port_SPIx_disable()				SPI_Cmd(SPIx,DISABLE)
#define port_SPIx_enable()              SPI_Cmd(SPIx,ENABLE)
#define port_SPIx_set_chip_select()		GPIO_SetBits(SPIx_CS_GPIO,SPIx_CS)
#define port_SPIx_clear_chip_select()	GPIO_ResetBits(SPIx_CS_GPIO,SPIx_CS)

#define port_SPIy_busy_sending()		(SPI_I2S_GetFlagStatus((SPIy),(SPI_I2S_FLAG_TXE))==(RESET))
#define port_SPIy_no_data()				(SPI_I2S_GetFlagStatus((SPIy),(SPI_I2S_FLAG_RXNE))==(RESET))
#define port_SPIy_send_data(x)			SPI_I2S_SendData((SPIy),(x))
#define port_SPIy_receive_data()		SPI_I2S_ReceiveData(SPIy)
#define port_SPIy_disable()				SPI_Cmd(SPIy,DISABLE)
#define port_SPIy_enable()              SPI_Cmd(SPIy,ENABLE)
#define port_SPIy_set_chip_select()		GPIO_SetBits(SPIy_CS_GPIO,SPIy_CS)
#define port_SPIy_clear_chip_select()	GPIO_ResetBits(SPIy_CS_GPIO,SPIy_CS)
#define port_LCD_RS_set()				GPIO_SetBits(SPIy_GPIO,LCD_RS)
#define port_LCD_RS_clear()				GPIO_ResetBits(SPIy_GPIO,LCD_RS)
#define port_LCD_RW_set()				GPIO_SetBits(SPIy_GPIO,LCD_RW)
#define port_LCD_RW_clear()				GPIO_ResetBits(SPIy_GPIO,LCD_RW)

#define port_GET_stack_pointer()		__get_MSP()
#define port_GET_rtc_time()				RTC_GetCounter()
#define port_SET_rtc_time(x)			RTC_SetCounter(x)

ITStatus EXTI_GetITEnStatus(uint32_t x);

#define port_AUDIBLE_enable()			// not used
#define port_AUDIBLE_disable()			// not used
#define port_AUDIBLE_set_interval_ms(x)	// not used
#define port_AUDIBLE_get_interval_ms(x)	// not used
#define port_GetEXT_IRQStatus()             EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn)
#define port_DisableEXT_IRQ()               NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn)
#define port_EnableEXT_IRQ()                NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn)
#define port_CheckEXT_IRQ()                 GPIO_ReadInputDataBit(DECAIRQ_GPIO, DECAIRQ)
int NVIC_DisableDECAIRQ(void);

void __weak process_dwRSTn_irq(void);
void __weak process_deca_irq(void);
//define LCD functions
#define port_LCD_Clear(x) 											0
#define port_LCD_SetBackColor(x) 									0
#define port_LCD_SetTextColor(x) 									0
#define port_LCD_DisplayString(line, column, font_index, buffer) 	0
void __weak button_callback(void);

int is_IRQ_enabled(void);

int is_button_low(uint16_t GPIOpin);
#define is_fastrng_on(x)  is_button_low(x)

#define is_button_high(x)			0
void led_on(led_t led);
void led_off(led_t led);
#define gpio_set(x)				0
#define gpio_reset(x)				0
#define is_gpio_out_low(x)			0
#define is_gpio_out_high(x)			0
			
int peripherals_init(void);
void spi_peripheral_init(void);

void SPI_ChangeRate(uint16_t scalingfactor);
void SPI_ConfigFastRate(uint16_t scalingfactor);

unsigned long portGetTickCnt(void);

#define portGetTickCount() 			portGetTickCnt()

void reset_DW1000(void);
void setup_DW1000RSTnIRQ(int enable);

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
