#include "hal_key.h"
#include "hal_usart.h"

void HalKey1_IO_Init(void)
{
		GPIO_InitTypeDef	GPIO_InitStructure; 				 //定义GPIO结构体

		RCC_APB2PeriphClockCmd(HAL_KEY1_RCC, ENABLE);	//使能PA端口时钟	
		GPIO_InitStructure.GPIO_Pin = HAL_KEY1_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(HAL_KEY1_PORT, &GPIO_InitStructure);
}

void HalKey2_IO_Init(void)
{
		GPIO_InitTypeDef	GPIO_InitStructure; 				 //定义GPIO结构体

		RCC_APB2PeriphClockCmd(HAL_KEY2_RCC, ENABLE);	//使能PA端口时钟	
		GPIO_InitStructure.GPIO_Pin = HAL_KEY2_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(HAL_KEY2_PORT, &GPIO_InitStructure);
}


void HalKey3_IO_Init(void)
{
		GPIO_InitTypeDef	GPIO_InitStructure; 				 //定义GPIO结构体

		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	//JTAG disable
		RCC_APB2PeriphClockCmd(HAL_KEY3_RCC, ENABLE);	//使能PA端口时钟	
		GPIO_InitStructure.GPIO_Pin = HAL_KEY3_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
		GPIO_Init(HAL_KEY3_PORT, &GPIO_InitStructure);
}


void HalKey4_IO_Init(void)
{
		GPIO_InitTypeDef	GPIO_InitStructure; 				 //定义GPIO结构体

		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	//JTAG disable
		RCC_APB2PeriphClockCmd(HAL_KEY3_RCC, ENABLE);	//使能PA端口时钟	
		GPIO_InitStructure.GPIO_Pin = HAL_KEY3_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
		GPIO_Init(HAL_KEY3_PORT, &GPIO_InitStructure);
}

void HalKey1_Exit_Init(void)
{
		EXTI_InitTypeDef EXTI_InitStructure;	
		
		GPIO_EXTILineConfig(HAL_KEY1_PORT_SOURCE, HAL_KEY1_PIN_SOURCE);	

		EXTI_InitStructure.EXTI_Line = HAL_KEY1_EXTI_LIN;/*中断线*/
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;/*触发模式*/
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;/*触发信号*/
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;/*使能中断线*/
		EXTI_Init(&EXTI_InitStructure);/*调用库函数，初始化EXTI*/
}

void HalKey2_Exit_Init(void)
{
		EXTI_InitTypeDef EXTI_InitStructure;	

		GPIO_EXTILineConfig(HAL_KEY2_PORT_SOURCE, HAL_KEY2_PIN_SOURCE);	
		
		EXTI_InitStructure.EXTI_Line = HAL_KEY2_EXTI_LIN;/*中断线*/
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;/*触发模式*/
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;/*触发信号*/
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;/*使能中断线*/
		EXTI_Init(&EXTI_InitStructure);/*调用库函数，初始化EXTI*/
}


void HalKey3_Exit_Init(void)
{
		EXTI_InitTypeDef EXTI_InitStructure;	
		
		GPIO_EXTILineConfig(HAL_KEY3_PORT_SOURCE, HAL_KEY3_PIN_SOURCE);	
		
		EXTI_InitStructure.EXTI_Line = HAL_KEY3_EXTI_LIN;/*中断线*/
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;/*触发模式*/
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;/*触发信号*/
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;/*使能中断线*/
		EXTI_Init(&EXTI_InitStructure);/*调用库函数，初始化EXTI*/
}

void HalKey4_Exit_Init(void)
{
#ifdef HW_RTLS
		EXTI_InitTypeDef EXTI_InitStructure;	
		
		GPIO_EXTILineConfig(HAL_KEY4_PORT_SOURCE, HAL_KEY4_PIN_SOURCE);	
		
		EXTI_InitStructure.EXTI_Line = HAL_KEY4_EXTI_LIN;/*中断线*/
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;/*触发模式*/
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;/*触发信号*/
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;/*使能中断线*/
		EXTI_Init(&EXTI_InitStructure);/*调用库函数，初始化EXTI*/
#endif
}

void HalKey1_NVIC_Config(void)
{
		NVIC_InitTypeDef NVIC_InitStructure;		

		NVIC_InitStructure.NVIC_IRQChannel = HAL_KEY1_EXIT_IRQN;/*配置选中的中断向量*/
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;/*配置抢占优先级*/
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;/*配置响应优先级*/
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;/*使能中断向量*/
		NVIC_Init(&NVIC_InitStructure);/*调用库函数，初始化中断向量*/			

}

void HalKey2_NVIC_Config(void)
{
		NVIC_InitTypeDef NVIC_InitStructure;		

		NVIC_InitStructure.NVIC_IRQChannel = HAL_KEY2_EXIT_IRQN;/*配置选中的中断向量*/
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;/*配置抢占优先级*/
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;/*配置响应优先级*/
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;/*使能中断向量*/
		NVIC_Init(&NVIC_InitStructure);/*调用库函数，初始化中断向量*/			

}

void HalKey3_NVIC_Config(void)
{
		NVIC_InitTypeDef NVIC_InitStructure;		

		NVIC_InitStructure.NVIC_IRQChannel = HAL_KEY3_EXIT_IRQN;/*配置选中的中断向量*/
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;/*配置抢占优先级*/
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;/*配置响应优先级*/
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;/*使能中断向量*/
		NVIC_Init(&NVIC_InitStructure);/*调用库函数，初始化中断向量*/			

}

void HalKey4_NVIC_Config(void)
{
		NVIC_InitTypeDef NVIC_InitStructure;		

		NVIC_InitStructure.NVIC_IRQChannel = HAL_KEY3_EXIT_IRQN;/*配置选中的中断向量*/
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;/*配置抢占优先级*/
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;/*配置响应优先级*/
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;/*使能中断向量*/
		NVIC_Init(&NVIC_InitStructure);/*调用库函数，初始化中断向量*/			

}
/***************************************************************************************************
 * @fn      HalKeyInit
 *
 * @brief   Initialize Key Service
 *
 * @param   init - pointer to void that contains the initialized value
 *
 * @return  None
 ***************************************************************************************************/
void HalKeyInit (void)
{
#ifdef HAL_KEY_POWER_ISR
		HalKey1_IO_Init();
		HalKey1_Exit_Init();
		HalKey1_NVIC_Config();

		HalKey2_IO_Init();
		HalKey2_Exit_Init();
		HalKey2_NVIC_Config();		
#endif	

#ifdef HAL_KEY_ACT_ISR
		HalKey3_IO_Init();
		HalKey3_Exit_Init();
		HalKey3_NVIC_Config();
#endif

#ifdef HW_RTLS
		HalKey4_IO_Init();
		HalKey4_Exit_Init();
		HalKey4_NVIC_Config();		
#endif
}

/**************************************************************************************************
 * @fn      HalKeyRead
 *
 * @brief   Read the current value of a key
 *
 * @param   None
 *
 * @return  keys - current keys status
 **************************************************************************************************/
uint8_t HalKeyRead ( void )
{
  uint8_t keys = 0;

  return keys;
}

void HalKeyConfig (bool interruptEnable, halKeyCBack_t cback)
{
}
/***************************************************************************************************
***************************************************************************************************/
