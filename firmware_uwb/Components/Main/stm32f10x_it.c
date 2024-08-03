/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
	/* Tick timer count. */
volatile unsigned long time32_incr;
#include "OSAL_Comdef.h"
#include "hal_key.h"
#include "hal_spi.h"
#include "hal_flash.h"
#include "hal_pmu.h"
#include "hal_led.h"
#include "hal_timer.h"
#include "port.h" 
#include "Generic.h"
#include "MT.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
	
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	_dbg_printf("HardFault_Handler\n");	
	//sys_para.HardFault_error_bit +=1;
	//HalWrite_Flash(PAGE61_ADDR, &sys_para.flag, 16);	//检测不是第一次配置，从PAGE61_ADDR读取数据并装配 					

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
#ifdef HW_RELEASE 		
		NVIC_SystemReset(); //复位		  
#endif 		
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
	_dbg_printf("MemManage_Handler\n");	
	//sys_para.MemManage_error_bit +=1;
	//HalWrite_Flash(PAGE61_ADDR, &sys_para.flag, 16);	//检测不是第一次配置，从PAGE61_ADDR读取数据并装配 					

  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
#ifdef HW_RELEASE 		
			NVIC_SystemReset(); //复位		  
#endif  
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
	_dbg_printf("BusFault_Handler\n");	
	//sys_para.BusFault_error_bit +=1;
	//HalWrite_Flash(PAGE61_ADDR, &sys_para.flag, 16);	//检测不是第一次配置，从PAGE61_ADDR读取数据并装配 					
	
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
#ifdef HW_RELEASE 		
			NVIC_SystemReset(); //复位		  
#endif
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
	_dbg_printf("UsageFault_Handler\n");	
	//sys_para.UsageFault_error_bit +=1;
	//HalWrite_Flash(PAGE61_ADDR, &sys_para.flag, 16);	//检测不是第一次配置，从PAGE61_ADDR读取数据并装配 					
	
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
#ifdef HW_RELEASE 		
			NVIC_SystemReset(); //复位		  
#endif
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern void HalDelayTime_Counter(void);
void SysTick_Handler(void)
{
	time32_incr++;
	
	HalDelayTime_Counter();
}


void EXTI9_5_IRQHandler(void)
{
		//_dbg_printf("EXTI9_5_IRQHandler \n");
    process_deca_irq();
    /* Clear EXTI Line 8 Pending Bit */
    EXTI_ClearITPendingBit(DECAIRQ_EXTI);
}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

void EXTI0_IRQHandler(void)
{
		process_dwRSTn_irq();
    /* Clear EXTI Line 0 Pending Bit */
    EXTI_ClearITPendingBit(DECARSTIRQ_EXTI);
}


void EXTI1_IRQHandler(void)
{
#ifdef HW_BASE
#else
	if(EXTI_GetITStatus(HAL_KEY1_EXTI_LIN) != RESET)
	{
		if(GPIO_ReadInputDataBit(HAL_KEY1_PORT, HAL_KEY1_PIN)){
			//_dbg_printf("HAL_KEY1_PIN output: 1, USB Disconnect \n");
			APP_SYS_MODE_SET(Sys_Operate_Mode_USB_NON_CONNET); 				
		}
		else{
			//_dbg_printf("HAL_KEY1_PIN output: 0, USB Connect \n");
			HalBeep_Off();
			HalPmu_SYSCLKConfig_STOP();
			APP_SYS_MODE_SET(Sys_Operate_Mode_USB_CONNET); 	
		}
		EXTI_ClearITPendingBit(HAL_KEY1_EXTI_LIN);
	}
#endif
}

void EXTI2_IRQHandler(void)
{
#ifdef HW_BASE
#else

	if(EXTI_GetITStatus(HAL_KEY2_EXTI_LIN) != RESET)
	{
		if(GPIO_ReadInputDataBit(HAL_KEY2_PORT, HAL_KEY2_PIN)){
			if(sys_work_mode & Sys_Operate_Mode_USB_CONNET){
				APP_SYS_MODE_SET(Sys_Operate_Mode_USB_Battery_FULL);		
				//_dbg_printf("HAL_KEY2_PIN output: 1, Full charged \n");			
			}
		}
		EXTI_ClearITPendingBit(HAL_KEY2_EXTI_LIN);
	}
#endif
}

static int key3_state_count = 0;
void EXTI3_IRQHandler(void)
{
#ifdef HW_BASE
#else
	if(EXTI_GetITStatus(HAL_KEY3_EXTI_LIN) != RESET)
	{
		if(GPIO_ReadInputDataBit(HAL_KEY3_PORT, HAL_KEY3_PIN) == 0){
			HalPmu_SYSCLKConfig_STOP();
			
			key3_state_count = 0;
			TIM_Cmd(TIM4,ENABLE);
		}
		//
		EXTI_ClearITPendingBit(HAL_KEY3_EXTI_LIN);
	}
#endif	
}

#ifdef HAL_SPI_LIS3DH
//LIS3DH INT1
void EXTI4_IRQHandler(void)
{
	HalPmu_SYSCLKConfig_STOP();
	
	if(mt_spi_exit == 1)
		_dbg_printf("This is HalSPI Lis3dh test Threshold overflow\n");
	
	EXTI_ClearITPendingBit(LIS3DHIRQ_EXTI);  //清除LINE2上的中断标志位  
}
#endif

//TIM2_IRQHandler
static uint16_t count = 0;
void TIM2_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx更新中断标志 

		count++;
		if(count >= 32768)
			count = 0;	
		//App_Modelu_Sys_Deal_IO_LED_Event(count);
		
		HalIWDG_Feed();
	}
}

static uint16_t count_beep = 0;
void TIM3_IRQHandler(void)   //TIM3中断
{
#ifdef HW_BASE
#else
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志 

		count_beep++;
		if(count_beep % 2==0)
			HaBeepSet (HAL_BEEP1, HAL_BEEP_MODE_ON);
		else
			HaBeepSet (HAL_BEEP1, HAL_BEEP_MODE_OFF);		
	}
#endif
}


void TIM4_IRQHandler(void)   //TIM4中断
{
#ifdef HW_BASE
#else
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //清除TIMx更新中断标志 
		if(GPIO_ReadInputDataBit(HAL_KEY3_PORT, HAL_KEY3_PIN) == 0){
			key3_state_count++;
			if(key3_state_count >= 300)//大于3s
			{
				//_dbg_printf("NVIC_SystemReset\n");	
				HalLedSet(HAL_LED_ALL, HAL_LED_MODE_ON);
				HalDevice_Off();
				//GPIO_ResetBits(HAL_LED_PORT, HAL_LED_POWER);	
			  //__set_FAULTMASK(1);//关闭总中断
  			//NVIC_SystemReset();//请求单片机重启
			}
		}
		else
		{	
			//_dbg_printf("SOS is loosen and push time：%dms\n",key3_state_count * 10);		
			TIM_Cmd(TIM4,DISABLE);
		}		
	}
#endif	
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
