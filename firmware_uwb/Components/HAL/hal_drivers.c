#include "hal_drivers.h"


static uint8_t Hal_TaskID;


/**************************************************************************************************
 * @fn      Hal_Init
 *
 * @brief   Hal Initialization function.
 *
 * @param   task_id - Hal TaskId
 *
 * @return  None
 **************************************************************************************************/
void Hal_Init( uint8_t task_id )
{
		/* Register task ID */
		Hal_TaskID = task_id;
}

/**************************************************************************************************
 * @fn      Hal_DriverInit
 *
 * @brief   Initialize HW - These need to be initialized before anyone.
 *
 * @param   task_id - Hal TaskId
 *
 * @return  None
 **************************************************************************************************/
void Hal_Driver_Init (void)
{
	/* CFG 中断优先级组*/
	Hal_NVIC_Init(HAL_NVIC_GROUP_SN);
	
  /* TIMER */
#if (defined HAL_TIMER) && (HAL_TIMER == TRUE)
	HalTimerInit();
#endif

  /* FLASH */
#if (defined HAL_FLASH) && (HAL_FLASH == TRUE)
  HalFlashInit();
#endif	
	
  /* ADC */
#if (defined HAL_ADC) && (HAL_ADC == TRUE)
  HalAdcInit();
#endif

  /* LED */
#if (defined HAL_LED) && (HAL_LED == TRUE)
  HalLedInit();
#endif

  /* UART */
#if (defined HAL_USART) && (HAL_USART == TRUE)
  HalUARTInit();
#endif

  /* KEY */
#if (defined HAL_KEY) && (HAL_KEY == TRUE)
  HalKeyInit();
#endif
  
  /* SPI */
#if (defined HAL_SPI) && (HAL_SPI == TRUE)
  HalSpiInit();
#endif

  /* HID */
#if (defined HAL_USB) && (HAL_USB == TRUE)
  HalUsbInit();
#endif

  /* RTC */
#if (defined HAL_RTC) && (HAL_RTC == TRUE)
  HalRTCInit();
#endif

  /* PMU */
#if (defined HAL_PMU) && (HAL_PMU == TRUE)
	HalPmuInit();
#endif
}

/**************************************************************************************************
 * @fn      Hal_ProcessEvent
 *
 * @brief   Hal Process Event
 *
 * @param   task_id - Hal TaskId
 *          events - events
 *
 * @return  None
 **************************************************************************************************/
uint16_t Hal_ProcessEvent( uint8_t task_id, uint16_t events )
{
  (void)task_id;  // Intentionally unreferenced parameter

  return 0;
}

