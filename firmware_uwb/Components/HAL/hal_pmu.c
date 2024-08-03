#include "hal_pmu.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_spi.h"
#include "hal_key.h"

int deep_slp = 0;


void HalPmu_LowpowerCfg(void)
{
	//定义IO初始化结构体  

	GPIO_InitTypeDef GPIO_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB ,ENABLE); 	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  //GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	//JTAG disable
	
	GPIO_InitStructure.GPIO_Pin = ~(HAL_KEY1_PIN);   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;   

	//初始化                 

	GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
	GPIO_InitStructure.GPIO_Pin = ~(HAL_LED_POWER | HAL_KEY3_PIN | LIS3DIRQ);   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;   
	GPIO_Init(GPIOB, &GPIO_InitStructure);   

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB,DISABLE);	
													 
	ADC_Cmd(ADC1,DISABLE);		
}


int HalPmuInit(void)
{
		/* 使能电源管理单元的时钟 */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
}

void HalPmu_Enter(void)
{
		deep_slp = 1;
	
		HalPmu_LowpowerCfg();
		
		/* 进入停止模式，设置电压调节器为低功耗模式，等待中断唤醒*/
		PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);	
}

/**
  * @brief  停机唤醒后配置系统时钟: 使能 HSE, PLL
  *         并且选择PLL作为系统时钟.
  * @param  None
  * @retval None
  */
void HalPmu_SYSCLKConfig_STOP(void)
{
	//SystemInit();
	//RCC_Configuration_part2();

	ErrorStatus HSEStartUpStatus;
  /* 使能 HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* 等待 HSE 准备就绪 */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {

    /* 使能 PLL */ 
    RCC_PLLCmd(ENABLE);

    /* 等待 PLL 准备就绪 */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* 选择PLL作为系统时钟源 */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* 等待PLL被选择为系统时钟源 */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
	
	//HalAdcenable();
	deep_slp = 0;
	//_dbg_printf("deep_slp:%d\n",deep_slp);


}

/***************************************************************************************************
***************************************************************************************************/
