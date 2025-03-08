#include "hal_pmu.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_spi.h"
#include "hal_key.h"

int deep_slp = 0;


void HalPmu_LowpowerCfg(void)
{
	//����IO��ʼ���ṹ��  

	GPIO_InitTypeDef GPIO_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB ,ENABLE); 	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  //GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	//JTAG disable
	
	GPIO_InitStructure.GPIO_Pin = ~(HAL_KEY1_PIN);   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;   

	//��ʼ��                 

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
		/* ʹ�ܵ�Դ����Ԫ��ʱ�� */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
}

void HalPmu_Enter(void)
{
		deep_slp = 1;
	
		HalPmu_LowpowerCfg();
		
		/* ����ֹͣģʽ�����õ�ѹ������Ϊ�͹���ģʽ���ȴ��жϻ���*/
		PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);	
}

/**
  * @brief  ͣ�����Ѻ�����ϵͳʱ��: ʹ�� HSE, PLL
  *         ����ѡ��PLL��Ϊϵͳʱ��.
  * @param  None
  * @retval None
  */
void HalPmu_SYSCLKConfig_STOP(void)
{
	//SystemInit();
	//RCC_Configuration_part2();

	ErrorStatus HSEStartUpStatus;
  /* ʹ�� HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* �ȴ� HSE ׼������ */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {

    /* ʹ�� PLL */ 
    RCC_PLLCmd(ENABLE);

    /* �ȴ� PLL ׼������ */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* ѡ��PLL��Ϊϵͳʱ��Դ */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* �ȴ�PLL��ѡ��Ϊϵͳʱ��Դ */
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
