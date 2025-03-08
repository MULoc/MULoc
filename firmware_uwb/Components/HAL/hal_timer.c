#include "hal_timer.h"
#include "hal_led.h"


static volatile uint32_t SysTickDelayTime;

/*******************************************************************************
* ������  : Delay_nMs
* ����    : ��ʱ����(n����)
* ����    : nm����ʱʱ��(n����)
* ���    : ��
* ����    : �� 
* ˵��    : ��
*******************************************************************************/
void HalDelay_nMs(uint32_t nms)
{ 
	SysTickDelayTime = nms;		 
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	//ʹ�ܵδ�ʱ�� 
	while(SysTickDelayTime != 0);					//�ȴ���ʱʱ�䵽
}

/*******************************************************************************
* ������  : SysTickDelayTime_Counter
* ����    : ��ȡ���ĳ���
* ����    : ��
* ���    : ��
* ����    : �� 
* ˵��    : ��SysTick�жϳ���SysTick_Handler()����(stm32f10x_it.c)
*******************************************************************************/ 
void HalDelayTime_Counter(void)
{
	if (SysTickDelayTime > 0)
	{ 	
		SysTickDelayTime--;
	}
}

/*******************************************************************************
* ������  : HalTimeSysTick_Init
* ����    : ��ʼ��ϵͳ�δ�ʱ��SysTick
* ����    : ��
* ���    : ��
* ����    : �� 
* ˵��    : 1)��SystemFrequency / 1000		1ms�ж�һ��
*			2)��SystemFrequency / 100000	10us�ж�һ��
*			3)��SystemFrequency / 1000000	1us�ж�һ��
*			���㷽��:(SystemFrequency / Value)��ϵͳʱ�ӽ����ж�һ��
*******************************************************************************/
void HalTimeSysTick_Init(void)
{
	if (SysTick_Config(SystemCoreClock / CLOCKS_PER_SEC))
	{
		/* Capture error */
		while (1);
	}
	NVIC_SetPriority (SysTick_IRQn, 5);
}
/*******************************************************************************
* ������  : HALTimer2_Init
* ����    : Timer2��ʼ������
* ����    : ��
* ���    : ��
* ����    : �� 
* ˵��    : ��
*******************************************************************************/
void HalTimer2_Init(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		//ʹ��Timer2ʱ��		
		TIM_TimeBaseStructure.TIM_Period = TIM2_ARR;					//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ(������5000Ϊ500ms)
		TIM_TimeBaseStructure.TIM_Prescaler = TIM2_PSC;					//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ(10KHz�ļ���Ƶ��)
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//����ʱ�ӷָ�:TDTS = TIM_CKD_DIV1
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);				//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE ); 				//ʹ��TIM2ָ�����ж�		
		TIM_Cmd(TIM2, ENABLE);  														//ʹ��TIMx����
}

void HalTimer3_Init(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);		//ʹ��Timer3ʱ��		
		TIM_TimeBaseStructure.TIM_Period = TIM3_ARR;					//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ(������5000Ϊ500ms)
		TIM_TimeBaseStructure.TIM_Prescaler = TIM3_PSC;					//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ(10KHz�ļ���Ƶ��)
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//����ʱ�ӷָ�:TDTS = TIM_CKD_DIV1
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);				//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE ); 				//ʹ��TIM2ָ�����ж�		
		TIM_Cmd(TIM3,DISABLE);											/* �رն�ʱ�� */ 
}


void HalTimer4_Init(void)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
		
		TIM_DeInit(TIM4); 										/* ����������ʱ�� */
		
		TIM_TimeBaseStructure.TIM_Period=TIM4_ARR;						/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ)100ms */
		TIM_TimeBaseStructure.TIM_Prescaler=TIM4_PSC; 					/* ʱ��Ԥ��Ƶ�� 72M/72 */
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		/* �ⲿʱ�Ӳ�����Ƶ */
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 	/* ���ϼ���ģʽ */
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
		
		TIM_ClearFlag(TIM4, TIM_FLAG_Update); 					/* �������жϱ�־ */
		
		TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);					/* �����жϴ���*/ 	 
		
		TIM_Cmd(TIM4,DISABLE);											/* �رն�ʱ�� */ 
}

void HalIWDG_Init(unsigned char prer,unsigned int rlr)
{	
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //ʹ�ܶԼĴ���IWDG_PR��IWDG_RLR��д����	
	IWDG_SetPrescaler(prer);  //����IWDGԤ��Ƶֵ:����IWDGԤ��ƵֵΪ64
	IWDG_SetReload(rlr);  		//����IWDG��װ��ֵ
	IWDG_ReloadCounter();  		//����IWDG��װ�ؼĴ�����ֵ��װ��IWDG������
	IWDG_Enable();  					//ʹ��IWDG
}


void HalTimer2_NVIC_Config()
{
		NVIC_InitTypeDef NVIC_InitStructure;
		/*�ж����ȼ�NVIC����*/
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;				//TIM2�ж�
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 11;	//��ռ���ȼ�1��
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//�����ȼ�1��
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ��IRQͨ��
		NVIC_Init(&NVIC_InitStructure); 							//��ʼ��NVIC�Ĵ���	 	
}


void HalTimer3_NVIC_Config()
{
		NVIC_InitTypeDef NVIC_InitStructure;
		/*�ж����ȼ�NVIC����*/
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;				//TIM2�ж�
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 11;	//��ռ���ȼ�1��
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//�����ȼ�1��
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ��IRQͨ��
		NVIC_Init(&NVIC_InitStructure); 							//��ʼ��NVIC�Ĵ���	 	
}


void HalTimer4_NVIC_Config()
{
		NVIC_InitTypeDef NVIC_InitStructure;
		/*�ж����ȼ�NVIC����*/
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;				//TIM2�ж�
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 11;	//��ռ���ȼ�1��
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//�����ȼ�1��
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ��IRQͨ��
		NVIC_Init(&NVIC_InitStructure); 							//��ʼ��NVIC�Ĵ���	 	
}




void HalTimerInit(void)
{
#ifdef HAL_SysTick	
		HalTimeSysTick_Init();
#endif
	
#ifdef HAL_TIMER2	
		HalTimer2_Init();
		HalTimer2_NVIC_Config();	
#endif	

#ifdef HAL_TIMER3	
		HalTimer3_Init();
		HalTimer3_NVIC_Config();	
#endif

#ifdef HAL_TIMER4	
		HalTimer4_Init();
		HalTimer4_NVIC_Config();	
#endif	

#ifdef HAL_IWDG
		HalIWDG_Init(7,625);
#endif
}

void HalIWDG_Feed(void)
{   
#ifdef HAL_IWDG
 	IWDG_ReloadCounter();			//reload										   
 #endif
}

