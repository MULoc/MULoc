#include "hal_adc.h"

void HalAdc_IO_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;	
		RCC_APB2PeriphClockCmd(HAL_ADC_RCC, ENABLE ); //ʹ��ADC1��GPIOA����ʱ�� 		

		/*����PA0ģ��ͨ����������*/
		GPIO_InitStructure.GPIO_Pin = HAL_ADC1_PIN;			//ѡ��Ҫ��ʼ����GPIOA��PA0����
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//����ģʽΪģ������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//������������������Ϊ50MHz
		GPIO_Init(HAL_ADC_PORT, &GPIO_InitStructure);				//���ÿ⺯���е�GPIO��ʼ����������ʼ��GPIOB�е�PB5,PB6,PB7,PB8����

}

void HalAdc_Init(void)
{
		ADC_InitTypeDef ADC_InitStructure; 

		RCC_ADCCLKConfig(RCC_PCLK2_Div6); //����ADCʱ�ӷ�Ƶ����Ϊ6(72M/6=12M),ADC�����Ƶ��Ϊ14M

		/*��ʼ������ADC1*/
		ADC_DeInit(ADC1); //��λADC1,����ADC1�����мĴ�������Ϊȱʡֵ

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					//ADC1�����ڶ���ģʽ
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;						//ɨ��ģʽ���ã���ͨ����ʹ��ʹ�ܣ���ͨ����ʹ��ʧ��
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;					//ģ��ת�������ڵ���ת��ģʽ
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//�����������ת������,Ҳ�������ó���������
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//ADC�����Ҷ���
		ADC_InitStructure.ADC_NbrOfChannel = HAL_ADC1_MAX_CH;				//˳����й���ת����ADCͨ������Ŀ
		ADC_Init(HAL_ADC1_SN, &ADC_InitStructure);									//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���  
		
		ADC_Cmd(HAL_ADC1_SN, ENABLE);						//ʹ��ADC1������û����ADC1	
		ADC_ResetCalibration(HAL_ADC1_SN);					//ʹ��ADC1��λУ׼�Ĵ���	 
		while(ADC_GetResetCalibrationStatus(HAL_ADC1_SN));	//�ȴ���λ���	
		ADC_StartCalibration(HAL_ADC1_SN);					//����ADУ׼ 
		while(ADC_GetCalibrationStatus(HAL_ADC1_SN));		//�ȴ�У׼���	

}

void HalAdcInit( void )
{
		HalAdc_IO_Init();
		HalAdc_Init();
}

void HalAdcenable(void)
{
		ADC_Cmd(HAL_ADC1_SN, ENABLE);						//ʹ��ADC1������û����ADC1			
}

void HalAdcDisable(void)
{
		ADC_Cmd(HAL_ADC1_SN, DISABLE);						//ʹ��ADC1������û����ADC1		
}

/*******************************************************************************
* ������  : ADC1_Get_AdcValue
* ����    : ��ȡ������ADC��Ӧͨ����ADת��ֵ
* ����    : ch��ADC1ת��ͨ��
* ���    : ��
* ����    : u16��ADC1ת��ͨ������ת�����ص�ADֵ 
* ˵��    : ������ֻ��ת��ADC1��ͨ��0����ch=0
*******************************************************************************/
uint16_t HalAdcValueGet(uint8_t ch)   
{
		uint16_t data;
		HalAdcenable();
		ADC_RegularChannelConfig(HAL_ADC1_SN, ch, 1, ADC_SampleTime_239Cycles5 );	//����ADC1��ת��ͨ��ch,һ������,����ʱ��Ϊ239.5����	  			     
		ADC_SoftwareStartConvCmd(HAL_ADC1_SN, ENABLE);								//�������ADC1��ʼת��	 
		while(!ADC_GetFlagStatus(HAL_ADC1_SN, ADC_FLAG_EOC ));						//�ȴ�ADת������
		data =ADC_GetConversionValue(HAL_ADC1_SN);								//�������һ��ADC1������ת����ADֵ
		HalAdcDisable();

		return data;
}
