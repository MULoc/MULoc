#include "hal_adc.h"

void HalAdc_IO_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;	
		RCC_APB2PeriphClockCmd(HAL_ADC_RCC, ENABLE ); //使能ADC1和GPIOA外设时钟 		

		/*设置PA0模拟通道输入引脚*/
		GPIO_InitStructure.GPIO_Pin = HAL_ADC1_PIN;			//选择要初始化的GPIOA的PA0引脚
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//工作模式为模拟输入
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//设置引脚输出最大速率为50MHz
		GPIO_Init(HAL_ADC_PORT, &GPIO_InitStructure);				//调用库函数中的GPIO初始化函数，初始化GPIOB中的PB5,PB6,PB7,PB8引脚

}

void HalAdc_Init(void)
{
		ADC_InitTypeDef ADC_InitStructure; 

		RCC_ADCCLKConfig(RCC_PCLK2_Div6); //设置ADC时钟分频因子为6(72M/6=12M),ADC最大工作频率为14M

		/*初始化配置ADC1*/
		ADC_DeInit(ADC1); //复位ADC1,即将ADC1的所有寄存器设置为缺省值

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					//ADC1工作在独立模式
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;						//扫描模式设置，多通道下使用使能，单通道下使用失能
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;					//模数转换工作在单次转换模式
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//由软件来触发转换启动,也可以设置成外设启动
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//ADC数据右对齐
		ADC_InitStructure.ADC_NbrOfChannel = HAL_ADC1_MAX_CH;				//顺序进行规则转换的ADC通道的数目
		ADC_Init(HAL_ADC1_SN, &ADC_InitStructure);									//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器  
		
		ADC_Cmd(HAL_ADC1_SN, ENABLE);						//使能ADC1，但还没启动ADC1	
		ADC_ResetCalibration(HAL_ADC1_SN);					//使能ADC1复位校准寄存器	 
		while(ADC_GetResetCalibrationStatus(HAL_ADC1_SN));	//等待复位完成	
		ADC_StartCalibration(HAL_ADC1_SN);					//开启AD校准 
		while(ADC_GetCalibrationStatus(HAL_ADC1_SN));		//等待校准完成	

}

void HalAdcInit( void )
{
		HalAdc_IO_Init();
		HalAdc_Init();
}

void HalAdcenable(void)
{
		ADC_Cmd(HAL_ADC1_SN, ENABLE);						//使能ADC1，但还没启动ADC1			
}

void HalAdcDisable(void)
{
		ADC_Cmd(HAL_ADC1_SN, DISABLE);						//使能ADC1，但还没启动ADC1		
}

/*******************************************************************************
* 函数名  : ADC1_Get_AdcValue
* 描述    : 读取并返回ADC对应通道的AD转换值
* 输入    : ch：ADC1转换通道
* 输出    : 无
* 返回    : u16：ADC1转换通道单次转换返回的AD值 
* 说明    : 本例程只能转换ADC1的通道0，即ch=0
*******************************************************************************/
uint16_t HalAdcValueGet(uint8_t ch)   
{
		uint16_t data;
		HalAdcenable();
		ADC_RegularChannelConfig(HAL_ADC1_SN, ch, 1, ADC_SampleTime_239Cycles5 );	//设置ADC1的转换通道ch,一个序列,采样时间为239.5周期	  			     
		ADC_SoftwareStartConvCmd(HAL_ADC1_SN, ENABLE);								//软件启动ADC1开始转换	 
		while(!ADC_GetFlagStatus(HAL_ADC1_SN, ADC_FLAG_EOC ));						//等待AD转换结束
		data =ADC_GetConversionValue(HAL_ADC1_SN);								//返回最近一次ADC1规则组转换的AD值
		HalAdcDisable();

		return data;
}
