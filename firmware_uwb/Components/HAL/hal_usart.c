#include <stdarg.h>
#include "hal_usart.h"

#define SRC_USART1_DR	(&(USART1->DR))		//串口接收寄存器作为源头
#define SRC_USART2_DR	(&(USART2->DR))		//串口接收寄存器作为源头


#define DMA1_MEM_LEN 512//保存DMA每次数据传送的长度
char _dbg_TXBuff[DMA1_MEM_LEN];

char USART1_DMA_RX_BYTE;
char USART2_DMA_RX_BYTE;

/*
 *******************************************************************************
 *   串口1发送函数
 *	 由于使用的是普通模式,在每次发送完成后,重装载通道
 *	 装载好通道后等待发送完成。
 *******************************************************************************
 */
uint16_t USART1_SendBuffer(const char* buffer, uint16_t length, int flag)
{
#ifdef HAL_USART1_DMA
	if(flag == TRUE)
	{
		for(int i = 0; i < length; i++)
		{
			USART1->SR;

			 /* e.g. 给USART写一个字符 */
			 USART_SendData(USART1, (uint8_t) buffer[i]);

			 /* 循环直到发送完成 */
			 while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); 
		}
	}
	else
	{
		DMA_Cmd(DMA1_Channel4, DISABLE);//数据传输完成，关闭DMA4通道
		DMA1_Channel4->CNDTR = length;						//数据传输数目
		DMA1_Channel4->CMAR = (u32)buffer;				//内存地址		 
		DMA_Cmd(DMA1_Channel4, ENABLE); 					//使能DMA通道4		
	}	
#else
	for(int i = 0; i < length; i++)
	{
		USART1->SR;

		 /* e.g. 给USART写一个字符 */
		 USART_SendData(USART1, (uint8_t) buffer[i]);

		 /* 循环直到发送完成 */
		 while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); 
	}

#endif
	return length;
}

/*
 *******************************************************************************
 *   串口1发送函数
 *	 由于使用的是普通模式,在每次发送完成后,重装载通道
 *	 装载好通道后等待发送完成。
 *******************************************************************************
 */
uint16_t USART2_SendBuffer(const char* buffer, uint16_t length)
{
#ifdef HAL_USART2_DMA
	DMA1_Channel7->CNDTR = length;	    			//数据传输数目
	DMA1_Channel7->CMAR = (u32)buffer;				//内存地址	   
	DMA_Cmd(DMA1_Channel7, ENABLE);	 					//使能DMA通道4
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);//使能USART的DMA发送请求

	while(DMA_GetFlagStatus(DMA1_FLAG_TC7) == RESET);//等待DMA发送完成，从bytes-->USART1_DR	,显示到串口调试助手上

	DMA_Cmd(DMA1_Channel7, DISABLE);//数据传输完成，关闭DMA4通道
	DMA_ClearFlag(DMA1_FLAG_TC7); 	//清除DMA1通道7数据完成标志
	USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE); //关闭USART的DMA发送请求		

#else
	for(int i = 0; i < length; i++)
	{
		USART2->SR;

		 /* e.g. 给USART写一个字符 */
		 USART_SendData(USART2, (uint8_t) buffer[i]);

		 /* 循环直到发送完成 */
		 while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	
	}
#endif	
	return length;
}


/*
 *******************************************************************************
 *		DMA方式的_dbg_printf
 *******************************************************************************
 */
void _dbg_printf(const char *format,...)
{
#ifdef HW_RELEASE 
#else
		uint32_t length;
		va_list args;
	 
		va_start(args, format);
		length = vsnprintf((char*)_dbg_TXBuff, sizeof(_dbg_TXBuff), (char*)format, args);
		va_end(args);
	
		USART1_SendBuffer((const char*)_dbg_TXBuff,length, TRUE); 
#endif
}

/*
 *******************************************************************************
 *		串口1DMA初始化
 *		关闭RXNE和TC中断,开启IDLE
 *		配置RX,TX传输的目的地址和源地址
 *******************************************************************************
 */
void HalUASRT1_DMA_Config(void)
{
		DMA_InitTypeDef DMA_InitStructure;		

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

#ifdef HAL_USART1_DMA	
		//TX发送DMA
		DMA_DeInit(DMA1_Channel4);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART1_DR;				//DMA外设基地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;							//外设作为数据传输目的地
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设地址寄存器不变
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 					//内存地址寄存器递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 	//外设数据宽度8bit
		DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte; 		//内存数据宽度8bit
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 							//正常模式
		DMA_InitStructure.DMA_Priority = DMA_Priority_High; 						//优先级：高
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								//非内存到内存
		DMA_Init(DMA1_Channel4, &DMA_InitStructure);	

		DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);  	//配置DMA发送完成后产生中断	
		//DMA_Cmd(DMA1_Channel4, ENABLE); 								//使能DMA通道4		
		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);//使能USART的DMA发送请求
#endif
		
		//RX接受DMA
		DMA_DeInit(DMA1_Channel5);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART1_DR;					 //DMA外设基地址
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&USART1_DMA_RX_BYTE;					 //DMA内存基地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;						 //外设作为数据传输的来源
		DMA_InitStructure.DMA_BufferSize = 1; 									 //DMA缓存大小
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		 //外设地址寄存器不变
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 				 //内存地址寄存器递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据宽度8bit 
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 		 //内存数据宽度8bit
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 						 //循环模式
		DMA_InitStructure.DMA_Priority = DMA_Priority_High; 						 //优先级：高
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								 //非内存到内存
		DMA_Init(DMA1_Channel5, &DMA_InitStructure);							 //初始化DMA

		DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE); //使能DMA通道6传输完成中断
		DMA_Cmd(DMA1_Channel5, ENABLE); 								//使能DMA通道6
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	//使能USART2接收DMA请求 	
}



void HalUASRT2_DMA_Config(void)
{
		DMA_InitTypeDef DMA_InitStructure;		

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

		//TX发送DMA
		DMA_DeInit(DMA1_Channel7);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART2_DR;				//DMA外设基地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;							//外设作为数据传输目的地
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设地址寄存器不变
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 					//内存地址寄存器递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 	//外设数据宽度8bit
		DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte; 		//内存数据宽度8bit
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 							//正常模式
		DMA_InitStructure.DMA_Priority = DMA_Priority_High; 						//优先级：高
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								//非内存到内存
		DMA_Init(DMA1_Channel7, &DMA_InitStructure);	
	
		DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);  	//配置DMA发送完成后产生中断
		//DMA_Cmd(DMA1_Channel7, ENABLE); 								//使能DMA通道7	
		USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);		//使能USART的DMA发送请求
	
		//RX接受DMA
		DMA_DeInit(DMA1_Channel6);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART2_DR;					 //DMA外设基地址
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&USART2_DMA_RX_BYTE;					 //DMA内存基地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;						 //外设作为数据传输的来源
		DMA_InitStructure.DMA_BufferSize = 1; 									 //DMA缓存大小
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		 //外设地址寄存器不变
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 				 //内存地址寄存器递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据宽度8bit 
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 		 //内存数据宽度8bit
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 						 //循环模式
		DMA_InitStructure.DMA_Priority = DMA_Priority_High; 						 //优先级：高
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								 //非内存到内存
		DMA_Init(DMA1_Channel6, &DMA_InitStructure);							 //初始化DMA

		DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE); //使能DMA通道6传输完成中断
		DMA_Cmd(DMA1_Channel6, ENABLE); 								//使能DMA通道6
		USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);	//使能USART2接收DMA请求 	
}
void HalUSART1_DMA_TX_NVIC_Config(void)
{
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	
}
void HalUSART1_DMA_RX_NVIC_Config(void)
{
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	
}

void HalUSART2_DMA_TX_NVIC_Config(void)
{
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);		
}

void HalUSART2_DMA_RX_NVIC_Config(void)
{
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	
}


void HalUASRT1_NVIC_Config(void)
{
		
}

void HalUASRT2_NVIC_Config(void)
{
}

void HalUASRT3_NVIC_Config(void)
{
	  //Usart3 NVIC 配置
		NVIC_InitTypeDef NVIC_InitStructure;
	  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=10 ;//抢占优先级1
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}


void HalUSART1_Init(u32 bound)
{
		USART_InitTypeDef USART_InitStructure;
	 
	   //USART 初始化设置

		USART_InitStructure.USART_BaudRate = bound;//串口波特率
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		//USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_InitStructure.USART_Mode = USART_Mode_Tx;	//收发模式

	  USART_Init(USART1, &USART_InitStructure); //初始化串口
	  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
	  USART_Cmd(USART1, ENABLE);                    //使能串口
}

void HalUSART2_Init(u32 bound)
{
		USART_InitTypeDef USART_InitStructure;
	 
	   //USART 初始化设置

		USART_InitStructure.USART_BaudRate = bound;//串口波特率
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	  USART_Init(USART2, &USART_InitStructure); //初始化串口
	  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
	  USART_Cmd(USART2, ENABLE);                    //使能串口
}

void HalUSART3_Init(u32 bound)
{
		USART_InitTypeDef USART_InitStructure;
	 
	   //USART 初始化设置

		USART_InitStructure.USART_BaudRate = bound;//串口波特率
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	  USART_Init(USART3, &USART_InitStructure); //初始化串口
	  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
	  USART_Cmd(USART3, ENABLE);                    //使能串口
}


void HalUSART1_IO_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
			//USART1_TX   GPIOA.9
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
	   
	  //USART1_RX	  GPIOA.10初始化
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  
}

void HalUSART2_IO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能USART2，GPIOA时钟
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);	//使能USART2，GPIOA时钟
			//USART2_TX   GPIOA.2
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.9
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
	   
	  //USART2_RX	  GPIOA.3初始化
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA10
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  
}

void HalUSART3_IO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能USART2，GPIOA时钟
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);	//使能USART2，GPIOA时钟
			//USART3_TX   GPIOB.10
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB.10
	   
	  //USART3_RX	  GPIOB.11初始化
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB.11  
}


void HalUARTInit ( void )
{
#ifdef HAL_USART1
			HalUSART1_IO_Init();
			HalUSART1_Init(1382400);
	#ifdef HAL_USART1_DMA	
			HalUASRT1_DMA_Config(); 
			HalUSART1_DMA_TX_NVIC_Config(); 			
			//HalUSART1_DMA_RX_NVIC_Config(); 
	#endif
#endif
	
#ifdef HAL_USART2
			HalUSART2_IO_Init();
			HalUSART2_Init(1382400);
	#ifdef HAL_USART2_DMA			
			HalUASRT2_DMA_Config(); 
			HalUSART2_DMA_TX_NVIC_Config(); 		
			HalUSART2_DMA_RX_NVIC_Config();	
	#endif
#endif
	
#ifdef HAL_USART3
			HalUSART3_IO_Init();
			HalUSART3_Init(1382400);
			HalUASRT3_NVIC_Config();	
#endif

}


