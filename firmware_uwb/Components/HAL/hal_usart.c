#include <stdarg.h>
#include "hal_usart.h"

#define SRC_USART1_DR	(&(USART1->DR))		//���ڽ��ռĴ�����ΪԴͷ
#define SRC_USART2_DR	(&(USART2->DR))		//���ڽ��ռĴ�����ΪԴͷ


#define DMA1_MEM_LEN 512//����DMAÿ�����ݴ��͵ĳ���
char _dbg_TXBuff[DMA1_MEM_LEN];

char USART1_DMA_RX_BYTE;
char USART2_DMA_RX_BYTE;

/*
 *******************************************************************************
 *   ����1���ͺ���
 *	 ����ʹ�õ�����ͨģʽ,��ÿ�η�����ɺ�,��װ��ͨ��
 *	 װ�غ�ͨ����ȴ�������ɡ�
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

			 /* e.g. ��USARTдһ���ַ� */
			 USART_SendData(USART1, (uint8_t) buffer[i]);

			 /* ѭ��ֱ��������� */
			 while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); 
		}
	}
	else
	{
		DMA_Cmd(DMA1_Channel4, DISABLE);//���ݴ�����ɣ��ر�DMA4ͨ��
		DMA1_Channel4->CNDTR = length;						//���ݴ�����Ŀ
		DMA1_Channel4->CMAR = (u32)buffer;				//�ڴ��ַ		 
		DMA_Cmd(DMA1_Channel4, ENABLE); 					//ʹ��DMAͨ��4		
	}	
#else
	for(int i = 0; i < length; i++)
	{
		USART1->SR;

		 /* e.g. ��USARTдһ���ַ� */
		 USART_SendData(USART1, (uint8_t) buffer[i]);

		 /* ѭ��ֱ��������� */
		 while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); 
	}

#endif
	return length;
}

/*
 *******************************************************************************
 *   ����1���ͺ���
 *	 ����ʹ�õ�����ͨģʽ,��ÿ�η�����ɺ�,��װ��ͨ��
 *	 װ�غ�ͨ����ȴ�������ɡ�
 *******************************************************************************
 */
uint16_t USART2_SendBuffer(const char* buffer, uint16_t length)
{
#ifdef HAL_USART2_DMA
	DMA1_Channel7->CNDTR = length;	    			//���ݴ�����Ŀ
	DMA1_Channel7->CMAR = (u32)buffer;				//�ڴ��ַ	   
	DMA_Cmd(DMA1_Channel7, ENABLE);	 					//ʹ��DMAͨ��4
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);//ʹ��USART��DMA��������

	while(DMA_GetFlagStatus(DMA1_FLAG_TC7) == RESET);//�ȴ�DMA������ɣ���bytes-->USART1_DR	,��ʾ�����ڵ���������

	DMA_Cmd(DMA1_Channel7, DISABLE);//���ݴ�����ɣ��ر�DMA4ͨ��
	DMA_ClearFlag(DMA1_FLAG_TC7); 	//���DMA1ͨ��7������ɱ�־
	USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE); //�ر�USART��DMA��������		

#else
	for(int i = 0; i < length; i++)
	{
		USART2->SR;

		 /* e.g. ��USARTдһ���ַ� */
		 USART_SendData(USART2, (uint8_t) buffer[i]);

		 /* ѭ��ֱ��������� */
		 while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	
	}
#endif	
	return length;
}


/*
 *******************************************************************************
 *		DMA��ʽ��_dbg_printf
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
 *		����1DMA��ʼ��
 *		�ر�RXNE��TC�ж�,����IDLE
 *		����RX,TX�����Ŀ�ĵ�ַ��Դ��ַ
 *******************************************************************************
 */
void HalUASRT1_DMA_Config(void)
{
		DMA_InitTypeDef DMA_InitStructure;		

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

#ifdef HAL_USART1_DMA	
		//TX����DMA
		DMA_DeInit(DMA1_Channel4);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART1_DR;				//DMA�������ַ
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;							//������Ϊ���ݴ���Ŀ�ĵ�
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//�����ַ�Ĵ�������
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 					//�ڴ��ַ�Ĵ�������
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 	//�������ݿ��8bit
		DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte; 		//�ڴ����ݿ��8bit
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 							//����ģʽ
		DMA_InitStructure.DMA_Priority = DMA_Priority_High; 						//���ȼ�����
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								//���ڴ浽�ڴ�
		DMA_Init(DMA1_Channel4, &DMA_InitStructure);	

		DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);  	//����DMA������ɺ�����ж�	
		//DMA_Cmd(DMA1_Channel4, ENABLE); 								//ʹ��DMAͨ��4		
		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);//ʹ��USART��DMA��������
#endif
		
		//RX����DMA
		DMA_DeInit(DMA1_Channel5);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART1_DR;					 //DMA�������ַ
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&USART1_DMA_RX_BYTE;					 //DMA�ڴ����ַ
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;						 //������Ϊ���ݴ������Դ
		DMA_InitStructure.DMA_BufferSize = 1; 									 //DMA�����С
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		 //�����ַ�Ĵ�������
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 				 //�ڴ��ַ�Ĵ�������
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݿ��8bit 
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 		 //�ڴ����ݿ��8bit
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 						 //ѭ��ģʽ
		DMA_InitStructure.DMA_Priority = DMA_Priority_High; 						 //���ȼ�����
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								 //���ڴ浽�ڴ�
		DMA_Init(DMA1_Channel5, &DMA_InitStructure);							 //��ʼ��DMA

		DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE); //ʹ��DMAͨ��6��������ж�
		DMA_Cmd(DMA1_Channel5, ENABLE); 								//ʹ��DMAͨ��6
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	//ʹ��USART2����DMA���� 	
}



void HalUASRT2_DMA_Config(void)
{
		DMA_InitTypeDef DMA_InitStructure;		

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

		//TX����DMA
		DMA_DeInit(DMA1_Channel7);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART2_DR;				//DMA�������ַ
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;							//������Ϊ���ݴ���Ŀ�ĵ�
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//�����ַ�Ĵ�������
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 					//�ڴ��ַ�Ĵ�������
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 	//�������ݿ��8bit
		DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte; 		//�ڴ����ݿ��8bit
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 							//����ģʽ
		DMA_InitStructure.DMA_Priority = DMA_Priority_High; 						//���ȼ�����
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								//���ڴ浽�ڴ�
		DMA_Init(DMA1_Channel7, &DMA_InitStructure);	
	
		DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);  	//����DMA������ɺ�����ж�
		//DMA_Cmd(DMA1_Channel7, ENABLE); 								//ʹ��DMAͨ��7	
		USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);		//ʹ��USART��DMA��������
	
		//RX����DMA
		DMA_DeInit(DMA1_Channel6);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART2_DR;					 //DMA�������ַ
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&USART2_DMA_RX_BYTE;					 //DMA�ڴ����ַ
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;						 //������Ϊ���ݴ������Դ
		DMA_InitStructure.DMA_BufferSize = 1; 									 //DMA�����С
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		 //�����ַ�Ĵ�������
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 				 //�ڴ��ַ�Ĵ�������
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݿ��8bit 
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 		 //�ڴ����ݿ��8bit
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 						 //ѭ��ģʽ
		DMA_InitStructure.DMA_Priority = DMA_Priority_High; 						 //���ȼ�����
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								 //���ڴ浽�ڴ�
		DMA_Init(DMA1_Channel6, &DMA_InitStructure);							 //��ʼ��DMA

		DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE); //ʹ��DMAͨ��6��������ж�
		DMA_Cmd(DMA1_Channel6, ENABLE); 								//ʹ��DMAͨ��6
		USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);	//ʹ��USART2����DMA���� 	
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
	  //Usart3 NVIC ����
		NVIC_InitTypeDef NVIC_InitStructure;
	  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=10 ;//��ռ���ȼ�1
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
}


void HalUSART1_Init(u32 bound)
{
		USART_InitTypeDef USART_InitStructure;
	 
	   //USART ��ʼ������

		USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		//USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_InitStructure.USART_Mode = USART_Mode_Tx;	//�շ�ģʽ

	  USART_Init(USART1, &USART_InitStructure); //��ʼ������
	  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���
}

void HalUSART2_Init(u32 bound)
{
		USART_InitTypeDef USART_InitStructure;
	 
	   //USART ��ʼ������

		USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

	  USART_Init(USART2, &USART_InitStructure); //��ʼ������
	  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	  USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���
}

void HalUSART3_Init(u32 bound)
{
		USART_InitTypeDef USART_InitStructure;
	 
	   //USART ��ʼ������

		USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

	  USART_Init(USART3, &USART_InitStructure); //��ʼ������
	  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���
}


void HalUSART1_IO_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
			//USART1_TX   GPIOA.9
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
	   
	  //USART1_RX	  GPIOA.10��ʼ��
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  
}

void HalUSART2_IO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART2��GPIOAʱ��
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);	//ʹ��USART2��GPIOAʱ��
			//USART2_TX   GPIOA.2
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.9
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
	   
	  //USART2_RX	  GPIOA.3��ʼ��
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA10
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  
}

void HalUSART3_IO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��USART2��GPIOAʱ��
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);	//ʹ��USART2��GPIOAʱ��
			//USART3_TX   GPIOB.10
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB.10
	   
	  //USART3_RX	  GPIOB.11��ʼ��
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB.11  
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


