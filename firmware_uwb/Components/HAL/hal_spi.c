#include "hal_spi.h"


#ifdef HAL_SPI_DECA
void Hal_DECA_KEY_NVIC_Config()
{
		NVIC_InitTypeDef NVIC_InitStructure;		
	
		/* Enable and set EXTI Interrupt to the lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = DECAIRQ_EXTI_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = DECAIRQ_EXTI_USEIRQ;

		NVIC_Init(&NVIC_InitStructure);
}

void HalDecaExitInit(void)
{
		EXTI_InitTypeDef EXTI_InitStructure;	

		/* Connect EXTI Line to GPIO Pin */
		GPIO_EXTILineConfig(DECAIRQ_EXTI_PORT, DECAIRQ_EXTI_PIN);

		/* Configure EXTI line */
		EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MPW3 IRQ polarity is high by default
		EXTI_InitStructure.EXTI_LineCmd = DECAIRQ_EXTI_USEIRQ;
		EXTI_Init(&EXTI_InitStructure);
	
		Hal_DECA_KEY_NVIC_Config();
}


int Hal_SPI1_Deca_Init(void)
{
		SPI_InitTypeDef SPI_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;

		SPI_I2S_DeInit(SPI1);

		// SPIx Mode setup
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 //
		//SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
		//SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
		//SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI1_PRESCALER;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CRCPolynomial = 7;

		SPI_Init(SPI1, &SPI_InitStructure);

		// SPIx SCK and MOSI pin setup
		GPIO_InitStructure.GPIO_Pin = SPI1_SCK | SPI1_MOSI;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);

		// SPIx MISO pin setup
		GPIO_InitStructure.GPIO_Pin = SPI1_MISO;
		//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;

		GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);

		// SPIx CS pin setup
		GPIO_InitStructure.GPIO_Pin = SPI1_CS;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(SPI1_CS_GPIO, &GPIO_InitStructure);

		// Disable SPIx SS Output
		SPI_SSOutputCmd(SPI1, DISABLE);

		// Enable SPIx
		SPI_Cmd(SPI1, ENABLE);

		// Set CS high
		GPIO_SetBits(SPI1_CS_GPIO, SPI1_CS);
		//GPIO_ResetBits(SPI1_CS_GPIO, SPI1_CS);
			
		RCC_APB2PeriphClockCmd(DECA_RCC, ENABLE); //使能PA端口时钟	
		GPIO_InitStructure.GPIO_Pin = DECAIRQ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
		GPIO_Init(DECAIRQ_GPIO, &GPIO_InitStructure); 	
		
		HalDecaExitInit();

    return 0;
}


#endif


#ifdef HAL_SPI_LIS3DH

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{		
		uint8_t retry=0;				 	
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
		{
			retry++;
			if(retry>200)return 0;
		}			  
		SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据
		retry=0;

		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
		{
			retry++;
			if(retry>200)return 0;
		}	  						    
		return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据					    
}


//外部中断0服务程序
void Hal_Lis3dh_KEY_NVIC_Config()
{
		NVIC_InitTypeDef NVIC_InitStructure;		
	
		/* Enable and set EXTI Interrupt to the lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = LIS3DHIRQ_EXTI_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = LIS3DHIRQ_EXTI_USEIRQ;

		NVIC_Init(&NVIC_InitStructure);
}

void HalLis3dhExitInit(void)
{
		EXTI_InitTypeDef EXTI_InitStructure;	

		/* Connect EXTI Line to GPIO Pin */
		GPIO_EXTILineConfig(LIS3DHIRQ_EXTI_PORT, LIS3DHIRQ_EXTI_PIN);

		/* Configure EXTI line */
		EXTI_InitStructure.EXTI_Line = LIS3DHIRQ_EXTI;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MPW3 IRQ polarity is high by default
		EXTI_InitStructure.EXTI_LineCmd = LIS3DHIRQ_EXTI_USEIRQ;
		EXTI_Init(&EXTI_InitStructure);
	
		Hal_Lis3dh_KEY_NVIC_Config();
}

void Hal_SPI_LIS3DH_CS_Init(void)
{
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	//JTAG disable

		GPIO_InitTypeDef GPIO_InitStructure;

		// SPIx CS pin setup
		GPIO_InitStructure.GPIO_Pin = SPI1_LIS3DH_GPIO;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(SPI1_LIS3DH_CS_GPIO, &GPIO_InitStructure);
		SPI_LIS3DH_CS_HIGH();		
}

int Hal_SPI1_LIS3DH_Init(void)
{
			uint8_t response;
	
			Hal_SPI_LIS3DH_CS_Init();
	
			//Inizialize MEMS Sensor
			//set ODR (turn ON device)
			response = LIS3DH_SetODR(LIS3DH_ODR_100Hz);//Reg：0x20 data rate
		
			//set PowerMode 
			response = LIS3DH_SetMode(LIS3DH_NORMAL);//Reg：0x20 | 0x23高分辨率+NORMAL模式
	
			//set Fullscale
			response = LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2);//Reg：0x23 +-2G
	
			//set axis Enable
			response = LIS3DH_SetAxis(LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE); //Reg：0x20
	
			//init INT1
			response = LIS3DH_SetInt1Pin( LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE); //Reg：0x22
	
			//Int1 THS
			response = LIS3DH_SetInt1Threshold(5);//Reg：0x32
	
			//INT1 
			response = LIS3DH_SetIntConfiguration( LIS3DH_INT1_ZHIE_ENABLE | LIS3DH_INT1_ZLIE_ENABLE |
																					 LIS3DH_INT1_YHIE_ENABLE | LIS3DH_INT1_YLIE_ENABLE |
																					 LIS3DH_INT1_XHIE_ENABLE | LIS3DH_INT1_XLIE_ENABLE );//Reg：0x30
	
			//INT1 Moving recognition
			response = LIS3DH_SetIntMode(LIS3DH_INT_MODE_6D_MOVEMENT ); //Reg：0x30
	
			//INT1 6D
			response = LIS3DH_SetInt6D4DConfiguration( LIS3DH_INT1_6D_ENABLE );//Reg:0x30 | 0x24
	
			//INT1 Duration
			response = LIS3DH_SetInt1Duration(2);//Reg:0x33
	
			//		
			response = LIS3DH_HPFAOI1Enable( MEMS_ENABLE ); //Reg:0x21
	
			HalLis3dhExitInit();

}
#endif

#ifdef HAL_SPI_LCD
int Hal_SPI2_LCD_Init(void)
{
		return 0;
}
#endif

void HalSpiInit( void )
{
#ifdef	HAL_SPI_DECA
		/*Decawave uwb*/
		Hal_SPI1_Deca_Init();
#endif
		
#ifdef HAL_SPI_LIS3DH
		/*ST LIS3DH*/
		Hal_SPI1_LIS3DH_Init();
#endif

#ifdef HAL_SPI_LCD
		Hal_SPI2_LCD_Init();
#endif		
}

