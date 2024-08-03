/**
  ******************************************************************************
  * @file    hw_config.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/

#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "Queue.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
USART_InitTypeDef USART_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;


#define USB_COM_RX_BUF_SIZE         (256)//(1024 + 256)
//#define USB_COM_TX_BUF_SIZE         (512)//(1024 + 256)
#define USB_COM_TX_BUF_SIZE         (1024)//(1024 + 256)

static QUEUE8_t m_QueueUsbComRx         = {0};
static QUEUE8_t m_QueueUsbComTx         = {0};
static uint8_t  m_UsbComRxBuf[USB_COM_RX_BUF_SIZE]      = {0};     
static uint8_t  m_UsbComTxBuf[USB_COM_TX_BUF_SIZE]      = {0};
static uint8_t  m_UsbComTxBuf2[USB_COM_TX_BUF_SIZE]      = {0};
static uint8_t  m_UsbComTxBuf3[USB_COM_TX_BUF_SIZE]      = {0};
static uint8_t  m_UsbComTxBuf4[USB_COM_TX_BUF_SIZE]      = {0};

#define QUEUE_SIZE 4
static uint8_t queue[QUEUE_SIZE] = {0};
uint8_t front = 0;
uint8_t rear = 0;

static uint32_t write_length1 = 20000;
static uint32_t write_length2 = 20000;
static uint32_t write_length3 = 20000;
static uint32_t write_length4 = 20000;
static uint32_t current_length1 = 0;
static uint32_t current_length2 = 0;
static uint32_t current_length3 = 0;
static uint32_t current_length4 = 0;
static uint8_t free_flag1 = 1;
static uint8_t free_flag2 = 1;
static uint8_t free_flag3 = 1;
static uint8_t free_flag4 = 1;

static uint8_t flag = 0;


static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
/* Extern variables ----------------------------------------------------------*/

extern LINE_CODING linecoding;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  QUEUE_PacketCreate(&m_QueueUsbComRx, m_UsbComRxBuf, sizeof(m_UsbComRxBuf));
  QUEUE_PacketCreate(&m_QueueUsbComTx, m_UsbComTxBuf, sizeof(m_UsbComTxBuf));
    
  /* Enable USB_DISCONNECT GPIO clock */
  RCC_APB2PeriphClockCmd((uint32_t) RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);

  /* Configure USB pull-up pin */
  GPIO_InitStructure.GPIO_Pin = (uint16_t)USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
  
  /* Configure the EXTI line 18 connected internally to the USB IP */
  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
  /*Enable SystemCoreClock*/
//  SystemInit();
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure; 

  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
    /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
  if (NewState == DISABLE)
  {
    GPIO_ResetBits(USB_DISCONNECT, (uint16_t) USB_DISCONNECT_PIN);
  }
  else
  {
    GPIO_SetBits(USB_DISCONNECT, (uint16_t) USB_DISCONNECT_PIN);
  }
}

/*******************************************************************************
* Function Name : void USB_Config(void)
* Description   : USB系统初始化
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
void USB_Config(void)
{
    Set_System();

    Set_USBClock();

    USB_Interrupts_Config();

    USB_Init();
}

/*******************************************************************************
* Function Name : uint32_t USB_RxRead(uint8_t *buffter, uint32_t buffterSize)
* Description   : 从USB接收缓存中读数据
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t USB_RxRead(uint8_t *buffter, uint32_t buffterSize)
{
		return QUEUE_PacketOut(&m_QueueUsbComRx, buffter, buffterSize);
		//return 0;
}
/*******************************************************************************
* Function Name : uint32_t USB_RxWrite(uint8_t *buffter, uint32_t writeLen)
* Description   : 写数据到USB接收缓存中
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t USB_RxWrite(uint8_t *buffter, uint32_t writeLen)
{
    return QUEUE_PacketIn(&m_QueueUsbComRx, buffter, writeLen);
		//return 0;	
}
/*******************************************************************************
* Function Name : uint32_t USB_TxRead(uint8_t *buffter, uint32_t buffterSize)
* Description   : 从USB发送缓存中读数据
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t USB_TxRead(uint8_t *buffter, uint32_t buffterSize)
{
	return QUEUE_PacketOut(&m_QueueUsbComTx, buffter, buffterSize);;
		//return 0;
}

uint32_t USB_TxRead2(uint8_t *buffter, uint32_t buffterSize)
{
    //buffter = buffer+current_length;
	//current_length += buffterSize;
	
	// Buffer1 clear
	if (current_length1 >= write_length1)
	{
		current_length1 = 0;
		free_flag1 = 1;
		queue[rear] = 0;
		rear = (rear+1)%QUEUE_SIZE;
	}
	
	// Buffer2 clear
	if (current_length2 >= write_length2)
	{
		current_length2 = 0;
		free_flag2 = 1;
		queue[rear] = 0;
		rear = (rear+1)%QUEUE_SIZE;
	}
	
	// Buffer3 clear
	if (current_length3 >= write_length3)
	{
		current_length3 = 0;
		free_flag3 = 1;
		queue[rear] = 0;
		rear = (rear+1)%QUEUE_SIZE;
	}
	
	// Buffer4 clear
	if (current_length4 >= write_length4)
	{
		current_length4 = 0;
		free_flag4 = 1;
		queue[rear] = 0;
		rear = (rear+1)%QUEUE_SIZE;
	}
	
	// Buffers are empty
	if (queue[rear] == 0)
	{
		return 0;
	}
	
	// Buffer is is full and busy
	//if ((turn == 1) && (free_flag1 == 0))
	if (queue[rear] == 1)
	{
		if (current_length1 + buffterSize < write_length1)
		{
			if (flag!=0)
			{
//				m_UsbComTxBuf[current_length1] = flag;
//				m_UsbComTxBuf[current_length1+1] = flag;
//				m_UsbComTxBuf[current_length1+2] = flag;
//				m_UsbComTxBuf[current_length1+3] = flag;
//				m_UsbComTxBuf[current_length1+4] = flag;
//				m_UsbComTxBuf[current_length1+5] = flag;
				flag = 0;
			}
			
			memcpy(buffter,m_UsbComTxBuf+current_length1,buffterSize);
			current_length1 += buffterSize;
			return buffterSize;
		}
		else
		{
			memcpy(buffter,m_UsbComTxBuf+current_length1,write_length1-current_length1);
			uint32_t temp = write_length1-current_length1;
			current_length1 += buffterSize;
			return temp;
		}
	}
	//else if ((turn == 2) && (free_flag2 == 0))
	else if (queue[rear] == 2)
	{
		if (current_length2 + buffterSize < write_length2)
		{
			if (flag!=0)
			{
//				m_UsbComTxBuf2[current_length2] = flag;
//				m_UsbComTxBuf2[current_length2+1] = flag;
//				m_UsbComTxBuf2[current_length2+2] = flag;
//				m_UsbComTxBuf2[current_length2+3] = flag;
//				m_UsbComTxBuf2[current_length2+4] = flag;
//				m_UsbComTxBuf2[current_length2+5] = flag;
				flag = 0;
			}
			
			memcpy(buffter,m_UsbComTxBuf2+current_length2,buffterSize);
			current_length2 += buffterSize;
			return buffterSize;
		}
		else
		{
			memcpy(buffter,m_UsbComTxBuf2+current_length2,write_length2-current_length2);
			uint32_t temp = write_length2-current_length2;
			current_length2 += buffterSize;
			return temp;
		}
	}
	//else if ((turn == 3) && (free_flag3 == 0))
	else if (queue[rear] == 3)
	{
		if (current_length3 + buffterSize < write_length3)
		{
			if (flag!=0)
			{
//				m_UsbComTxBuf3[current_length3] = flag;
//				m_UsbComTxBuf3[current_length3+1] = flag;
//				m_UsbComTxBuf3[current_length3+2] = flag;
//				m_UsbComTxBuf3[current_length3+3] = flag;
//				m_UsbComTxBuf3[current_length3+4] = flag;
//				m_UsbComTxBuf3[current_length3+5] = flag;
				flag = 0;
			}
			
			memcpy(buffter,m_UsbComTxBuf3+current_length3,buffterSize);
			current_length3 += buffterSize;
			return buffterSize;
		}
		else
		{
			memcpy(buffter,m_UsbComTxBuf3+current_length3,write_length3-current_length3);
			uint32_t temp = write_length3-current_length3;
			current_length3 += buffterSize;
			return temp;
		}
	}
	else if (queue[rear] == 4)
	{
		if (current_length4 + buffterSize < write_length4)
		{
			if (flag!=0)
			{
//				m_UsbComTxBuf4[current_length4] = flag;
//				m_UsbComTxBuf4[current_length4+1] = flag;
//				m_UsbComTxBuf4[current_length4+2] = flag;
//				m_UsbComTxBuf4[current_length4+3] = flag;
//				m_UsbComTxBuf4[current_length4+4] = flag;
//				m_UsbComTxBuf4[current_length4+5] = flag;
				flag = 0;
			}
			
			memcpy(buffter,m_UsbComTxBuf4+current_length4,buffterSize);
			current_length4 += buffterSize;
			return buffterSize;
		}
		else
		{
			memcpy(buffter,m_UsbComTxBuf4+current_length4,write_length4-current_length4);
			uint32_t temp = write_length4-current_length4;
			current_length4 += buffterSize;
			return temp;
		}
	}
	
	
	//return (current_length+buffterSize > write_length) ? write_length-current_length:buffterSize;
	//return write_length;
	//return QUEUE_PacketOut(&m_QueueUsbComTx, buffter, buffterSize);;
		//return 0;
}
/*******************************************************************************
* Function Name : uint32_t USB_TxWrite(uint8_t *buffter, uint32_t writeLen)
* Description   : 写数据到USB发送缓存中
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t USB_TxWrite(uint8_t *buffter, uint32_t writeLen)
{
	return QUEUE_PacketIn(&m_QueueUsbComTx, buffter, writeLen);
		//return 0;
}

uint32_t USB_TxWrite2(uint8_t *buffter, uint32_t writeLen)
{
    if (free_flag1 == 1)  // Buffer1 is free now
	{	
		free_flag1 = 0;
		queue[front] = 1;
		front = (front+1)%QUEUE_SIZE;
		memcpy(m_UsbComTxBuf,buffter,writeLen);
//		m_UsbComTxBuf[0] = 1;
//		m_UsbComTxBuf[1] = 1;
//		m_UsbComTxBuf[2] = 1;
//		m_UsbComTxBuf[3] = 1;
		write_length1 = writeLen;
		return writeLen;
	}
	else if(free_flag2 == 1)  // Buffer2 is free now
	{
		free_flag2 = 0;
		queue[front] = 2;
		front = (front+1)%QUEUE_SIZE;
		memcpy(m_UsbComTxBuf2,buffter,writeLen);
//		m_UsbComTxBuf2[0] = 2;
//		m_UsbComTxBuf2[1] = 2;
//		m_UsbComTxBuf2[2] = 2;
//		m_UsbComTxBuf2[3] = 2;		
		write_length2 = writeLen;
		return writeLen;
	}
	else if(free_flag3 == 1)  // Buffer3 is free now
	{
		free_flag3 = 0;
		queue[front] = 3;
		front = (front+1)%QUEUE_SIZE;
		
		memcpy(m_UsbComTxBuf3,buffter,writeLen);
//		m_UsbComTxBuf3[0] = 3;
//		m_UsbComTxBuf3[1] = 3;
//		m_UsbComTxBuf3[2] = 3;
//		m_UsbComTxBuf3[3] = 3;		
		write_length3 = writeLen;
		return writeLen;
	}
	else if(free_flag4 == 1)  // Buffer3 is free now
	{
		free_flag4 = 0;
		queue[front] = 4;
		front = (front+1)%QUEUE_SIZE;
		
		memcpy(m_UsbComTxBuf4,buffter,writeLen);
//		m_UsbComTxBuf4[0] = 4;
//		m_UsbComTxBuf4[1] = 4;
//		m_UsbComTxBuf4[2] = 4;
//		m_UsbComTxBuf4[3] = 4;		
		write_length4 = writeLen;
		return writeLen;
	}
	
	flag = 221;
	
	return 0;

	//return QUEUE_PacketIn(&m_QueueUsbComTx, buffter, writeLen);
		//return 0;
}



/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(uint32_t*)ID1;
  Device_Serial1 = *(uint32_t*)ID2;
  Device_Serial2 = *(uint32_t*)ID3;  

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
