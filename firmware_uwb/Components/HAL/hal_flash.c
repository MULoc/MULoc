#include "hal_flash.h"


void HalFlashInit(void)
{
}
/*******************************************************************************
* 函数名  : Write_Flash
* 描述    : 写STM32指定地址的Flash
* 输入    : buff：写入数据缓冲区，len：写入数据长度
* 输出    : 无
* 返回值  : uint8_t:写稿成功返回1，失败返回0
* 说明    : 无
*******************************************************************************/
uint8_t HalWrite_Flash(uint32_t Address, uint32_t *buff, uint8_t len)
{
		volatile FLASH_Status FLASHStatus;
		uint8_t k=0;

		FLASHStatus = FLASH_COMPLETE;
		FLASH_Unlock();//解锁
		FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);//清除所有标志
		FLASHStatus = FLASH_ErasePage(Address);//扇区擦除
		if(FLASHStatus == FLASH_COMPLETE)
		{
			for(k=0;(k<len) && (FLASHStatus == FLASH_COMPLETE);k++)
			{
				FLASHStatus = FLASH_ProgramWord(Address, buff[k]);//写入一个字(32位)的数据入指定地址
				Address = Address + 4;//地址偏移4个字节	
			}		
			FLASH_Lock();//重新上锁，防止误写入
		}
		else
		{
			return 0;
		}
		if(FLASHStatus == FLASH_COMPLETE)
		{
			return 1;
		}
		return 0;
}

/*******************************************************************************
* 函数名  : Read_Flash
* 描述    : 读STM32指定地址的Flash
* 输入    : buff：读出数据缓冲区，len：读出数据长度
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void HalRead_Flash(uint32_t Address, uint32_t *buff, uint8_t len)
{
		uint8_t k;
		
		for(k=0; k<len; k++)
		{
			buff[k] =  (*(volatile uint32_t*) Address);//读指定地址的一个字的数据
			Address += 4;//地址偏移4个字节			
		}
}  
