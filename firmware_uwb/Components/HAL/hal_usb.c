#include "hal_usb.h"
#include "hw_config.h"
#include "usb_init.h"
#include "usb_mem.h"
#include "usb_conf.h"
#include "usb_regs.h"

void HalUsbInit( void )
{
    Set_System();

    Set_USBClock();

    USB_Interrupts_Config();

    USB_Init();	
}

int HalUsbRead(uint8_t *buffter, uint32_t len)
{
		int ret = 0;	
		if(USB_RxRead(buffter,len) != 0)
		{
			//HalUsbWrite(buffter, strlen(buffter));
			ret = 1;
		}	
		return ret;
}

void HalUsbWrite(uint8_t *buffter, uint32_t len)
{
		USB_TxWrite2(buffter, len);
	//USB_TxWrite(buffter, len);
}
