#include "compiler.h"
#include "port.h"

#include "deca_types.h"
#include "deca_regs.h"

#include "deca_spi.h"
#include "dw_main.h"
#include "dwm1000_timestamp.h"

#include "hal_led.h"
#include "hal_usb.h"
#include "hal_spi.h"
#include "hal_usart.h"

#include "bphero_uwb.h"

#include <math.h>

#ifdef RX_NODE

extern void usb_run(void);
extern int usb_init(void);

static int ret;

static uint32 status_reg = 0;

#define LCD_BUFF_LEN (200)
static uint8 usbVCOMout[LCD_BUFF_LEN];

extern dwt_config_t config;
extern dwt_config_t config2;

extern dwt_txconfig_t txconfig2;
extern dwt_txconfig_t txconfig3;

static uint8_t msg_common[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21};

static uint8_t cir_buffer[(4*CIR_LEN+1)*ANCHOR_NUM];

static uint8_t frame_seq_nb = 0;

static uint64_t rx_ts[ANCHOR_NUM];

static uint8_t msg_buffer[((ANCHOR_NUM-1)*SINGLE_LEN + END_LEN)* ANCHOR_NUM];

static int n = 0;
static uint16 fp_index;
static uint16_t current_tx;

static uint16_t offset1;
static uint16_t offset2;

static uint16_t current_freq = 1;

static int32 ci[ANCHOR_NUM];

uint8_t phase_cal[ANCHOR_NUM];

uint16_t maxGC[ANCHOR_NUM];
uint8_t rxPC[ANCHOR_NUM];

void dw_init(void)
{
		reset_DW1000();
	
		SPI_ConfigFastRate(SPI_BaudRatePrescaler_32);
	
		if(dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
		{
			while(1){};
		}
		
		dwt_loadopsettabfromotp(DWT_OPSET_TIGHT);
		
		SPI_ConfigFastRate(SPI_BaudRatePrescaler_4);
		
		dwt_configure(&config);
		dwt_configuretxrf(&txconfig2);
		
		dwt_setleds(1);
		
		dwt_setpanid(NET_PANID);
		
		dwt_setaddress16(1);
		
		dwt_setrxaftertxdelay(RX_ANT_DLY);
		dwt_settxantennadelay(TX_ANT_DLY);
}

int dw_main(void)
{
	
	dw_init();
	
	uint8_t current_idx = 0;
    uint8_t is_last_anchor = 0;
    uint8_t ret = 0;
		
	while (1)
    {
				
		dwt_setpreambledetecttimeout(0);
        dwt_setrxtimeout(0);

        dwt_rxenable(DWT_START_RX_IMMEDIATE);
			
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
		{};
			
		if (status_reg & SYS_STATUS_RXFCG)
		{
					
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
		
			if (frame_len <= FRAME_LEN_MAX)
			{
				dwt_readrxdata(rx_buffer, frame_len, 0);								
			}
						
			current_tx = rx_buffer[8];
            frame_seq_nb = rx_buffer[ALL_MSG_SN_IDX];
			
			memcpy(msg_buffer+((ANCHOR_NUM-1)*SINGLE_LEN + END_LEN)*current_tx, rx_buffer + ALL_MSG_COMMON_LEN, (ANCHOR_NUM-1)*SINGLE_LEN + END_LEN);
			
			rx_ts[current_tx] = get_rx_timestamp_u64();
			
		    fp_index = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET) >> 6;
			dwt_readaccdata(cir_buffer+current_tx*(CIR_LEN*4+1), CIR_LEN*4+1, (fp_index)*4);
			
			ci[current_tx] = dwt_readcarrierintegrator();
			
			dwt_readfromdevice(RX_TTCKO_ID, 4, 1, phase_cal+current_tx);
			
			maxGC[current_tx] = dwt_read16bitoffsetreg(RX_FQUAL_ID, 0x6);
			rxPC[current_tx] = (dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT;
		
            if (ANCHOR_NUM == current_tx + 1)
            {
				
                int xx = 0;

                for (uint8_t i = 0; i < ANCHOR_NUM; i++){

                    offset1 = ((ANCHOR_NUM-1)*SINGLE_LEN + END_LEN)*i;

                    for (uint8_t j = 0; j < ANCHOR_NUM; j++)
                    {
                        uint64_t rx_time;
											
                        if (j < i) {
                            offset2 = j*SINGLE_LEN;
                        } 
						else if(j > i){
                            offset2 = (j-1)*SINGLE_LEN;
                        }
						else if(j == i){
							continue;
						}
						
						memcpy(usbVCOMout+n, msg_buffer+offset1+offset2, SINGLE_LEN);
						n += SINGLE_LEN;
                    }

                    usbVCOMout[n] =  frame_seq_nb;
                    n += 1;

                }

                for (uint8_t i = 0; i < ANCHOR_NUM; i++){
                    
					memcpy(usbVCOMout+n, cir_buffer+(4*CIR_LEN+1)*i+1+4, 4);
                    n += 4;
									
					usbVCOMout[n] = phase_cal[i];
                    n += 1;
					
					usbVCOMout[n] = rxPC[i];
					n += 1;
					
					usbVCOMout[n] = (uint8_t)(maxGC[i] >> 8);
					usbVCOMout[n+1] = (uint8_t)(maxGC[i]);
					n += 2;

                    for (int k = 0; k < 5; k++)
                    {
						usbVCOMout[n] = (uint8_t)(rx_ts[i] >> ((4-k)*8));
						n += 1;
                    }
										
					for (int k = 0; k < 4; k++)
                    {
						usbVCOMout[n] = (uint8_t)(ci[i] >> ((3-k)*8));
						n += 1;
                    }
                }
                
                usbVCOMout[n] =  frame_seq_nb;
                n += 1;

                for (int i = 0; i<5; i++)
                {
					usbVCOMout[n] = 7-i;
					n += 1;
                }

				HalUsbWrite(usbVCOMout, n);
				n = 0;
				
				if (frame_seq_nb % 2 == 1)
				{
					if (frame_seq_nb % 4 == 1)
					{
						
						current_freq = 3;
						
						dwt_forcetrxoff();
						dwt_configure(&config2);
						dwt_configuretxrf(&txconfig3);
						
					}
					else if (frame_seq_nb % 4 == 3)
					{
						
						current_freq = 1;
						
						dwt_forcetrxoff();
						dwt_configure(&config);
						dwt_configuretxrf(&txconfig2);					
						
					}
				}
				
            }
						
		}
		else
		{			
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
		}
		
		
	}
}

#endif