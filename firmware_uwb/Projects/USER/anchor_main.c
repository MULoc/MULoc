#include "compiler.h"
#include "port.h"

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

#ifdef TX_NODE

extern void usb_run(void);
extern int usb_init(void);

static uint8_t anchor_state = ANCHOR_LISTEN;

static int ret;

static uint32 status_reg = 0;

#define LCD_BUFF_LEN (500)
static uint8 usbVCOMout[LCD_BUFF_LEN * 8];

extern dwt_config_t config;
extern dwt_config_t config2;
extern dwt_txconfig_t txconfig2;
extern dwt_txconfig_t txconfig3;

extern uint16 rfDelaysTREK[2];

static uint8_t current_tx;
static uint8_t offset;

static uint8_t msg_common[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21};

static uint8_t msg_payload[(ANCHOR_NUM-1)*SINGLE_LEN + END_LEN+4];

static uint8_t sending_msg[sizeof(msg_common)+sizeof(msg_payload)];

static uint8_t frame_seq_nb = 0;

static uint64_t rx_ts;
static uint64_t tx_ts;

static uint8_t cir_buffer[4*CIR_LEN+1];

static uint16_t current_freq = 1;

static uint16_t err_num = 0;

static uint16_t rec_cnt = 0;

static int n = 0;

uint8 phase_cal;

uint16_t maxGC;
uint8_t rxPC;

void dw_init(void)
{
	reset_DW1000();

	SPI_ConfigFastRate(SPI_BaudRatePrescaler_32);

	if(dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
	{
		while(1){};
	}
	
	dwt_xtaltrim(16);
	dwt_loadopsettabfromotp(DWT_OPSET_TIGHT);
	
	SPI_ConfigFastRate(SPI_BaudRatePrescaler_4);
	
	dwt_configure(&config);		
	
	dwt_setsmarttxpower(1);
	dwt_configuretxrf(&txconfig2);
	
	dwt_setleds(1);
	
	dwt_setpanid(NET_PANID);
	
	dwt_setaddress16(1);
	
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);
	
	dwt_setinterrupt(DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
	
	msg_f_send.sourceAddr[0] = 1 & 0xFF;
    msg_f_send.sourceAddr[1] =(1>>8)& 0xFF;
	
}

int dw_main(void)
{
    msg_common[8] = ANCHOR_ID;

	dw_init();
	
	uint8_t current_idx = 0;
    uint8_t is_last_anchor = 0;
	uint8_t ret = 0;

	for (int i = 0; i < 10; i++)
    {
        sending_msg[i] = msg_common[i];
    }
	
	led_on(LED_PC9);
		
	if (0 == ANCHOR_ID)
    {
			
		anchor_state = ANCHOR_SEND;

		sending_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
		dwt_writetxdata(sizeof(sending_msg), sending_msg, 0);
		dwt_writetxfctrl(sizeof(sending_msg), 0);
		
		dwt_setrxaftertxdelay(RX_AFTER_TX_DELAY);		

		dwt_setrxtimeout(RX_TIMEOUT);
		dwt_setpreambledetecttimeout(0);
	
		dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
				
    }
		
	while (1)
    {
		
		if (ANCHOR_LISTEN == anchor_state) {
				
			dwt_setpreambledetecttimeout(0);
			dwt_setrxtimeout(RX_TIMEOUT);

			dwt_rxenable(DWT_START_RX_IMMEDIATE);

		}
			
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
		{};
			
		if (anchor_state == ANCHOR_SEND)
        {
            anchor_state = ANCHOR_LISTEN;
        }
		
		if (status_reg & SYS_STATUS_RXFCG)
		{
			
			err_num = 0;
			rec_cnt ++;
			
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
		
			if (frame_len <= FRAME_LEN_MAX)
			{
				dwt_readrxdata(rx_buffer, frame_len, 0);								
			}
						
			current_tx = rx_buffer[8];
            frame_seq_nb = rx_buffer[ALL_MSG_SN_IDX];
						
			rx_ts = get_rx_timestamp_u64();
						
			if (current_tx < ANCHOR_ID){
				offset = SINGLE_LEN*current_tx+POA_LEN+4;
            }
            else{
				offset = SINGLE_LEN*(current_tx-1)+POA_LEN+4;
            }
			final_msg_set_ts(&msg_payload[offset], rx_ts);
			
			if (((current_tx + 1) % ANCHOR_NUM == ANCHOR_ID) && !((0 == ANCHOR_ID) && (frame_seq_nb % 2 == 1)))
			{
				uint32_t delay_time;
                uint32_t tx_time;
							
				if (0 == ANCHOR_ID) 
				{ 
					
					delay_time = DELAY_TIME_TURN;					
					frame_seq_nb++;
					
                }
                else 
				{
					delay_time = DELAY_TIME;
                }
								
				tx_time = (rx_ts + (delay_time * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(tx_time);
								
				sending_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				
				dwt_writetxdata(sizeof(sending_msg), sending_msg, 0);
                dwt_writetxfctrl(sizeof(sending_msg), 0); 
								
				dwt_setrxtimeout(RX_TIMEOUT);
								
                dwt_setrxaftertxdelay(RX_AFTER_TX_DELAY); 
                
				
				if ((ANCHOR_ID + 1 == ANCHOR_NUM)&&(frame_seq_nb % 2 == 1))
				{
					ret = dwt_starttx(DWT_START_TX_DELAYED);
					is_last_anchor = 1;
				}
				else
				{
					ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
					anchor_state = ANCHOR_SEND;
				}
				
				if (ANCHOR_NUM - 1 == ANCHOR_ID){
					is_last_anchor = 1;
                }
								
			}

			uint8 temp[5];
			dwt_readfromdevice(RX_TTCKO_ID, 0, RX_TTCKO_LEN, temp);
						
			phase_cal = temp[4];
			maxGC = dwt_read16bitoffsetreg(RX_FQUAL_ID, 0x6);
			rxPC = (dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT;
			
			uint16 fp_index = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET) >> 6;
			dwt_readaccdata(cir_buffer, CIR_LEN*4+1, (fp_index)*4);
			
			if (current_tx < ANCHOR_ID){
				memcpy(msg_payload+SINGLE_LEN*current_tx, cir_buffer+1+4, 4);
			}
			else{
				memcpy(msg_payload+SINGLE_LEN*(current_tx-1), cir_buffer+1+4, 4);
			}
			
			if (current_tx < ANCHOR_ID){
				msg_payload[SINGLE_LEN*current_tx+4] = phase_cal;
			}
			else{
				msg_payload[SINGLE_LEN*(current_tx-1)+4] = phase_cal;
			}
			
			if (current_tx < ANCHOR_ID){
				msg_payload[SINGLE_LEN*current_tx+5] = (uint8_t)rxPC;
			}
			else{
				msg_payload[SINGLE_LEN*(current_tx-1)+5] = (uint8_t)rxPC;
			}
			
			if (current_tx < ANCHOR_ID){
				msg_payload[SINGLE_LEN*current_tx+6] = (uint8_t)(maxGC >> 8);
				msg_payload[SINGLE_LEN*current_tx+7] = (uint8_t)maxGC;
			}
			else{
				msg_payload[SINGLE_LEN*(current_tx-1)+6] = (uint8_t)(maxGC >> 8);
				msg_payload[SINGLE_LEN*(current_tx-1)+7] = (uint8_t)maxGC;
			}
			
			if ((current_tx + 1 == ANCHOR_NUM) || ((current_tx + 2 == ANCHOR_NUM) && (ANCHOR_ID + 1 == ANCHOR_NUM) && is_last_anchor))
			{
								
				is_last_anchor = 0;
				
				for (int i = 10; i < sizeof(sending_msg); i++)
				{
					sending_msg[i] = msg_payload[i-10];
				}
				
				memset(msg_payload,0,(ANCHOR_NUM-1)*SINGLE_LEN + END_LEN+4);
				
				if (frame_seq_nb % 2 == 1 && anchor_state != ANCHOR_SEND)
				{
					
					if (ANCHOR_ID + 1 == ANCHOR_NUM)
					{
						while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_TXFRS)))
						{};
					}
				
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
					
					if (ANCHOR_ID + 1 == ANCHOR_NUM)
					{
						
						dwt_setpreambledetecttimeout(0);
						dwt_setrxtimeout(RX_TIMEOUT);

						dwt_rxenable(DWT_START_RX_IMMEDIATE);
						anchor_state = ANCHOR_SEND;
					}
					
					if (0 == ANCHOR_ID)
					{

						uint32_t delay_time;
						uint32_t tx_time;
						
						delay_time = DELAY_TIME_TURN + 600;
						frame_seq_nb++;
						
						tx_time = (rx_ts + (delay_time * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(tx_time);
										
						sending_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
						
						dwt_writetxdata(sizeof(sending_msg), sending_msg, 0);
						dwt_writetxfctrl(sizeof(sending_msg), 0); 
										
						dwt_setrxtimeout(RX_TIMEOUT);
										
						dwt_setrxaftertxdelay(RX_AFTER_TX_DELAY); 
						
						ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
						anchor_state = ANCHOR_SEND;

					}
					
				}
				
			}
		}
		else
		{
				
			err_num++;
					
			if (0 == ANCHOR_ID)
            {
				
				if (err_num>=1)
				{
					
					rec_cnt = 0;
					err_num = 0;
					frame_seq_nb = 0;
					
					current_freq = 1;
						
					dwt_forcetrxoff();
					dwt_configure(&config);
					dwt_configuretxrf(&txconfig2);
					
				}
				else
				{
					HalDelay_nMs(3);
				}

				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
				
                anchor_state = ANCHOR_SEND;
							
                sending_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(sending_msg), sending_msg, 0);
                dwt_writetxfctrl(sizeof(sending_msg), 0);
   
				dwt_setrxaftertxdelay(RX_AFTER_TX_DELAY);		

				dwt_setrxtimeout(RX_TIMEOUT);
				dwt_setpreambledetecttimeout(0);
  
                dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

            }
			else
			{
				if (err_num>=1)
				{
					
					rec_cnt = 0;
					err_num = 0;
					
					current_freq = 1;
						
					dwt_forcetrxoff();
					dwt_configure(&config);
					dwt_configuretxrf(&txconfig2);
					
				}
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			}
			
						
		}
		
		
		if (n>10)
		{
			HalUsbWrite(usbVCOMout, n);
			n = 0;
		}
				
	}
	
}

#endif