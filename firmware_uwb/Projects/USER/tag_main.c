/*! ------------------------------------------------------------------------------------------------------------------
 * @file tag_main.c
 * @brief Tag scheduling main loop. The UWB tag keeps on overhearing UWB packets sent by anchors and send TO/AO estimtes
 * to the PC over UWB.
 *
 * @author Junqi Ma
 * @date 2024.7.13
 *--------------------------------------------------------------------------------------------------------------------
 */

 /*Includes*/
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

static uint8_t cir_buffer[(4 * CIR_LEN + 1) * ANCHOR_NUM];

static uint64_t rx_ts[ANCHOR_NUM];


/* Payload format (CIR and receiving timestamps of messages from other anchors)

|---CIR(4 byte)---|--RxTime(5 byte)--|******|---CIR(4 byte)---|--RxTime(5 byte)--|--TxTime(5 byte)--|-IDX(1 byte)-|
|---------------------------13 byte * (Anchor Number-1)--------------------------|--------------6 byte------------| 
*/
static uint8_t msg_buffer[((ANCHOR_NUM - 1) * SINGLE_LEN + END_LEN) * ANCHOR_NUM];

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

static int n = 0;
static uint16 fp_index;
static uint16_t current_tx;

static uint16_t offset1;
static uint16_t offset2;

static uint16_t current_freq = 1;

static int32 ci[ANCHOR_NUM];

/* Phase compenstation value measured using SFD */
uint8_t phase_cal[ANCHOR_NUM];

/* RSSI value for timestamp compensation */
uint16_t maxGC[ANCHOR_NUM];
uint8_t rxPC[ANCHOR_NUM];

void dw_init(void)
{
	reset_DW1000();

	// Config the SPI speed to 2 MHz
	SPI_ConfigFastRate(SPI_BaudRatePrescaler_32);

	// DW1000 Initialization
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
	{
		while (1)
		{
		};
	}

	dwt_loadopsettabfromotp(DWT_OPSET_TIGHT);

	// Config the SPI speed to 18 MHz
	SPI_ConfigFastRate(SPI_BaudRatePrescaler_4);

	// Config the RF parameters
	dwt_configure(&config);
	dwt_configuretxrf(&txconfig2);

	// Enable the DW1000 TX/RX indicator LED
	dwt_setleds(1);

	// Configure network ID
	dwt_setpanid(NET_PANID);

	// Configure the 16bit short address
	dwt_setaddress16(1);

	// Configure the TX and RX antenna delay
	dwt_setrxaftertxdelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);
}

int dw_main(void)
{

	// Initialization
	dw_init();

	// The index of anchor that is currently sending message
	uint8_t current_idx = 0;
	// True or False
	uint8_t is_last_anchor = 0;
	uint8_t ret = 0;

	// The main loop
	while (1)
	{

		dwt_setpreambledetecttimeout(0);
		/* Clear reception timeout to start next ranging process. */
		dwt_setrxtimeout(0);

		/* Activate reception immediately. */
		dwt_rxenable(DWT_START_RX_IMMEDIATE);

		// Waiting for reception completion
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
		{};

		if (status_reg & SYS_STATUS_RXFCG)
		{

			// Read the frame length from DW register
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;

			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

			if (frame_len <= FRAME_LEN_MAX)
			{
				// Read data from RX buffer
				dwt_readrxdata(rx_buffer, frame_len, 0);
			}

			current_tx = rx_buffer[8];
			frame_seq_nb = rx_buffer[ALL_MSG_SN_IDX];

			// Copy the AO data from rx_buffer to msg_buffer
			memcpy(msg_buffer + ((ANCHOR_NUM - 1) * SINGLE_LEN + END_LEN) * current_tx, rx_buffer + ALL_MSG_COMMON_LEN, (ANCHOR_NUM - 1) * SINGLE_LEN + END_LEN);

			rx_ts[current_tx] = get_rx_timestamp_u64();

			// Read the first path CIR and save it to local buffer
			fp_index = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET) >> 6;
			dwt_readaccdata(cir_buffer + current_tx * (CIR_LEN * 4 + 1), CIR_LEN * 4 + 1, (fp_index) * 4);

			// Read Carrier integer for clock drift estimation
			ci[current_tx] = dwt_readcarrierintegrator();

			// Read RCPHASE for phase compensation
			dwt_readfromdevice(RX_TTCKO_ID, 4, 1, phase_cal + current_tx);

			// Read max growth cir and rxPC for RSSI estimation
			maxGC[current_tx] = dwt_read16bitoffsetreg(RX_FQUAL_ID, 0x6);
			rxPC[current_tx] = (dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT;

			// Ranging finished, send the ranging data to SoC/PC over USB
			if (ANCHOR_NUM == current_tx + 1)
			{

				int xx = 0;

				// AO estimates acquired from anchor i
				for (uint8_t i = 0; i < ANCHOR_NUM; i++)
				{

					// Offset1 is the byte offset
					offset1 = ((ANCHOR_NUM - 1) * SINGLE_LEN + END_LEN) * i;

					// AO estimates of anchor j acquired from anchor i
					for (uint8_t j = 0; j < ANCHOR_NUM; j++)
					{
						uint64_t rx_time;

						if (j < i)
						{
							offset2 = j * SINGLE_LEN;
						}
						else if (j > i)
						{
							offset2 = (j - 1) * SINGLE_LEN;
						}
						else if (j == i)
						{
							continue;
						}

						/* 4 byte CIR + 1 byte RPHASE + 1 byte rxPC + 2 byte maxGC + 5 byte rx_time */
						memcpy(usbVCOMout + n, msg_buffer + offset1 + offset2, SINGLE_LEN);
						n += SINGLE_LEN;
					}

					usbVCOMout[n] = frame_seq_nb;
					n += 1;
				}

				// Then TO estimates
				for (uint8_t i = 0; i < ANCHOR_NUM; i++)
				{

					// CIR
					memcpy(usbVCOMout + n, cir_buffer + (4 * CIR_LEN + 1) * i + 1 + 4, 4);
					n += 4;

					// Phase correction value
					usbVCOMout[n] = phase_cal[i];
					n += 1;

					usbVCOMout[n] = rxPC[i];
					n += 1;

					usbVCOMout[n] = (uint8_t)(maxGC[i] >> 8);
					usbVCOMout[n + 1] = (uint8_t)(maxGC[i]);
					n += 2;

					// Receiving timestamps
					for (int k = 0; k < 5; k++)
					{
						usbVCOMout[n] = (uint8_t)(rx_ts[i] >> ((4 - k) * 8));
						n += 1;
					}

					// Carrier integer number
					for (int k = 0; k < 4; k++)
					{
						usbVCOMout[n] = (uint8_t)(ci[i] >> ((3 - k) * 8));
						n += 1;
					}
				}

				// Frame sequence number
				usbVCOMout[n] = frame_seq_nb;
				n += 1;

				// Used for data segmentation: 7,6,5,4,3
				for (int i = 0; i < 5; i++)
				{
					usbVCOMout[n] = 7 - i;
					n += 1;
				}

				HalUsbWrite(usbVCOMout, n);
				n = 0;

				// Perform frequence hooping every two round of localization
				if (frame_seq_nb % 2 == 1)
				{
					if (frame_seq_nb % 4 == 1)
					{
						
						// Switch to channel 3
						current_freq = 3;

						dwt_forcetrxoff();
						dwt_configure(&config2);
						dwt_configuretxrf(&txconfig3);
					}
					else if (frame_seq_nb % 4 == 3)
					{

						// Switch to channel 1
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