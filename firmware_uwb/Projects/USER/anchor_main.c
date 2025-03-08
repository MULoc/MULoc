/*! ------------------------------------------------------------------------------------------------------------------
 * @file tag_main.c
 * @brief Anchor scheduling main loop. The anchor keeps on broadcasting UWB packets and overhearing UWB packets sent 
 * by other anchors.
 *
 * @author Junqi Ma
 * @date 2024.7.13
 *--------------------------------------------------------------------------------------------------------------------
 */

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

/* Payload format (CIR and receiving timestamps of messages from other anchors)

|---CIR(4 byte)---|--RxTime(5 byte)--|******|---CIR(4 byte)---|--RxTime(5 byte)--|--TxTime(5 byte)--|-IDX(1 byte)-|
|---------------------------13 byte * (Anchor Number-1)--------------------------|--------------6 byte------------| 
*/
static uint8_t msg_payload[(ANCHOR_NUM - 1) * SINGLE_LEN + END_LEN + 4];

static uint8_t sending_msg[sizeof(msg_common) + sizeof(msg_payload)];

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Timestamps of frames transmission/reception. */
static uint64_t rx_ts;
static uint64_t tx_ts;

static uint8_t cir_buffer[4 * CIR_LEN + 1];

static uint16_t current_freq = 1;

// Anchor 0 start error number
static uint16_t err_num = 0;

// Anchor 0 start error number
static uint16_t rec_cnt = 0;

static int n = 0;

/* Phase value measured from SFD */
uint8 phase_cal;

uint16_t maxGC;
uint8_t rxPC;

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

	dwt_xtaltrim(16);
	dwt_loadopsettabfromotp(DWT_OPSET_TIGHT);

	// Config the SPI speed to 18 MHz
	SPI_ConfigFastRate(SPI_BaudRatePrescaler_4);

	// Config the RF parameters
	dwt_configure(&config);

	dwt_setsmarttxpower(1);
	dwt_configuretxrf(&txconfig2);

	// Enable the DW1000 TX/RX indicator LED
	dwt_setleds(1);

	// Configure network ID
	dwt_setpanid(NET_PANID);

	// Configure the 16bit short address
	dwt_setaddress16(1);

	// Configure the TX and RX antenna delay
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	dwt_setinterrupt(DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);

	msg_f_send.sourceAddr[0] = 1 & 0xFF;
	msg_f_send.sourceAddr[1] = (1 >> 8) & 0xFF;
}

int dw_main(void)
{
	/* Configure the source address */
	msg_common[8] = ANCHOR_ID;

	// Initialization
	dw_init();

	// The index of anchor that is currently sending message
	uint8_t current_idx = 0;
	// True or False
	uint8_t is_last_anchor = 0;
	uint8_t ret = 0;

	// Write the common segments of UWB message into tx buffer
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

		// Start the ranging session from anchor 0
		dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
	}

	// The main loop
	while (1)
	{

		// If the anchor is in the listening state
		if (ANCHOR_LISTEN == anchor_state)
		{

			dwt_setpreambledetecttimeout(0);
			/* Clear reception timeout to start next ranging process. */
			dwt_setrxtimeout(RX_TIMEOUT);
			/* Activate reception immediately. */
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}

		// Waiting for reception completion
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
		{};

		// If the anchor have received the message, then change it to listening state
		if (anchor_state == ANCHOR_SEND)
		{
			anchor_state = ANCHOR_LISTEN;
		}

		if (status_reg & SYS_STATUS_RXFCG)
		{

			err_num = 0;
			rec_cnt++;

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

			rx_ts = get_rx_timestamp_u64();

			// Copy the receiving timestamps into buffer
			if (current_tx < ANCHOR_ID)
			{
				offset = SINGLE_LEN * current_tx + POA_LEN + 4;
			}
			else
			{
				// UWB anchors cannot receive the messages sent by themself
				offset = SINGLE_LEN * (current_tx - 1) + POA_LEN + 4;
			}
			final_msg_set_ts(&msg_payload[offset], rx_ts);

			// It's our turn to send a message. We should send later if we are anchor 0 and it's time for hooping
			if (((current_tx + 1) % ANCHOR_NUM == ANCHOR_ID) && !((0 == ANCHOR_ID) && (frame_seq_nb % 2 == 1)))
			{
				// Time delay between the messages from different anchors
				uint32_t delay_time;
				uint32_t tx_time;

				// If we are anchor 0, the frame number should +1 and the delay time should be set to DELAY_TIME_TURN
				if (0 == ANCHOR_ID)
				{

					delay_time = DELAY_TIME_TURN;
					frame_seq_nb++;
				}
				else
				{
					delay_time = DELAY_TIME;
				}

				// The tx time is set to "delay_time" seconds after the reception of the last message using delayed transmission
				tx_time = (rx_ts + (delay_time * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(tx_time);

				// Config the frame sequence number
				sending_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

				// Write tx data into tx buffer
				dwt_writetxdata(sizeof(sending_msg), sending_msg, 0);
				dwt_writetxfctrl(sizeof(sending_msg), 0);

				dwt_setrxtimeout(RX_TIMEOUT);

				// Enable rx RX_AFTER_TX_DELAY after the transmission
				dwt_setrxaftertxdelay(RX_AFTER_TX_DELAY);

				if ((ANCHOR_ID + 1 == ANCHOR_NUM) && (frame_seq_nb % 2 == 1))
				{
					// If we are the last anchor and should perform hopping, we should not start reception before hopping.
					ret = dwt_starttx(DWT_START_TX_DELAYED);
					is_last_anchor = 1;
				}
				else
				{
					ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
					anchor_state = ANCHOR_SEND;
				}

				if (ANCHOR_NUM - 1 == ANCHOR_ID)
				{
					is_last_anchor = 1;
				}
			}

			uint8 temp[5];
			dwt_readfromdevice(RX_TTCKO_ID, 0, RX_TTCKO_LEN, temp);

			phase_cal = temp[4];
			maxGC = dwt_read16bitoffsetreg(RX_FQUAL_ID, 0x6);
			rxPC = (dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT;

			// Read the first path CIR and save it to local buffer
			uint16 fp_index = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET) >> 6;
			dwt_readaccdata(cir_buffer, CIR_LEN * 4 + 1, (fp_index) * 4);

			// Copy CIR to msg_payload
			if (current_tx < ANCHOR_ID)
			{
				memcpy(msg_payload + SINGLE_LEN * current_tx, cir_buffer + 1 + 4, 4);
			}
			else
			{
				memcpy(msg_payload + SINGLE_LEN * (current_tx - 1), cir_buffer + 1 + 4, 4);
			}

			// Copy phase_cal to msg_payload
			if (current_tx < ANCHOR_ID)
			{
				msg_payload[SINGLE_LEN * current_tx + 4] = phase_cal;
			}
			else
			{
				msg_payload[SINGLE_LEN * (current_tx - 1) + 4] = phase_cal;
			}

			// Copy rxPC to msg_payload
			if (current_tx < ANCHOR_ID)
			{
				msg_payload[SINGLE_LEN * current_tx + 5] = (uint8_t)rxPC;
			}
			else
			{
				msg_payload[SINGLE_LEN * (current_tx - 1) + 5] = (uint8_t)rxPC;
			}

			// Copy maxGC to msg_payload
			if (current_tx < ANCHOR_ID)
			{
				msg_payload[SINGLE_LEN * current_tx + 6] = (uint8_t)(maxGC >> 8);
				msg_payload[SINGLE_LEN * current_tx + 7] = (uint8_t)maxGC;
			}
			else
			{
				msg_payload[SINGLE_LEN * (current_tx - 1) + 6] = (uint8_t)(maxGC >> 8);
				msg_payload[SINGLE_LEN * (current_tx - 1) + 7] = (uint8_t)maxGC;
			}

			// All messages have been sent
			if ((current_tx + 1 == ANCHOR_NUM) || ((current_tx + 2 == ANCHOR_NUM) && (ANCHOR_ID + 1 == ANCHOR_NUM) && is_last_anchor))
			{

				// The last anchor should write buffer after transmission
				is_last_anchor = 0;

				// Copy the payload to the sending buffer
				for (int i = 10; i < sizeof(sending_msg); i++)
				{
					// The first 10 bytes in UWB message are MAC
					sending_msg[i] = msg_payload[i - 10];
				}

				memset(msg_payload, 0, (ANCHOR_NUM - 1) * SINGLE_LEN + END_LEN + 4);

				// It's time for hopping
				if (frame_seq_nb % 2 == 1 && anchor_state != ANCHOR_SEND)
				{

					if (ANCHOR_ID + 1 == ANCHOR_NUM)
					{
						// If we are the last anchor, we should not start hopping until the message is successfully sent
						while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_TXFRS)))
						{};
					}

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

					if (ANCHOR_ID + 1 == ANCHOR_NUM)
					{

						dwt_setpreambledetecttimeout(0);
						/* Clear reception timeout to start next ranging process. */
						dwt_setrxtimeout(RX_TIMEOUT);
						// If we are the last anchor, we should start receive after hopping
						dwt_rxenable(DWT_START_RX_IMMEDIATE);
						anchor_state = ANCHOR_SEND;
					}

					if (0 == ANCHOR_ID)
					{
						// If we are anchor 0, we should start transmission after hopping

						// Time delay between the messages from different anchors
						uint32_t delay_time;
						uint32_t tx_time;

						delay_time = DELAY_TIME_TURN + 600;
						frame_seq_nb++;

						// The tx time is set to "delay_time" seconds after the reception of the last message using delayed transmission
						tx_time = (rx_ts + (delay_time * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(tx_time);

						// Config the frame sequence number
						sending_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

						// Write tx data into tx buffer
						dwt_writetxdata(sizeof(sending_msg), sending_msg, 0);
						dwt_writetxfctrl(sizeof(sending_msg), 0);

						dwt_setrxtimeout(RX_TIMEOUT);

						// Enable rx RX_AFTER_TX_DELAY after the transmission
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

			// Handling packet loss
			if (0 == ANCHOR_ID)
			{

				// Hooping Now! If received more than ANCHOR_NUM-2 messages.
				if (err_num >= 1)
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

				// Restart from anchor 0 if any packet is lost
				anchor_state = ANCHOR_SEND;

				sending_msg[ALL_MSG_SN_IDX] = frame_seq_nb; 
				dwt_writetxdata(sizeof(sending_msg), sending_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(sending_msg), 0);/* Zero offset in TX buffer, ranging. */

				dwt_setrxaftertxdelay(RX_AFTER_TX_DELAY);

				dwt_setrxtimeout(RX_TIMEOUT);
				dwt_setpreambledetecttimeout(0);

				// Start transmission immediately
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
			}
			else
			{
				// Hopping Now! If received more than ANCHOR_NUM-2 messages
				if (err_num >= 1)
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

		if (n > 10)
		{
			HalUsbWrite(usbVCOMout, n);
			n = 0;
		}
	}
}

#endif