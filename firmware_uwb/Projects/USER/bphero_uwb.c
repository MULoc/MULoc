#include "bphero_uwb.h"
#include "port.h"
#include <math.h>
#include <stdio.h>

int psduLength = 0;
srd_msg_dsss msg_f_send ;
srd_msg_dsss msg_f_send2 ;

uint8 rx_buffer[FRAME_LEN_MAX];
uint32 status_reg = 0;
uint16 frame_len = 0;

/* Default communication configuration */
dwt_config_t config =
{
    1,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_64,   /* Preamble length. */
    DWT_PAC8,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    //DWT_BR_110K,     /* Data rate. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_EXT, /* PHY header mode. */
    (65 + 8 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default communication configuration */
dwt_config_t config2 =
{
    3,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_64,   /* Preamble length. */
    DWT_PAC8,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    //DWT_BR_110K,     /* Data rate. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_EXT, /* PHY header mode. */
    (65 + 8 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

dwt_txconfig_t txconfig1 = 
{
	0xc9,			
	0x17171717  
}; // CH1


dwt_txconfig_t txconfig2 = 
{	
	0xc9,			
	0x15151515,
	
}; // CH2

dwt_txconfig_t txconfig3 = 
{
	0xc5,			// PG Delay
	0x2b2b2b2b,
	
}; // CH3


dwt_txconfig_t txconfig7 = 
{
	0x93,			// PG Delay
	0xD1D1D1D1    	// Power
};

