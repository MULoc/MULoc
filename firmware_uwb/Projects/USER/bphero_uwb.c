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

dwt_config_t config =
{
    1,               
    DWT_PRF_64M,     
    DWT_PLEN_64,   
    DWT_PAC8,      
    9,              
    9,              
    1,              
    DWT_BR_6M8,     
    DWT_PHRMODE_EXT,
    (65 + 8 - 8)
};

dwt_config_t config2 =
{
    3,               
    DWT_PRF_64M,     
    DWT_PLEN_64,   
    DWT_PAC8,      
    9,             
    9,             
    1,             
    DWT_BR_6M8,    
    DWT_PHRMODE_EXT,
    (65 + 8 - 8)
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

