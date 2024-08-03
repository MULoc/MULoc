#ifndef FRAME_HEADER_H
#define FRAME_HEADER_H

#include "common_header.h"
#include "deca_device_api.h"

#define SIG_RX_UNKNOWN          99      // Received an unknown frame
//DecaRTLS frame function codes
#define RTLS_DEMO_MSG_TAG_POLL              (0x81)          // Tag poll message
#define RTLS_DEMO_MSG_ANCH_RESP             (0x70)          // Anchor response to poll
//#define RTLS_DEMO_MSG_ANCH_POLL             (0x71)          // Anchor to anchor poll message
//#define RTLS_DEMO_MSG_ANCH_RESP2            (0x72)          // Anchor response to poll from anchor
#define RTLS_DEMO_MSG_ANCH_FINAL            (0x73)          // Anchor final massage back to Anchor
#define RTLS_DEMO_MSG_TAG_FINAL             (0x82)          // Tag final massage back to Anchor


//lengths including the Decaranging Message Function Code byte
#define TAG_POLL_MSG_LEN                    20               // FunctionCode(1), Range Num (1)
#define ANCH_RESPONSE_MSG_LEN               8               // FunctionCode(1), Sleep Correction Time (2), Measured_TOF_Time(4), Range Num (1) (previous)
#define TAG_FINAL_MSG_LEN                   33              // FunctionCode(1), Range Num (1), Poll_TxTime(5),
// Resp0_RxTime(5), Resp1_RxTime(5), Resp2_RxTime(5), Resp3_RxTime(5), Final_TxTime(5), Valid Response Mask (1)

#define MAX_MAC_MSG_DATA_LEN                (TAG_FINAL_MSG_LEN) //max message len of the above

#define STANDARD_FRAME_SIZE         127

#define ADDR_BYTE_SIZE_L            (8)
#define ADDR_BYTE_SIZE_S            (2)

#define FRAME_CONTROL_BYTES         2
#define FRAME_SEQ_NUM_BYTES         1
#define FRAME_PANID                 2
#define FRAME_CRC                   2
#define FRAME_SOURCE_ADDRESS_S        (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S          (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L        (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L          (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP                 (FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID) //5
#define FRAME_CRTL_AND_ADDRESS_L    (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP) //21 bytes for 64-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_S    (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //9 bytes for 16-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_LS   (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //15 bytes for 1 16-bit address and 1 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_L-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_S-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_LS-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 15 - 16 - 2 = 94

//NOTE: the user payload assumes that there are only 88 "free" bytes to be used for the user message (it does not scale according to the addressing modes)
//#define MAX_USER_PAYLOAD_STRING MAX_USER_PAYLOAD_STRING_LL

//#define MAX_TAG_LIST_SIZE               (8)
//#define MAX_ANCHOR_LIST_SIZE            (4) //this is limited to 4 in this application
//#define NUM_EXPECTED_RESPONSES          (3) //e.g. MAX_ANCHOR_LIST_SIZE - 1
//#define NUM_EXPECTED_RESPONSES_ANC      (1) //anchors A0, A1 and A2 are involved in anchor to anchor ranging
//#define NUM_EXPECTED_RESPONSES_ANC0     (2) //anchor A0 expects response from A1 and A2

//#define GATEWAY_ANCHOR_ADDR             (0x8000)
//#define A1_ANCHOR_ADDR                  (0x8001)
//#define A2_ANCHOR_ADDR                  (0x8002)
//#define A3_ANCHOR_ADDR                  (0x8003)


//application data message byte offsets
//#define FCODE                               0               // Function code is 1st byte of messageData
//#define PTXT                                2               // Poll TX time
//#define RRXT0                               7               // A0 Response RX time
//#define RRXT1                               12              // A1 Response RX time
//#define RRXT2                               17              // A2 Response RX time
//#define RRXT3                               22              // A3 Response RX time
//#define FTXT                                27              // Final TX time
//#define VRESP                               32              // Mask of valid response times (e.g. if bit 1 = A0's response time is valid)
//#define RES_TAG_SLP0                        1               // Response tag sleep correction LSB
//#define RES_TAG_SLP1                        2               // Response tag sleep correction MSB
//#define TOFR                                3               // ToF (n-1) 4 bytes
//#define TOFRN                               7               // range number 1 byte
//#define POLL_RNUM                           1               // Poll message range number

//this it the delay used for configuring the receiver on delay (wait for response delay)
//NOTE: this RX_RESPONSE_TURNAROUND is dependent on the microprocessor and code optimisations
//#define RX_RESPONSEX_TURNAROUND (50) //takes about 100 us for response to come back
//#define RX_RESPONSE1_TURNAROUND (200) //takes about 200 us for the 1st response to come back (from A0)
//#define RX_RESPONSE1_TURNAROUND_6M81 (300) //takes about 100 us for response to come back
//#define RX_RESPONSE1_TURNAROUND_110K (300) //takes about 100 us for response to come back


// This file defines data and functions for access to Parameters in the Device
//message structure for Poll, Response and Final message

typedef struct
{
    uint8 frameCtrl[2];                             //  frame control bytes 00-01
    uint8 seqNum;                                   //  sequence_number 02
    uint8 panID[2];                                 //  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];               //  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];             //  13-20 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LL] ; //  22-124 (application data and any user payload)
    uint8 fcs[2] ;                                  //  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlsl ;

typedef struct
{
    uint8 frameCtrl[2];                             //  frame control bytes 00-01
    uint8 seqNum;                                   //  sequence_number 02
    uint8 panID[2];                                 //  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];               //  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];             //  07-08
    uint8 messageData[MAX_USER_PAYLOAD_STRING_SS] ; //  09-124 (application data and any user payload)
    uint8 fcs[2] ;                                  //  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dsss ;

typedef struct
{
    uint8 frameCtrl[2];                             //  frame control bytes 00-01
    uint8 seqNum;                                   //  sequence_number 02
    uint8 panID[2];                                 //  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];               //  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];             //  13-14
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                                  //  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlss ;

typedef struct
{
    uint8 frameCtrl[2];                             //  frame control bytes 00-01
    uint8 seqNum;                                   //  sequence_number 02
    uint8 panID[2];                                 //  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];               //  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];             //  07-14 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                                  //  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dssl ;


#endif
