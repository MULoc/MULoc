/*
 ********************************************************************************
 *
 *                                 osal.h
 *
 * File          : osal.h
 * Version       : V1.0
 * Author        : tony
 * Mode          : Thumb2
 * Toolchain     : 
 * Description   : 
 *                
 * History       :
 * Date          : 2013.07.22
 *******************************************************************************/
 
#ifndef __OSAL_H_
#define __OSAL_H_

#ifdef __cplusplus
extern "C"
}
#endif

/**************************************************************************************************
 * 																				INCLUDES
 **************************************************************************************************/	
#include <stdbool.h>
#include "OSAL_Comdef.h"

	

/**************************************************************************************************
 * 																				CONSTANTS
 **************************************************************************************************/	
#define ST_CHIP

#define QUEUE_MSG_LEN 128
#define QUEUE_MSG_MAX 3




#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define BUILD_UINT16(loByte, hiByte) \
          ((u16)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((u32)((u32)((Byte0) & 0x00FF) \
          + ((u32)((Byte1) & 0x00FF) << 8) \
          + ((u32)((Byte2) & 0x00FF) << 16) \
          + ((u32)((Byte3) & 0x00FF) << 24)))


/***************************************************************************************************
 * 																				TYPEDEF
 ***************************************************************************************************/
typedef struct{
	uint8_t flag;
	uint8_t len;
	uint8_t buf[QUEUE_MSG_LEN];
}Message;


#ifdef ST_CHIP
	typedef struct queue 
	{
		Message pMsg[QUEUE_MSG_MAX];
		int front;    
		int rear;    
		int maxsize; 
	}QUEUE,*PQUEUE;
#else	
	typedef struct queue 
	{
		Message *pMsg;
		int front;    
		int rear;    
		int maxsize; 
	}QUEUE,*PQUEUE;	
#endif
/***************************************************************************************************
 * 																				GLOBAL VARIABLES
 ***************************************************************************************************/


/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/	
extern void osal_CreateQueue(PQUEUE Q,int maxsize);	
extern void osal_TraverseQueue(PQUEUE Q);						
extern bool osal_FullQueue(PQUEUE Q);								
extern bool osal_EmptyQueue(PQUEUE Q);							
extern bool osal_Enqueue(PQUEUE Q, Message val);		
extern bool osal_Dequeue(PQUEUE Q, Message *val);		

extern void osal_itoa (unsigned int n,char s[]);
extern void osal_Str2Byte(const char* source, uint8_t* dest, int sourceLen);  
extern void osal_Hex2Str( const char *sSrc,  char *sDest, int nSrcLen );


extern uint8_t osal_init_system( void );
extern void osal_start_system( void );
extern void osal_run_system( void );

#ifdef __cplusplus
}
#endif
 
#endif//__OSAL_H_
