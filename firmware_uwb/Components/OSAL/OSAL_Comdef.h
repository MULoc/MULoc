/*
********************************************************************************
*
*                                 OSAL_Comdef.h
*
* File          : OSAL_Comdef.h
* Version       : V1.0
* Author        : tony
* Mode          : 
* Toolchain     : 
* Description   : 
*                
* History       :
* Date          : 2013.07.22
*******************************************************************************/

#ifndef __OSAL_COMDEF_H_
#define __OSAL_COMDEF_H_

#ifdef __cplusplus
extern "C"
}
#endif

#include "stdint.h"

#define HW_PA
#define xHW_RELEASE
#define HW_BASE

//typedef unsigned char uint8;
//typedef char int8;

//typedef unsigned short uint16;
//typedef short int16;

//typedef unsigned int uint32;
//typedef int int32;

//typedef volatile uint32 v_uint32;


/* ------------------------------------------------------------------------------------------------
 *                                             Macros
 * ------------------------------------------------------------------------------------------------
 */

#ifndef BV
#define BV(n)      (1 << (n))
#endif

#ifndef BF
#define BF(x,b,s)  (((x) & (b)) >> (s))
#endif

#ifndef MIN
#define MIN(n,m)   (((n) < (m)) ? (n) : (m))
#endif

#ifndef MAX
#define MAX(n,m)   (((n) < (m)) ? (m) : (n))
#endif

#ifndef ABS
#define ABS(n)     (((n) < 0) ? -(n) : (n))
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


#ifndef NULL
#define NULL 0
#endif

/* takes a byte out of a uint32 : var - uint32,  ByteNum - byte to take out (0 - 3) */
#define BREAK_UINT32( var, ByteNum ) \
          (uint8_t)((uint32_t)(((var) >>((ByteNum) * 8)) & 0x00FF))

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((uint32_t)((uint32_t)((Byte0) & 0x00FF) \
          + ((uint32_t)((Byte1) & 0x00FF) << 8) \
          + ((uint32_t)((Byte2) & 0x00FF) << 16) \
          + ((uint32_t)((Byte3) & 0x00FF) << 24)))

#define BUILD_UINT16(loByte, hiByte) \
          ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define BUILD_UINT8(hiByte, loByte) \
          ((uint8_t)(((loByte) & 0x0F) + (((hiByte) & 0x0F) << 4)))

#define HI_UINT8(a) (((a) >> 4) & 0x0F)
#define LO_UINT8(a) ((a) & 0x0F)

// Write the 32bit value of 'val' in little endian format to the buffer pointed 
// to by pBuf, and increment pBuf by 4
#define UINT32_TO_BUF_LITTLE_ENDIAN(pBuf,val) \
	do { \
		*(pBuf)++ = ((((uint32_t)(val)) >>  0) & 0xFF); \
		*(pBuf)++ = ((((uint32_t)(val)) >>  8) & 0xFF); \
		*(pBuf)++ = ((((uint32_t)(val)) >> 16) & 0xFF); \
		*(pBuf)++ = ((((uint32_t)(val)) >> 24) & 0xFF); \
	} while (0)

// Return the 32bit little-endian formatted value pointed to by pBuf, and increment pBuf by 4
#define BUF_TO_UINT32_LITTLE_ENDIAN(pBuf) (((pBuf) += 4), BUILD_UINT32((pBuf)[-4], (pBuf)[-3], (pBuf)[-2], (pBuf)[-1]))


#ifdef __cplusplus
}
#endif	

#endif//__OSAL_COMDEF_H_
