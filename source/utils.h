/*
 * Tracker Firmware
 *
 * Utility functions
 *
 */
#ifndef UTILS_H
#define UTILS_H

#include "FreeRTOS.h"
#include "queue.h"

/* Convert a 4-bit value into a HEX-character. */
extern signed char cNibbleToChar( signed char cNibble );

/* Convert a HEX-character ('0'- '9', 'A' - 'F') into a 4-bit value. */
extern signed char cCharToNibble( signed char cChar );

/* Convert a 8-bit value into a string of HEX-characters including end-of-string 0x0. */
extern void vByteToHexStrg( signed char *pcStrg, unsigned char ucValue );

/* Convert a string of HEX-characters into an 8-bit value. */
extern unsigned char ucHexStrgToByte( signed char *pcStrg );

/* Convert a 16-bit value into a string of HEX-characters including end-of-string 0x0. */
extern void vShortToHexStrg( signed char *pcStrg, unsigned short usValue );

/* Convert a 16-bit value into a string of unsigned integer characters including end-of-string 0x0. */
extern void vShortToIntStrg( signed char *pcDest, unsigned short usValue, bool bSuppressLeadingZ );

/* Convert a string of HEX-characters in little endian in the UART Rx buffer into an 32-bit value. */
extern unsigned long ulHexStrgToLongLE( unsigned portBASE_TYPE uxStrgIdx );

/* Convert a string of HEX-characters into a 16-bit value. */
extern unsigned short usHexStrgToShort( signed char *pcStrg );

/* Convert a 32-bit value into a string of HEX-characters including end-of-string 0x0. */
extern void vLongToHexStrg( signed char *pcStrg, unsigned long ulValue );

/* Convert a string of HEX-characters into a 32-bit value. */
extern unsigned long ulHexStrgToLong( signed char *pcStrg );

/* Convert a a string of unsigned integer characters including end-of-string 0x0 into a 16-bit value. 
   The maximum size of the string is 5 characters (xLen). */
extern unsigned short usIntStrgToShort( signed char *pcStrg, portBASE_TYPE	xLen );

/* Return RAM string length. */
extern unsigned char ucStrgRAMLen( signed char *pcChar );

// /* Return ROM string length. */
// extern unsigned char ucStrgROMLen( PGM_P pcChar );

// /* Return EEPROM string length. */
// extern unsigned char ucStrgEEPROMLen( signed char *pcChar );

// /* Copy a striong from ROM to RAM. */
// extern void vStrcpyROM2RAM( signed char *pcRAMChar, PGM_P pcROMChar, bool bTranslateApostr );

/* Return pointer to the character after the next comma ','  in a string. */
extern signed char *pcFindComma( signed char *pcChar, unsigned portBASE_TYPE uxMaxChar );

/* Convert double quotes in RAM string to single quotes. */
extern void vTranslateDoubleQuote( signed char *pcChar );

/* Convert single quotes in RAM string to double quotes. */
extern void vTranslateSingleQuote( signed char *pcChar );

/* Remove all messages of a specific type from a queue. All other messages remain in the 
   queue in the same order as before.
*/
extern void vRemoveTypeFromQueue( QueueHandle_t xQueue, unsigned portBASE_TYPE xMessageType );

/* Remove all messages which are not server commands from a queue. Server command messages remain in the 
   queue in the same order as before.
*/
extern void vRemoveAllMsgsFromQueue( QueueHandle_t xQueue );

/* Check, if a message of a specific type is in a queue. */
extern bool bMsgTypeInQueue( QueueHandle_t xQueue, unsigned portBASE_TYPE xMessageType );

/* Send a message to the specified queue if it is not already in there. */
extern BaseType_t xSendUniqueMsgToQueue( QueueHandle_t xQueue, unsigned portBASE_TYPE *xMessage, TickType_t xTicksToWait );

// /* Short delay outside the scheduler to generate a delay. */
// extern void vDelay( unsigned short ucMaxCnt );

// /* Hard-reset the system. */
// extern void vSystemReset( void );

#endif