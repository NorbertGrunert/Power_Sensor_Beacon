/*
 * TL182 Tracker Firmware
 *
 * RTC header file
 *
 */
 
#ifndef RTC_H
#define RTC_H


/* Public function prototypes. */
/* RTC init. */
extern void vRTCInit( void );

/* Start the RTC timer. */
extern void vRTCStart( void );

/* Read the RTC time stamp value as 16-bit value. */
extern unsigned short usReadRTC( void );

/* Read the RTC time stamp value as 32-bit value. */
extern unsigned long ulReadRTC( void );
/*-----------------------------------------------------------*/

#endif