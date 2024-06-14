/*
 * Tracker Firmware
 *
 * RTC
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

#include "tracker.h"

/* nRF SDK files. */
#define NRF_LOG_MODULE_NAME 		RTCLK
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_rtc.h"

/* Scheduler include files. */
#include "FreeRTOS.h"

/* Device specific include files. */
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* RTC overflow callback function. */
void prvRTCOverflowCallback( void );

/* RTC init. */
void vRTCInit( void );

/* Read the RTC time stamp value as short. */
unsigned short usReadRTC( void );

/* Read the RTC time stamp value as long. */
unsigned long ulReadRTC( void );
/*-----------------------------------------------------------*/

/* Global variables. */
/*-----------------------------------------------------------*/

/* Module scope variables. */
/* RTC overflow counter. */
static volatile	unsigned long	ulRTCOverflowCnt;
/*-----------------------------------------------------------*/

/* Create timer. */
void vRTCInit( void )
{
	ulRTCOverflowCnt = 0;
	vRegisterOvflCB( prvRTCOverflowCallback );
	
	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();	
}
/*-----------------------------------------------------------*/

/* RTC callback. */
void prvRTCOverflowCallback( void )
{
	/* Increment the RTC overflow counter. */
	ulRTCOverflowCnt++;
}
/*-----------------------------------------------------------*/

/* Read the RTC time stamp value as 32-bit value. */
unsigned long ulReadRTC( void )
{
	volatile unsigned long long 	ullRTC;
	
	/* Make a local copy of the RTC value. */

	portENTER_CRITICAL();
	/* Get the HW RTC count. */
	ullRTC = ( unsigned long long )nrf_rtc_counter_get( portNRF_RTC_REG );
	/* Divide by 32768 to convert the counter to seconds. */
	ullRTC >>= 10;
	/* Add the overflow counter. */
	ullRTC += ( ulRTCOverflowCnt << 14 );
	portEXIT_CRITICAL();
	
	return ( unsigned long )( ullRTC & 0xffffffff );
}
/*-----------------------------------------------------------*/

/* Read the RTC time stamp value as 16-bit value. */
unsigned short usReadRTC( void )
{
	return ( unsigned short )( ulReadRTC() & 0x0000ffff );
}
/*-----------------------------------------------------------*/