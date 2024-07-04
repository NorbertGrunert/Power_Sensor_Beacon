/*
 * BLE interface device
 *
 * General header file for misc. definitions
 *
 */
 
#ifndef MAIN_H
#define MAIN_H

#include "FreeRTOS.h"
#include "nrf_drv_wdt.h"

#define VERSION						"NINA-B3"

#define ERROR_STACK_DUMP_LEN		( 20 )			/* Length of stack dumped to persistent RAM in reset case. */

#define RAM_START					( 0x20000000 )
#define RAM_END						( 0x20040000 )


/* Set the base confguration for NRF logging. 
   Lower individual module severity level:
		https://devzone.nordicsemi.com/f/nordic-q-a/18307/logger-module-config-question-sdk12 */
#ifndef TL_CONFIG_LOG_ENABLED
	#define TL_CONFIG_LOG_ENABLED 1
	
			/* <0=> Off 		*/
			/* <1=> Error       */
			/* <2=> Warning     */
			/* <3=> Info        */
			/* <4=> Debug       */
	#define TL_CONFIG_LOG_LEVEL		4
	
			/* <0=> Default 	*/
			/* <1=> Black       */
			/* <2=> Red         */
			/* <3=> Green       */
			/* <4=> Yellow      */
			/* <5=> Blue        */
			/* <6=> Magenta     */
			/* <7=> Cyan        */
			/* <8=> White 	    */
	#define TL_CONFIG_INFO_COLOR	5
	#define TL_CONFIG_DEBUG_COLOR	5	
#endif

#define NRF_LOG_DEFAULT_OFF			0
#define NRF_LOG_DEFAULT_ERROR		1
#define NRF_LOG_DEFAULT_WARNING		2
#define NRF_LOG_DEFAULT_INFO		3
#define NRF_LOG_DEFAULT_DEBUG		4

#undef  NRF_LOG_DEFAULT_LEVEL
#define NRF_LOG_DEFAULT_LEVEL		NRF_LOG_DEFAULT_WARNING
/*-----------------------------------------------------------*/

#define	portTICKS_PER_MSEC			( configTICK_RATE_HZ / 1024 )
#define	portTICKS_PER_10MSEC		( configTICK_RATE_HZ / 102 )
#define	portTICKS_PER_100MSEC		( configTICK_RATE_HZ / 10 )
#define	portTICKS_PER_SEC			( configTICK_RATE_HZ )
#define	portTICKS_PER_MIN			( 60 * portTICKS_PER_SEC )

#define portRAW_RTC_TICKS_PER_MS	( 32768 / 1000 )

/* Task stack sizes. The idle task and timer stack sizes are defined in FreeRTOSConfig.h */
#define bleSTACK_SIZE				( ( unsigned short ) 1024 )
#define powerSensorSTACK_SIZE		( ( unsigned short ) 1024 )
#define bleAdHandlerSTACK_SIZE		( ( unsigned short ) 1024 )

/* Task priority definitions. */
#define SD_TASK_PRIORITY			( tskIDLE_PRIORITY + 4 )
#define POWER_SENSOR_TASK_PRIORITY	( tskIDLE_PRIORITY + 3 )
#define BLE_ADHANDLER_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define LEDTGL_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
/* Timer priority is + 4 ! Defined in FreeRTOSConfig.h. */

/* Highest application-related HW-interrupt priority. All higher interrupts (lower number)
   belong to the Softdevice. */
#define	HIGHEST_APP_IRQ_PRIORITY	( 3 )

enum xMEM_TYPE
{
	ROM,				/* Flash memory. */
	RAM,				/* RAM */
	EEPROM				/* EEPROM */
};

/* Flags for some data copy routines in different modules to translate apostrophes to quotation marks. */
#define TRANSLATE_APOSTR		true
#define NO_TRANSLATE_CHR		false

/* Helper macros to print defines during compile time. */
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)
/*-----------------------------------------------------------*/

/* Global variables. */
/* Watchdog channel ID. Needs to be used when feeding the watchdog. */
extern nrf_drv_wdt_channel_id 		xWdtChannelId;


#endif