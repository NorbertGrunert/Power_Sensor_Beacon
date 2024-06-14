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

#define BAD_FW_RESET_CNT			( 30 )

#define ERROR_STACK_DUMP_LEN		( 20 )			/* Length of stack dumped to persistent RAM in reset case. */


/* Public function prototypes. */
/* Set the bad boot counter in the system error record to 0. */
extern void vResetBadBootCounter( void );

/* Application error handler. */
extern void app_error_fault_handler( uint32_t uiId, uint32_t uiPc, uint32_t uiInfo );
/*-----------------------------------------------------------*/

/* Global variables. */
/* Watchdog channel ID. Needs to be used when feeding the watchdog. */
extern nrf_drv_wdt_channel_id 		xWdtChannelId;

/* Persistent memory section containing variables pertaining to the watchdog and software resets. */
extern long __attribute__( (section ( ".non_init" ) ) ) 				ulErrorId;
extern long __attribute__( (section ( ".non_init" ) ) ) 				ulErrorPc;
extern struct xERROR_INFO __attribute__( ( section( ".non_init" ) ) )	xErrorInfo;
extern uint32_t __attribute__( ( section( ".non_init" ) ) ) 			uiErrorStack[ ERROR_STACK_DUMP_LEN ];

#endif