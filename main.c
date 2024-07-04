/*
 * FreeRTOS test application integrating RTC and UART.
 */

/* Standard include files. */
#include <stdbool.h>
#include <stdint.h>


/* Scheduler include files. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"


/* nRF specific include files. */
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "hardfault.h"
#include "sdk_errors.h"
#include "app_error.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_rng.h"
#include "nrf_drv_wdt.h"
#include "nrf_power.h"
#include "nrf_delay.h"

#define NRF_LOG_MODULE_NAME 		MAIN
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


/* Device specific include files. */
#include "ble_main.h"
#include "power_sensor.h"
#include "drv_uart.h"
#include "main.h"
#include "rtc.h"
#include "rtc_blink.h"

#include "nrf_temp.h"		// DEBUG DEBUG DEBUG
/*-----------------------------------------------------------*/

/* Function prototypes. */

/* Watchdog event handler. */
static void vWdtEventHandler( void );
/*-----------------------------------------------------------*/

/* Global variables. */

/* Watchdog channel ID. Needs to be used when feeding the watchdog. */ 
nrf_drv_wdt_channel_id 		xWdtChannelId;

/* Flag telling that the system is inside the fatal error handler. */
bool						bInFatalErrorHandler = false;
/*-----------------------------------------------------------*/

int main(void)
{
    ret_code_t              xErrCode;
    nrf_drv_wdt_config_t 	xWdtConfig = NRF_DRV_WDT_DEAFULT_CONFIG;
	
	/* Enable DC/DC converter. */
    nrf_power_dcdcen_set( true );

    xErrCode = NRF_LOG_INIT( NULL );
    APP_ERROR_CHECK( xErrCode );

    NRF_LOG_DEFAULT_BACKENDS_INIT();
	
    /* Initialize clock driver for better time accuracy in FREERTOS */
    xErrCode = nrf_drv_clock_init();
    APP_ERROR_CHECK(xErrCode);

	/* Add a 2s delay before enabling the watchdog. This makes sure that the battery's PCM gets enabled before consuming more than the trickle charge
	   current. Else the device would boot on the charge current only w/o battery support and crash as soon as significantly more than the charge 
	   current is required, e.g. during cellular module start-up. */
	nrf_delay_ms( 2000 );
	
    /* Configure watchdog. 8 watchdog channels are available. For the time being only
	   one is used and fed in the RTOS scheduler.
	   TODO: Use one watchdog channel per task so that the watchdog barks even if only
	   one task crashes. */
    xErrCode = nrf_drv_wdt_init( &xWdtConfig, vWdtEventHandler );
    APP_ERROR_CHECK( xErrCode );    
	xErrCode = nrf_drv_wdt_channel_alloc( &xWdtChannelId );
    APP_ERROR_CHECK( xErrCode );
    nrf_drv_wdt_enable();

	/* Initialise the real-time clock. */
	vRTCInit();    

	/* Initialise serial interface. */
    vUartPortInit();
	
    /* Configure LED-pins as outputs */
    bsp_board_init( BSP_INIT_LEDS );
	
	/* Initilise the LED task. */
	vLedTaskInit( LEDTGL_TASK_PRIORITY );
	
	/* Initialise the BLE task. */
	vBleInit();
	
	/* Initialise the power sensor task. */
	vPowerSensorInit( POWER_SENSOR_TASK_PRIORITY );
	
    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	
	NRF_LOG_INFO( "NINA emulator started." );	

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    while (true)
    {
        ASSERT( false );
        /* FreeRTOS should not be here... FreeRTOS goes back to the start of stack
         * in vTaskStartScheduler function. */
    }
}
/*-----------------------------------------------------------*/

/* Watchdog event handler.

   This handler is called in case of a watchdog time-out.
*/
static void vWdtEventHandler( void )
{
	/* Force a system reset. */
	configASSERT( false );
}
/*-----------------------------------------------------------*/

/* Function called in case of stack overflows. As this is critical,
   store all relevant information in the EEPROM and execute a hard reset of 
   the entire system.
*/
void vApplicationStackOverflowHook( void * xTask, char *pcTaskName )
{
	( void )xTask;
	( void )pcTaskName;
	
	/* Raise system error. */
	configASSERT( false );
}
/*-----------------------------------------------------------*/

/* Function called in case of heap allocation failure. As this is critical,
   execute a hard reset of the entire system.
*/
void vApplicationMallocFailedHook( void )
{
	/* Raise system error. */
	configASSERT( false );
}
/*-----------------------------------------------------------*/