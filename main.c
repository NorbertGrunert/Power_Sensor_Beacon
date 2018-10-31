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
#include "sdk_errors.h"
#include "app_error.h"
#include "nrf_drv_rtc.h"
#include "nrf_serial.h"
#include "nrf_drv_rng.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


/* Device specific include files. */
#include "main.h"
#include "drv_uart.h"
#include "rtc_blink.h"
#include "ble_main.h"
#include "ble_parser.h"

#include "nrf_temp.h"		// DEBUG DEBUG DEBUG
#include "nrf_delay.h"

/*-----------------------------------------------------------*/

int main(void)
{
    ret_code_t xErrCode;
	
    xErrCode = NRF_LOG_INIT( NULL );
    APP_ERROR_CHECK( xErrCode );

    NRF_LOG_DEFAULT_BACKENDS_INIT();
	
    /* Initialize clock driver for better time accuracy in FREERTOS */
    xErrCode = nrf_drv_clock_init();
    APP_ERROR_CHECK(xErrCode);

	/* Initialise serial interface. */
    vUartPortInit();
	
    /* Configure LED-pins as outputs */
    bsp_board_init( BSP_INIT_LEDS );
	
	/* Initilise the LED task. */
	vLedTaskInit( LEDTGL_TASK_PRIORITY );
	
	/* Initialise the BLE task. */
	vBleInit();
	
	/* Initialise the BLE parser task. */
	vBleParserInit( BLE_PARSER_TASK_PRIORITY );

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