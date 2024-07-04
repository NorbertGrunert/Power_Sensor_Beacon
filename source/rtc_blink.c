#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"
#include "nrf_drv_rtc.h"
#include "rtc_blink.h"
#include "drv_uart.h"
#include "main.h"


/*
 * Reference to LED0 toggling FreeRTOS task.
 */
static TaskHandle_t  m_led_toggle_task_handle;

/*
 * Semaphore set in RTC event
 */
static SemaphoreHandle_t m_led_semaphore;

/*
 * RTC configuration
 */
static nrf_drv_rtc_config_t const m_rtc_config = NRF_DRV_RTC_DEFAULT_CONFIG;

/*
 * Instance of the RTC used for led blinking
 */
static nrf_drv_rtc_t const m_rtc = NRF_DRV_RTC_INSTANCE(BLINK_RTC);
/*-----------------------------------------------------------*/


/*
 * Function Prototypes. 
 */
static void blink_rtc_handler(nrf_drv_rtc_int_type_t int_type);
static void vLedToggleTask(void * pvParameter );
/*-----------------------------------------------------------*/


/*
 * LED task init
 */
void vLedTaskInit( UBaseType_t uxPriority )
{
    /* Create task for LED0 blinking with priority set to 2 */
    ( void )xTaskCreate( vLedToggleTask, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, uxPriority, &m_led_toggle_task_handle );
}
/*-----------------------------------------------------------*/


/*
 * RTC interrupt handler.
 */
static void blink_rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    BaseType_t yield_req = pdFALSE;
    ret_code_t err_code;
	
//    bsp_board_led_invert( BSP_BOARD_LED_0 );
	
    err_code = nrf_drv_rtc_cc_set(
        &m_rtc,
        BLINK_RTC_CC,
        ( nrf_rtc_cc_get( m_rtc.p_reg, BLINK_RTC_CC ) + BLINK_RTC_TICKS ) & RTC_COUNTER_COUNTER_Msk,
        true );
    APP_ERROR_CHECK( err_code );

   /* The returned value may be safely ignored, if error is returned it only means that
    * the semaphore is already given (raised). */
   ( void )xSemaphoreGiveFromISR( m_led_semaphore, &yield_req );
   
   portYIELD_FROM_ISR( yield_req );
}
/*-----------------------------------------------------------*/


/* 
 * LED0 task function.
 */
static void vLedToggleTask( void * pvParameter )
{
    ret_code_t err_code;

    ( void )pvParameter;
	
	/* Initialise the RTC. */
    err_code = nrf_drv_rtc_init( &m_rtc, &m_rtc_config, blink_rtc_handler );
    APP_ERROR_CHECK( err_code );
	
    err_code = nrf_drv_rtc_cc_set( &m_rtc, BLINK_RTC_CC, BLINK_RTC_TICKS, true );
    APP_ERROR_CHECK( err_code );
	
    nrf_drv_rtc_enable( &m_rtc );

	/* Create the semaphore to communicate from the RTC IRQ back to the LED task. */
    m_led_semaphore = xSemaphoreCreateBinary();
    ASSERT( NULL != m_led_semaphore );

	/* Task main loop. */
    while (true)
    {
//		/* Toggle LED. */
//        bsp_board_led_invert( BSP_BOARD_LED_0 );

        /* Wait for the event from the RTC */
        ( void )xSemaphoreTake (m_led_semaphore, portMAX_DELAY );
    }

    /* Tasks must be implemented to never return... */
}
/*-----------------------------------------------------------*/
