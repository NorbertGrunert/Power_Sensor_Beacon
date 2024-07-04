/*
 * RTC example
 */

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


/**
 * @brief RTC instance number used for blinking
 *
 */
#define BLINK_RTC 2

/**
 * @brief RTC compare channel used
 *
 */
#define BLINK_RTC_CC 0

/* Number of RTC ticks between interrupts */
#define BLINK_RTC_TICKS   ( RTC_US_TO_TICKS( 500000ULL, RTC_DEFAULT_CONFIG_FREQUENCY ) )

/* Initialise the LED task. */
void vLedTaskInit( UBaseType_t uxPriority );

