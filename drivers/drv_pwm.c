/*
 * Tracker Firmware
 *
 * PWM Driver
 *
 * The PWM driver API uses ucDutyCycle as basis. The value range is 0 <= ucDutyCycle <= 100.
 * From a HW perspective, ucDutyCycle=0 corresponds to the output continuously driven low
 * and ucDutyCycle=100 to the output continuously driven high.
 *
 * When being used to control the charger, ucDutyCycle=0 means full charge while ucDutyCycle=100
 * means no charge current.
 *
 */
 
/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

#include "tracker.h"

/* nRF SDK files. */
#undef  NRF_LOG_DEFAULT_LEVEL
#define NRF_LOG_DEFAULT_LEVEL				NRF_LOG_DEFAULT_DEBUG
#define NRF_LOG_MODULE_NAME 				DRV_PWM
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_drv_pwm.h"

/* Scheduler include files. */
#include "FreeRTOS.h"

/* Device specific include files. */
#include "custom_board.h"
#include "drv_pwm.h"
#include "config.h"
/*-----------------------------------------------------------*/

static nrf_drv_pwm_t 				xPwmInstance0 = NRF_DRV_PWM_INSTANCE( 0 );

/* Declare variables holding PWM sequence values. Here only one channel is used.  */
nrf_pwm_values_individual_t 		xSeqValues[] = { 0, 0, 0, 0 };
nrf_pwm_sequence_t const 			xPwmSeq =
									{
										.values.p_individual 	= xSeqValues,
										.length          		= NRF_PWM_VALUES_LENGTH( xSeqValues ),
										.repeats         		= 0,
										.end_delay       		= 0
									};
									
bool								bPwmIsRunning;
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Configure the DACs. */
void vPWMInit( void );

/* Write a PWM channel. */
void vWritePwmDutyCycle( unsigned char ucDutyCycle );

/* Read a PWM channel. */
unsigned char ucReadPwmDutyCycle( void );
/*-----------------------------------------------------------*/

/* Configure the PWM.
*/
void vPWMInit( void )
{
	bPwmIsRunning = false;
}
/*-----------------------------------------------------------*/

/* Set duty cycle between 0 and 100%. 
   0% means constant high level, 100% means constant low level. 
*/
void vWritePwmDutyCycle( unsigned char ucDutyCycle )
{
	ret_code_t					xErrCode;
    nrfx_pwm_config_t const 	xPwmConfig0 =
								{
									.output_pins =
									{
										BATT_CHRG, 								/* PWM channel 0. */
										NRF_DRV_PWM_PIN_NOT_USED,             	/* PWM channel 1. */
										NRF_DRV_PWM_PIN_NOT_USED,             	/* PWM channel 2. */
										NRF_DRV_PWM_PIN_NOT_USED,             	/* PWM channel 3. */
									},
									.irq_priority = APP_IRQ_PRIORITY_LOWEST,
									.base_clock   = NRF_PWM_CLK_4MHz,
									.count_mode   = NRF_PWM_MODE_UP,
									.top_value    = 100,
									.load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
									.step_mode    = NRF_PWM_STEP_AUTO
								};
	
    /* Init PWM hardware. */
	if ( !bPwmIsRunning )
	{
		xErrCode = nrfx_pwm_init( &xPwmInstance0, &xPwmConfig0, NULL );
		APP_ERROR_CHECK( xErrCode );
	}
	
    /* Check if value is outside of range. If so, set to 100%. */
    if ( ucDutyCycle >= 100 )
    {
        xSeqValues->channel_0 = 0;
    }
    else
    {
        xSeqValues->channel_0 = 100 - ucDutyCycle;
    }
    
    nrfx_pwm_simple_playback( &xPwmInstance0, &xPwmSeq, 1, NRF_DRV_PWM_FLAG_LOOP );
	
	bPwmIsRunning = true;
}
/*-----------------------------------------------------------*/

/* Read a processor PWM channel. */
unsigned char ucReadPwmDutyCycle( void )
{
	return 100 - xSeqValues->channel_0;
}
/*-----------------------------------------------------------*/

/* Stop the PWM. */
void vStopPwm( void )
{
	if ( bPwmIsRunning )
	{
		( void )nrfx_pwm_stop( &xPwmInstance0, false );

		/* Uninitialise the PWM HW to make sure it does neither request the HFCLK nor consume power. */
		nrfx_pwm_uninit( &xPwmInstance0 );
	}
	
	bPwmIsRunning = false;
}
/*-----------------------------------------------------------*/

/* Return the stopped status of the  PWM. */
bool bPwmIsStopped( void )
{
 	return !bPwmIsRunning;
}