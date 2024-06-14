/*
 * Tracker Bootloader
 *
 * LED Driver
 *
 */

/* Standard include files. */
#include <stdlib.h>

#include "tracker.h"

/* nRF SDK include files */
#define NRF_LOG_MODULE_NAME 		DRV_ADC
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_saadc.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* Device specific include files. */
#include "custom_board.h"
#include "drv_adc.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
void vADCInit( void );
void vADCChannelInit( enum xADC_CHANNEL xAdcChannelIdx );
void vADCChannelUnInit( enum xADC_CHANNEL xAdcChannelIdx );
uint32_t uiReadADC( enum xADC_CHANNEL xChannel, bool bProtectedAccess );
/*-----------------------------------------------------------*/

/* Global variables. */
/* Mutex handle to protect ADC access. */
SemaphoreHandle_t		xMutexADC = NULL;
/*-----------------------------------------------------------*/

/* Module scope variables. */
nrf_saadc_value_t		xADCResult;

/* Table of ADC configuration values per channel. The channel index is defined in xADC_CHANNEL. */
const struct
{
	nrf_saadc_gain_t      		gain;       /* Gain control value. */
	nrf_saadc_reference_t 		reference;  /* Reference control value. */
	nrf_saadc_input_t     		pin_p;      /* Input positive pin selection. */
} 
xAdcChannelConfig[ ADC_CHANNEL_LAST_IDX + 1 ] =
{
	{
		/* Channel 0: Battery NTC. */
		.gain       = ADC_GAIN_NTC_FACTOR,
		.reference  = NRF_SAADC_REFERENCE_VDD4,
		.pin_p      = ( nrf_saadc_input_t )( NTC_ADC_PIN )
	},
	{
		/* Channel 1: CHRG_STATE_ADC : Charge state pin for the LiIon charge controller. */
		.gain       = ADC_GAIN_CHRG_STATE_FACTOR,
		.reference  = NRF_SAADC_REFERENCE_INTERNAL,
		.pin_p      = ( nrf_saadc_input_t )( VCHRG_STATE_ADC_PIN )
	},
	{
		/* Channel 2: VGSM_ADC : GSM module supply voltage. */
		.gain       = ADC_GAIN_VGSM_FACTOR,
		.reference  = NRF_SAADC_REFERENCE_INTERNAL,
		.pin_p      = ( nrf_saadc_input_t )( VGSM_ADC_PIN )
	},
	{
		/* Channel 3: VBAT_ADC : Battery voltage. */
		.gain       = ADC_GAIN_VBAT_FACTOR,
		.reference  = NRF_SAADC_REFERENCE_INTERNAL,
		.pin_p      = ( nrf_saadc_input_t )( VBAT_ADC_PIN )
	},
	{
		/* Channel 4: VCHRG_ADC : Wireless charger voltage. */
		.gain       = ADC_GAIN_VCHRG_FACTOR,
		.reference  = NRF_SAADC_REFERENCE_INTERNAL,
		.pin_p      = ( nrf_saadc_input_t )( VCHRG_ADC_PIN )
	},
	{
		/* Channel 5: VCC_ADC : VCC */
		.gain       = ADC_GAIN_VCC_FACTOR,
		.reference  = NRF_SAADC_REFERENCE_INTERNAL,
		.pin_p      = ( nrf_saadc_input_t )( VCC_ADC_PIN )
	}
};
/*-----------------------------------------------------------*/ 

/* SAADC event handler. Unsued function as the nRF SAADC driver is used in blocking mode, but required for the syntax. */
void vSaadcCallbackHandler(nrf_drv_saadc_evt_t const * pxEvent)
{
	( void )pxEvent;
}
/*-----------------------------------------------------------*/ 

/* Initialise ADC peripheral. 
   All channels used are initialised here.
*/
void vADCInit( void )
{
	ret_code_t					xErrCode;
	unsigned portBASE_TYPE		uxChIdx;
	nrf_saadc_channel_config_t	xChannelConfig = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE_40US( NTC_ADC_PIN );

	/* Create a mutex to protect access to the ADC so that it can be used from different tasks. */
	xMutexADC = xSemaphoreCreateMutex();	

	/* Initialize the saadc. */
	xErrCode = nrf_drv_saadc_init( NULL, vSaadcCallbackHandler );
	APP_ERROR_CHECK( xErrCode );

	/* For all ADC channels, create a config structure and assign it default values. Then initialize the channel which will 
	   be connected to that specific pin. */
	for ( uxChIdx = 0; uxChIdx <= ADC_CHANNEL_LAST_IDX; uxChIdx++ )
	{
		vADCChannelInit( uxChIdx );
	}

	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/    

/* Initialise a single ADC channel. Idx index given as parameter points to a ROM table of
   initialisation values.
*/
void vADCChannelInit( enum xADC_CHANNEL xAdcChannel )
{
	ret_code_t					xErrCode;
	nrf_saadc_channel_config_t	xChannelConfig = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE_40US( NTC_ADC_PIN );

	xChannelConfig.pin_p 	 = xAdcChannelConfig[ xAdcChannel ].pin_p;
	xChannelConfig.gain 	 = xAdcChannelConfig[ xAdcChannel ].gain;
	xChannelConfig.reference = xAdcChannelConfig[ xAdcChannel ].reference;
	xErrCode = nrfx_saadc_channel_init( xAdcChannel, &xChannelConfig );
	APP_ERROR_CHECK( xErrCode );
}
/*-----------------------------------------------------------*/    

/* Uninitialise a single ADC channel. Idx index given as parameter points to a ROM table of
   initialisation values.
*/
void vADCChannelUnInit( enum xADC_CHANNEL xAdcChannel )
{
	nrfx_saadc_channel_uninit( xAdcChannel );
}

/*-----------------------------------------------------------*/    

/* Read the ADC for the given xAdcChannel. The read process is blocking.
   Only one sample is used. If required, several samples can be read and averaged using HW. 
*/
uint32_t uiReadADC( enum xADC_CHANNEL xAdcChannel, bool bProtectedAccess )
{
	int32_t					iADCResultAvg;
	unsigned portBASE_TYPE	uxAdcCnt;

	if ( bProtectedAccess )
	{
		configASSERT( xSemaphoreTake( xMutexADC, TO_MUTEX_ADC ) );
	}

	/* A blocking function which will be called and the processor waits until the value is read.
	   The sample value read is in 2's complement and is automatically converted once retrieved.
	   The first parameter is the adc input channel. The second parameter is to pass the address 
	   of the variable in which the ADC sample value will be stored. */
	iADCResultAvg = 0;
	for ( uxAdcCnt = 0; uxAdcCnt < ADC_AVG_CNT; uxAdcCnt++ )
	{
		nrfx_saadc_sample_convert( xAdcChannel, &xADCResult );
		
		/* The ADC is used in single ended mode and thus returns positive samples. It may occur, though, 
		   that negative samples are returned from the nRF SDK driver. Thus clamp the value to 0. */
		if ( xADCResult > 0 )
		{
			iADCResultAvg += xADCResult;
		}		
	}

	iADCResultAvg /= ADC_AVG_CNT;
	
	if ( bProtectedAccess )
	{
		xSemaphoreGive( xMutexADC );
	}
	
	return ( uint32_t )iADCResultAvg;	
}
/*-----------------------------------------------------------*/         
