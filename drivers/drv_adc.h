/*
 * Tracker Bootloader
 *
 * ADC driver header file
 *
 */
 
#ifndef DRV_ADC_H
#define DRV_ADC_H

#include "FreeRTOS.h"

#include "tracker.h"
#include "custom_board.h"
/*-----------------------------------------------------------*/

/* Time-out (s) for gaining access to the ADC. */
#define TO_MUTEX_ADC					( 10 * portTICKS_PER_SEC ) 		
#define ADC_AVG_CNT						( 4 )						/* Number of times each channel is sampled for averaging. */

#define ADC_MAX_VALUE					( 4095 )					/* 12bit resolution: 2^12 - 1 */
#define ADC_VREF						( 0.6 )						/* Vref internal is 0.6V */

#define ADC_GAIN_NTC					( 4 )						/* NTC max. value: 1.8V */
#define ADC_GAIN_NTC_FACTOR				NRF_SAADC_GAIN1_4
#define ADC_GAIN_CHRG_STATE				( 4 )						/* Voltage CHRG_STATE max. value: 1.8V */
#define ADC_GAIN_CHRG_STATE_FACTOR		NRF_SAADC_GAIN1_4
#define ADC_GAIN_VGSM					( 1 * 7.05 )				/* VGSM max. value: 4.2V * 30.1k / (182k + 30.1k) */
#define ADC_GAIN_VGSM_FACTOR			NRF_SAADC_GAIN1
#define ADC_GAIN_VBAT					( 2 * 5 )					/* VBAT max. value: 4.2V / 2 / 5 */
#define ADC_GAIN_VBAT_FACTOR			NRF_SAADC_GAIN1_2
#define ADC_GAIN_VCHRG				    ( 1 * 8.5 )					/* VCHRG max. value: 5V * 10k / (75k + 10k) */
#define ADC_GAIN_VCHRG_FACTOR		    NRF_SAADC_GAIN1
#define ADC_GAIN_VCC					( 4 )						/* VCC max. value: 1.8V */
#define ADC_GAIN_VCC_FACTOR				NRF_SAADC_GAIN1_4			
/*-----------------------------------------------------------*/

/* Macros. */
/* Macro for setting @ref nrf_saadc_channel_config_t to default settings in single-ended mode. */
#define NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE_40US( PIN_P )	\
{                                                   		\
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      		\
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      		\
    .gain       = NRF_SAADC_GAIN1_6,                		\
    .reference  = NRF_SAADC_REFERENCE_INTERNAL,     		\
    .acq_time   = NRF_SAADC_ACQTIME_40US,           		\
    .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      		\
    .burst      = NRF_SAADC_BURST_DISABLED,         		\
    .pin_p      = ( nrf_saadc_input_t )( PIN_P ),       	\
    .pin_n      = NRF_SAADC_INPUT_DISABLED          		\
}
/*-----------------------------------------------------------*/

/* Function prototypes. */
extern void vADCInit( void );
extern void vADCChannelInit( enum xADC_CHANNEL xAdcChannelIdx );
extern void vADCChannelUnInit( enum xADC_CHANNEL xAdcChannelIdx );
extern uint32_t uiReadADC( enum xADC_CHANNEL xChannel, bool bProtectedAccess );
/*-----------------------------------------------------------*/

#endif