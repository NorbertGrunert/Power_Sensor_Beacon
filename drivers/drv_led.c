/*
 * Tracker Firmware
 *
 * LED Driver
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

#include "tracker.h"

/* nRF SDK include files */
#define NRF_LOG_MODULE_NAME 		DRV_LED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_sdh.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* Device specific include files. */
#include "custom_board.h"
#include "drv_uart.h"
#include "parser.h"
#include "gsm.h"
#include "ctrl.h"
#include "drv_led.h"
#include "charger.h"
#include "motiondet.h"
#include "drv_nvm.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Switch on the red LED. */
void vSwitchOnLedRed( unsigned short usRequester );

/* Switch off the red LED. */
void vSwitchOffLedRed( unsigned short usRequester );

/* Switch on the green LED. */
void vSwitchOnLedGreen( unsigned short usRequester );

/* Switch off the green LED. */
void vSwitchOffLedGreen( unsigned short usRequester );

/* Switch on the blue LED. */
void vSwitchOnLedBlue( unsigned short usRequester );

/* Switch off the blue LED. */
void vSwitchOffLedBlue( unsigned short usRequester );

/* Let the LED start blinking. */
void vStartLedBlinking( void );

/* Let the LED stop blinking. */
void vStopLedBlinking( void );

/* Set LED blink frequency / ratio in function of module state. */
void prvSetLedFreq( TickType_t *xLedOnTime, TickType_t *xLedOffTime, portBASE_TYPE *xLedFlashMaxCount );

/* LED callback. */
void prvLedCallback( TimerHandle_t xTimer );

/* LED driver init. */
void vLedInit( void );
/*-----------------------------------------------------------*/

/* Global variables. */
/*-----------------------------------------------------------*/

/* Module scope variables. */
/* Timer handle for the LED task. */
static TimerHandle_t 	xLedTimer;	

/* LED Flash counter. */
portBASE_TYPE			xLedFlashCounter;

/* LED state. */
bool					bLedOn;

/* LED requests. This variable allows several requesters in different tasks to control the blue LED. The LED status is 
   then a OR-combination of the individual requests. */
unsigned short			usLedReq;

/* Table containing the LED blink times. The table needs to be aligned with the control state enum! */
const struct 
{
	TickType_t			xLedOn;
	TickType_t			xLedOff;
	portBASE_TYPE		xLedFlashCount;
} 
xLedOnOffTimes[] = 
{	
	{ ledON_STBY, 		ledOFF_STBY, 		ledFC_STBY		},		/* CTRL_STANDBY			*/
	{ ledON_CHRG_OOO,	ledOFF_CHRG_OOO,	ledFC_CHRG_OOO	},		/* CTRL_CHRG_OOO  		*/
	{ ledON_OOO, 		ledOFF_OOO, 		ledFC_OOO		},		/* CTRL_OOO				*/
	{ ledON_CHRG_NRML,	ledOFF_CHRG_NRML,	ledFC_CHRG_NRML },		/* CTRL_CHRG_NRML 		*/
	{ ledON_ACTIVE, 	ledOFF_ACTIVE, 		ledFC_ACTIVE	},		/* CTRL_ACTIVE			*/
	{ ledON_INACTIVE, 	ledOFF_INACTIVE,	ledFC_INACTIVE	},		/* CTRL_INACTIVE  		*/
	{ ledON_STILL, 		ledOFF_STILL, 		ledFC_STILL		},		/* CTRL_STILL	   		*/
	{ ledON_PREALERT, 	ledOFF_PREALERT,	ledFC_PREALERT 	},		/* CTRL_PREALERT		*/
	{ ledON_ALERT, 		ledOFF_ALERT, 		ledFC_ALERT		},		/* CTRL_ALERT			*/	
	{ ledON_PRESOS, 	ledOFF_PRESOS,		ledFC_PRESOS 	},		/* CTRL_PRESOS			*/
	{ ledON_SOS, 		ledOFF_SOS, 		ledFC_SOS		},		/* CTRL_SOS				*/	
	{ ledON_SLEEP, 		ledOFF_SLEEP, 		ledFC_SLEEP		}		/* CTRL_SLEEP	   		*/
};                                                                      
/*-----------------------------------------------------------*/         

/* LED driver init. */
void vLedInit( void )
{
	/* Create LED timer. */
	xLedTimer = xTimerCreate
							( "LED",						/* Timer name for debug. */
							  ledOFF_INACTIVE,				/* Initial timer period in ticks (dummy here). */
							  pdTRUE,						/* Auto-reload. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  prvLedCallback				/* Callback for the LED timer. */
							);
	xLedFlashCounter = 0;
	
	/* Set outputs for LEDs. LEDs are off. */
	nrf_gpio_cfg_output( LED_RED );
	nrf_gpio_cfg_output( LED_GREEN );
	nrf_gpio_cfg_output( LED_BLUE );
	#if defined ( TL510 ) || defined ( TL512 )
		nrf_gpio_pin_clear( LED_RED );
		nrf_gpio_pin_clear( LED_GREEN );
		nrf_gpio_pin_clear( LED_BLUE );
	#else
		nrf_gpio_pin_set( LED_RED );
		nrf_gpio_pin_set( LED_GREEN );
		nrf_gpio_pin_set( LED_BLUE );
	#endif
	bLedOn = false;
	usLedReq = 0;

	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Switch on the red LED. */
void vSwitchOnLedRed( unsigned short usRequester )
{
	portENTER_CRITICAL();	
	usLedReq |= usRequester;
	portEXIT_CRITICAL();	
	#if defined ( TL510 ) || defined ( TL512 )
		nrf_gpio_pin_set( LED_RED );
	#else
		nrf_gpio_pin_clear( LED_RED );
	#endif
}
/*-----------------------------------------------------------*/         

/* Switch off the red LED. */
void vSwitchOffLedRed( unsigned short usRequester )
{
	portENTER_CRITICAL();	
	usLedReq &= ( unsigned short )~usRequester;
	if ( usRequester & ledEXCL_LED_REQ )
	{
		#if defined ( TL510 ) || defined ( TL512 )
			nrf_gpio_pin_clear( LED_RED );
		#else
			nrf_gpio_pin_set( LED_RED );
		#endif
	}
	else
	{
		if ( ( usLedReq & ( ledRED_LED_REQ | ledRED_GSM_REQ | ledRED_BLE_REQ) ) == 0 )
		{
			#if defined ( TL510 ) || defined ( TL512 )
				nrf_gpio_pin_clear( LED_RED );
			#else
				nrf_gpio_pin_set( LED_RED );
			#endif
		}
	}
	portEXIT_CRITICAL();	
}
/*-----------------------------------------------------------*/         

/* Switch on the green LED. */
void vSwitchOnLedGreen( unsigned short usRequester )
{
	portENTER_CRITICAL();	
	usLedReq |= usRequester;
	portEXIT_CRITICAL();	
	#if defined ( TL510 ) || defined ( TL512 )
		nrf_gpio_pin_set( LED_GREEN );
	#else
		nrf_gpio_pin_clear( LED_GREEN );
	#endif
}
/*-----------------------------------------------------------*/         

/* Switch off the green LED. */
void vSwitchOffLedGreen( unsigned short usRequester )
{
	portENTER_CRITICAL();	
	if ( usRequester & ledEXCL_LED_REQ )
	{
		#if defined ( TL510 ) || defined ( TL512 )
			nrf_gpio_pin_clear( LED_GREEN );
		#else
			nrf_gpio_pin_set( LED_GREEN );
		#endif
	}
	else
	{
		usLedReq &= ( unsigned short )~usRequester;
		if ( ( usLedReq & ( ledGREEN_LED_REQ | ledGREEN_GSM_REQ | ledGREEN_BLE_REQ ) ) == 0 )
		{
			#if defined ( TL510 ) || defined ( TL512 )
				nrf_gpio_pin_clear( LED_GREEN );
			#else
				nrf_gpio_pin_set( LED_GREEN );
			#endif
		}
	}
	portEXIT_CRITICAL();	
}
/*-----------------------------------------------------------*/         

/* Switch on the blue LED. */
void vSwitchOnLedBlue( unsigned short usRequester )
{
	portENTER_CRITICAL();	
	usLedReq |= usRequester;
	portEXIT_CRITICAL();	
	#if defined ( TL510 ) || defined ( TL512 )
		nrf_gpio_pin_set( LED_BLUE );
	#else
		nrf_gpio_pin_clear( LED_BLUE );
	#endif
}
/*-----------------------------------------------------------*/         

/* Switch off the blue LED. */
void vSwitchOffLedBlue( unsigned short usRequester )
{
	portENTER_CRITICAL();	
	usLedReq &= ( unsigned short )~usRequester;
	if ( usRequester & ledEXCL_LED_REQ )
	{
		#if defined ( TL510 ) || defined ( TL512 )
			nrf_gpio_pin_clear( LED_BLUE );
		#else
			nrf_gpio_pin_set( LED_BLUE );
		#endif
	}
	else
	{
		if ( ( usLedReq & ( ledBLUE_LED_REQ | ledBLUE_GSM_REQ | ledBLUE_BLE_REQ ) ) == 0 )
		{
			#if defined ( TL510 ) || defined ( TL512 )
				nrf_gpio_pin_clear( LED_BLUE );
			#else
				nrf_gpio_pin_set( LED_BLUE );
			#endif
		}
	}
	portEXIT_CRITICAL();	
}
/*-----------------------------------------------------------*/         

/* Let the LED start blinking. */
void vStartLedBlinking( void )
{
	/* Start the LED timer. */
	( void )xTimerStart( xLedTimer, portMAX_DELAY );
}
/*-----------------------------------------------------------*/

/* Let the LED stop blinking. */
void vStopLedBlinking( void )
{
	/* Switch LED off. */
	vSwitchOffLedBlue( ledBLUE_LED_REQ );
	vSwitchOffLedRed( ledRED_LED_REQ );			
	vSwitchOffLedGreen( ledGREEN_LED_REQ );	
	bLedOn = false;

	/* Stop the LED timer. */
	( void )xTimerStop( xLedTimer, portMAX_DELAY );
}
/*-----------------------------------------------------------*/

/* Set LED blink frequency / ratio in function of module state. */
void prvSetLedFreq( TickType_t *xLedOnTime, TickType_t *xLedOffTime, portBASE_TYPE *xLedFlashMaxCount )
{
	enum xCTRL_STATE xCtrlState;
	
	/* Get state of the device. */
	xCtrlState = xGetCtrlState();
	
	/* Set the LED blink behaviour accordingly. */
	if ( bBatteryEmpty() == false )
	{
		if ( !bTemperatureOutsideOp() )
		{
			*xLedOnTime        = xLedOnOffTimes[ ( unsigned portBASE_TYPE )xCtrlState ].xLedOn;
			*xLedOffTime       = xLedOnOffTimes[ ( unsigned portBASE_TYPE )xCtrlState ].xLedOff;
			*xLedFlashMaxCount = xLedOnOffTimes[ ( unsigned portBASE_TYPE )xCtrlState ].xLedFlashCount;
		}
		else
		{
			/* Device is in extreme temperature condition. All functions are shut down as far as possible. 
			   Use a special blink rate here. Depending on whether the temperature is over or under the 
			   operating temperature range, the blue (cold) or green (hot) LEDs blink together with the red one. */
			*xLedOnTime        = ledON_OUTSIDEOPTEMP;
			*xLedOffTime       = ledOFF_OUTSIDEOPTEMP;
			*xLedFlashMaxCount = ledFC_OUTSIDEOPTEMP;
		}
	}
	else
	{
		*xLedOnTime        = ledON_BATEMPTY;
		*xLedOffTime       = ledOFF_BATEMPTY;
		*xLedFlashMaxCount = ledFC_BATEMPTY;
	}
}
/*-----------------------------------------------------------*/

/* LED callback. */
void prvLedCallback( TimerHandle_t xTimer )
{
	TickType_t		xLedOnTime;			/* Current LED on time, depends on module state. */
	TickType_t		xLedOffTime;		/* Current LED off time, depends on module state. */
	portBASE_TYPE	xLedFlashMaxCount;	/* Maximum LED flash count */

	/* Calm down the compiler. */
	( void )xTimer;
	
	if ( ( !bDeviceTilted ) || ( bGetOnCharger() ) )
	{
		/* Device is in normal position. */
		/* Set LED behavior in function of device state. */
		prvSetLedFreq( &xLedOnTime, &xLedOffTime, &xLedFlashMaxCount );
		
		/* Toggle the LED state. The target LED is:
				when being in any charging state AND battery is full:	LED_BLUE
				in all other states:									LED_RED 
		   Making sure that the LED which is not selected is off while not interfering with its 
		   other function is a bit tricky. The blue LED is always switched off when leaving any charging state
		   to allow for using it with a defined state when not being in charge mode. */	
		if ( !bLedOn )
		{
			/* LED is off. */
			
			/* As precaution, switch off all LEDs. */
			vSwitchOffLedRed( ledRED_LED_REQ );
			vSwitchOffLedGreen( ledGREEN_LED_REQ );
			vSwitchOffLedBlue( ledBLUE_LED_REQ );
			
			/* Switch on LED_RED. */
			vSwitchOnLedRed( ledRED_LED_REQ );
			
			/* If the temperature is outside the operating temperature limits, also let either the blue or green
			   LED blink together with the red one. */			   
			if ( bTemperatureBelowOp() )
			{
				vSwitchOnLedBlue( ledBLUE_LED_REQ );	   
			}
			if ( bTemperatureAboveOp() )
			{
				vSwitchOnLedGreen( ledGREEN_LED_REQ );	   
			}

			bLedOn = true;
			
			/* Safety net: xTimerChangePeriod crashes with new period set to 0. */
			xLedOnTime = ( xLedOnTime == 0 ) ? 1 : xLedOnTime;
	
			/* Set LED timer to ON duration. */
			( void )xTimerChangePeriod( xLedTimer, xLedOnTime, 0 );
		}
		else
		{
			/* LED is on */
			/* Increment the flash count. */
			xLedFlashCounter++;
			
			/* Switch off all LEDs. Switching 'off' the blue LED only removes the
			   request from the LED driver. If the LED is requested from another driver (BLE...),
			   it will stay on. */
			vSwitchOffLedRed( ledRED_LED_REQ );
			vSwitchOffLedGreen( ledGREEN_LED_REQ );
			vSwitchOffLedBlue( ledBLUE_LED_REQ );

			bLedOn = false;
			
			if ( xLedFlashCounter >= xLedFlashMaxCount )
			{
				/* Safety net: xTimerChangePeriod crashes with new period set to 0. */
				xLedOffTime = ( xLedOffTime == 0 ) ? 1 : xLedOffTime;
		
				/* Set LED timer to OFF duration. */
				( void )xTimerChangePeriod( xLedTimer, xLedOffTime, 0 );
				
				/* Reset the flash count. */
				xLedFlashCounter = 0;
			}
			else
			{
				/* Set LED timer to short pause between flashes. */
				( void )xTimerChangePeriod( xLedTimer, ledFLASH_PAUSE, 0 );
			}
		}
	}
	else
	{
		/* Device is in tilted position: indicate charging state on LEDs. */
		if ( !bLedOn )
		{
			/* LED is off: switch on. */
			
			/* As precaution, switch off all LEDs. */
			vSwitchOffLedRed( ledRED_LED_REQ | ledEXCL_LED_REQ );
			vSwitchOffLedGreen( ledGREEN_LED_REQ | ledEXCL_LED_REQ );
			vSwitchOffLedBlue( ledBLUE_LED_REQ | ledEXCL_LED_REQ );
			
			if ( bBatteryFull() )
			{
				/* Switch on LED_BLUE. */
				vSwitchOnLedBlue( ledBLUE_LED_REQ | ledEXCL_LED_REQ );
			}
			else
			{
				if ( bBatteryLow() )
				{
					/* Switch on LED_RED. */
					vSwitchOnLedRed( ledRED_LED_REQ | ledEXCL_LED_REQ );
				}
				else
				{
					/* Switch on LED_GREEN. */
					vSwitchOnLedGreen( ledGREEN_LED_REQ | ledEXCL_LED_REQ );
				}
			}
			bLedOn = true;
			
			/* Set LED timer to ON duration. */
			( void )xTimerChangePeriod( xLedTimer, ledON_BATDISPLAY, 0 );
		}
		else
		{
			/* LED is on: switch off. */
			
			/* As precaution, switch off all LEDs. */
			vSwitchOffLedRed( ledRED_LED_REQ | ledEXCL_LED_REQ );
			vSwitchOffLedGreen( ledGREEN_LED_REQ | ledEXCL_LED_REQ );
			vSwitchOffLedBlue( ledBLUE_LED_REQ | ledEXCL_LED_REQ );
			
			bLedOn = false;
			
			/* Set LED timer to OFF duration. */
			( void )xTimerChangePeriod( xLedTimer, ledOFF_BATDISPLAY, 0 );
		}
	}
}
/*-----------------------------------------------------------*/

