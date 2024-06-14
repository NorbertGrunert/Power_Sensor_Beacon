/*
   Tracker Firmware
  
   Battery Charger
 
   The battery charger has several tasks:
		- Monitoring and categorising the battery voltage into different charge state (empty, low, full, 98pct and normal).
		- Monitoring and categorising the battery temperature (outside range for charging, outside range for operation).
		- Detecting if the device is on the charger.
		- Limiting the charging voltage to BATT_VOLT_MAX_EXT_CHRG_THRES in extended temperature operation range
		  (JEITA extended temperature charging).
   
   The heart of the module is vChargerHandler() which is regularly called from the timer. All detections happen in there.
   
   Battery voltage measurement:
		Reads the battery's voltage on VDDH and filters the result using a 1st order IIR. To improve stability of the results,
		the filter constant is different for rising and for falling battery voltages.
		
   Battery temperature measurement:
		Reads the battery's NTC value via the NTC_ADC and converts the result to a temperature, ref. to sGetBatteryTemperature().
		Several temperature zones are defined:
					< -20C			No device operation
					-20C ... 0C		Device may operate but charging is prohibited.
					0C ... 45C		Unrestricted operation and charging
					45C ... 60C		Extended temerature range: device may operate but charge only at a certain voltage (<4.0V)
					> 60C			Extreme temperature range: The device is not allowed to operate.
   
   Charge detection:
		TL500/510:
			Compares the voltage on VCHRG_SENSE against V_CHARGE_MIN using VCHRG_ADC.
		TL501, TL502, TL512 and later:
			Compares VCHRG_SENSE with the LPCOMP against a threshold and uses an interrupt on the signal going high to detect 
			the charger. Once the charger has been detected, normal operation continues.
		
   Charge current regulation
		The charge current is regulated by injecting a current into the resistor which is used by the HW charging device to 
		control the charge current.   
   
		The charge current is determined by a PWM output (BATT_CHRG): 

		TL500/510:
							 ___				 ___
		  PWM OUT ----------|___|-------+-------|___|----+---------  <--- Ichrg / 1000 (STC4054 for TL500, MCP73832-2AC for TL510)
		 (push-pull)		R19/R8	    |       R20/R7   |
							 2.4k	    |        5k6     -
									   ---              | | R21/R10
									   ---  220n        | | 10k
										|                -
										|				 |
										-				 -

		TL501 and TL502:
							 ___				 ___
		  PWM OUT ----------|___|-------+-------|___|----+---------  <--- Ichrg / 1000 (STC4054)
		(push high only)	 R19	    |        R20     |
							 931	    |        931     -
									   ---              | | R21
									   ---  220n        | | 2k55
										|                -
										|				 |
										-				 -
								
	    PWM duty cycle to charge current translation:
	    
	    	duty cycle			TL500/510		TL501/502
	    		  0				225mA			-- n/a --
	    		 10%			201mA			-- n/a --
	    		 20%			181mA			-- n/a --
	    		 30%			155mA			  218mA
	    		 40%			134mA			  174mA
	    		 50%			111mA			  136mA
	    		 60%			 90mA			  101mA
	    		 70%			 68mA			   69mA
	    		 80%			 45mA			   41mA
	    		 90%			 24mA			   15mA
	    		100%			  0mA		        0mA (calculated: -8.6mA)
	    
	    The relation between Ichrg and PWM duty cycle can be approximated as follows:

		TL500/510:	    			I = 225mA * ( 1 - PWM )		   0 <= PWM <= 1
		TL501/502:		   			I = 310mA - 321mA * PWM		 0.3 <= PWM <= 1
		
		Note that the duty cycle PWM = 1 - ucChargeSetting/100 where ucChargeSetting is the percentage of the maximum charge current.


		TL512:
							 ___				 ___
		  PWM OUT ----------|___|-------+-------|___|----+---------  <--- Ichrg / 1000 (MCP73832-2DC)
		(push high only		 R8 	    |        R7      |
		 or pull low		 3k3	    |        3k3     -
		 only)						   ---              | | R10
									   ---  220n        | | 18k
										|                -
										|				 |
										-				 -
								


		TL500/510:
		----------
        When the PWM output is not driven (high-Z), the charge current is 1V/10k=100mA>. In trickle charging monde (VBat <2.9V),
		the charge current is divided by 10, i.e. 10mA.
		The charge controller is a MCP73832-2AC which supports trickle charging at 1/10 of the nominal current.

		TL501/502:
		----------
        When the PWM output is not driven (high-Z), the default current setting 1V/2k55=392mA is too high so that the PWM must be
		active even in default charge mode.

		In trickle charging mode (VBat <2.9V), the charge current is divided by 10, i.e. 39mA (TL501/502).

		In normal temperature conditions, on detection of the charger (VCHRG_SENSE is above V_CHARGE_MIN), the PWM
		is to set 75% duty cycle which results in default charging (226mA).


		TL512:
		------
		The charge controller is a MCP73832-2DC which does not use trickle charging. To avoid issues with the nRF52840 which draws 
		around 15...20mA current at about 1.7V while its DC/DC starts up, the default current w/o FW support is set to 55mA.
		The same is achieved with PWM in open-source at 0% duty cycle or PWM in open-drain at 100% duty cycle.

		To lower the charge current, the PWM is configured as open-source, i.e. it pull actively high but does not pull low. Thus, 
		with a duty cycle of about 35% a current of 0 is reached. Increasing the duty cycle further shuts down the charger.
		
		To increase the charge current, the PWM is configured in open-drain, i.e. it pull actively low but does not pull high.
		With a duty cycle of 0%, the charge current is 204mA.


		When not being on the charger, the PWM output is set continuously high (push) to disable charging.

		In extended temperature conditions, the PWM is preset to hold the battery voltage at 4.0V.
		In extreme temperature conditions, the PWM output is set continuously high to disable charging.
*/

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

#include "tracker.h"

/* nRF SDK include files */
#undef  NRF_LOG_DEFAULT_LEVEL
#define NRF_LOG_DEFAULT_LEVEL		NRF_LOG_DEFAULT_DEBUG
#define NRF_LOG_MODULE_NAME 		CHRG
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrfx_lpcomp.h"
#include "nrf_drv_lpcomp.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* Device specific include files. */
#include "custom_board.h"
#include "drv_uart.h"
#include "drv_adc.h"
#include "drv_pwm.h"

#include "ble_ctrl.h"
#include "charger.h"
#include "config.h"
#include "ctrl.h"
#include "drv_nvm.h"
#include "gsm.h"
#include "parser.h"
#include "rtc.h"
#include "utils.h"
#include "trace.h"
#include "tracemsg.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Initialise the charger driver. The ADC needs to be initialised before. */
void vChargerInit( UBaseType_t uxPriority );

/* Select all devices featuring the HW component change allowing to use PWM
   as charge current control. */
#if !defined ( TL500 ) && !defined ( TL510 )
	static void	vSetChargeDetectionToLPCOMP( void );
	static void vSetChargeDetectionToADC( void );
#endif

/* Battery measurement time callback. */
void vBattMeasTimerCallBack( TimerHandle_t xTimer );

/* Charge control time callback. */
void vChrgCtrlTimerCallBack( TimerHandle_t xTimer );

/* Estimate the initial charger current. */
unsigned char ucEstimateInitialChrgCurrent( void );

/* Battery charge current control loop. Calculate the next charge current value. */
unsigned char ucCalcBattChrgSetting( unsigned short usActBattChrgCurrent, signed short sVoltageError );

/* Charger callback. */
void vChargerHandler( void );

/* Battery voltage read handler. Called periodically from RTC. */
void vVBatHandler( void );

/* Obtain charger state from variable. */
bool bGetOnCharger( void );

/* Obtain charger state from ADC. */
bool bDetectOnCharger( bool bProtectedAccess );

/* Set charger to the requested mode. */
void vSetChargerMode( enum xCHRG_STATUS xChrgMode, unsigned char ucChargeSetting );

/* Obtain charger status. */
enum xCHRG_STATUS xGetChargerStatus( void );

bool bGetBatteryEndOfCharge( bool bProtectedAccess );

/* Return simultaneous charging state and battery full. */
bool bIsChargingAndBatteryFull( void );

/* Set the for battery voltage status in xBatVoltStatus. */
void vSetBatteryVoltageStatus( bool bProtectedAccess );

/* Set the battery temperature status in xBatTempStatus. */
void vSetBatteryTempStatus( bool bProtectedAccess );

/* Return battery last gasp status. */
bool bBatteryLastGasp( void );

/* Return battery depleted status. */
bool bBatteryDepleted( void );

/* Return battery low status. */
bool bBatteryEmpty( void );

/* Return battery low state. */
bool bBatteryLow( void );

/* Return battery full state. */
bool bBatteryFull( void );

/* Return battery full at 98% state. */
bool bBattery98Pct( void );

/* Return if temperature is outside charging conditions. */
bool bTemperatureOutsideChrg( void );

/* Return if temperature is outside operating conditions. */
bool bTemperatureOutsideOp( void );

/* Return if temperature is below operating conditions. */
bool bTemperatureBelowOp( void );

/* Return if temperature is above operating conditions. */
bool bTemperatureAboveOp( void );

/* Check if battery voltage and temperature allow for booting the GSM module. If not, wait until both
   conditions are good. */
void vCheckVoltageAndTemperature( void );

/* Get the current battery voltage. */
short sGetBattVoltage( void );

/* Return the current battery charge state estimation. */
short sGetBattCharge( void );

/* Get the battery temperature. */
short sGetBatteryTemperature( void );

#if !defined ( TL500 ) && !defined ( TL510 )
	static void vChrgIrqHandler( nrf_lpcomp_event_t xLpcompEvent );
#endif

/* The BLE advertising report handling task. */
static portTASK_FUNCTION_PROTO( vChrgTask, pvParameters );
/*-----------------------------------------------------------*/

/* Global variables. */
/* CHRG command queue handle. */
QueueHandle_t					xChrgCmdQueue; 	

/* Timer handle for the battery measurement timer. */
TimerHandle_t 					xBattMeasTimer;

/* Timer handle for the charge control timer. */
TimerHandle_t 					xChrgCtrlTimer;

/* Flags indicating the battery voltage status. */
static struct xBAT_V_STATUS		xBatVoltStatus;

/* Flags indicating the battery temperature status. */
static struct xBAT_T_STATUS		xBatTempStatus;

/* Status of the charger. */
static enum xCHRG_STATUS		xChrgMode;

/* Battery voltage. */
static short 					sBattVoltage;

/* Backup of the temperature-outside-spec flag. */
static bool						bPrevTemperatureOutsideOp;

/* Backup of the battery-depleted flag. */
static bool						bPrevBatteryDepleted;

/* Charge current setting used in the control loop. */  
unsigned char					ucActBattChrgSetting;
/*-----------------------------------------------------------*/

/* Initialise the charger module. The ADC and PWM need to be initialised before. */
void vChargerInit( UBaseType_t uxPriority )
{
	/* Create a queue to the CHRG task. */
	xChrgCmdQueue = xQueueCreate( chrgCMD_QUEUE_SIZE, ( unsigned portBASE_TYPE ) sizeof( enum xCHRG_CMD ) );

	xBattMeasTimer = xTimerCreate
							( "BATT", 						/* Timer name for debug. */
							  BATT_MEAS_INTERVAL,			/* Interval between battery measurements. */
							  pdTRUE,						/* Periodic. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  vBattMeasTimerCallBack		/* Callback for the battery measure timer. */
							);	
	
	xChrgCtrlTimer = xTimerCreate
							( "CHRG", 						/* Timer name for debug. */
							  CHRG_CTRL_LOOP_INTERVAL,		/* Interval for running the charge current control loop. */
							  pdTRUE,						/* Periodic. */
							  ( void * )0,					/* Init for the ID / expiry count. */
							  vChrgCtrlTimerCallBack		/* Callback for the charge control loop timer. */
							);	
	
	vSetBatteryVoltageStatus( UNPROTECTED );
	vSetBatteryTempStatus( UNPROTECTED );

	xBatVoltStatus.bChargingAndBatteryFull 	= bGetBatteryEndOfCharge( UNPROTECTED );
	xBatVoltStatus.bCharging 				= bDetectOnCharger( UNPROTECTED );
	
	#if !defined ( TL500 ) && !defined ( TL510 )
		/* In case the device is not on the charger while initialising, configure the charge detection to use 
		   a GPIO interrupt. */
		if ( !bGetOnCharger() )
		{
			vSetChargeDetectionToLPCOMP();
		}
	#endif

	sBattVoltage = uiReadADC( VBAT_ADC, UNPROTECTED );

	bPrevTemperatureOutsideOp = bTemperatureOutsideOp();
	bPrevBatteryDepleted = bBatteryDepleted();

	ucActBattChrgSetting = 0;

	#if defined ( TL501 ) || defined ( TL502 ) || defined ( TL512 ) 
		vSetChargerMode( CHRG_DISABLED, 0 );
	#endif
	
	/* The Charger task is spawned here. */
	xTaskCreate( vChrgTask, "CHRG", chrgSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );

	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Battery measurement time callback. 

   Triggers battery voltage and temperature measurements in the charger task.
*/
void vBattMeasTimerCallBack( TimerHandle_t xTimer )
{
	enum xCHRG_CMD 		xChrgCmd;

	( void )xTimer;

	/* Ask the charge controller to perform a battery voltage and temperature measurement. */
	xChrgCmd = CHRG_BAT_MEAS;
	xQueueSend( xChrgCmdQueue, &xChrgCmd, 0 ); 
}
/*-----------------------------------------------------------*/

/* Charge control timer callback. 

   Triggers a single run of the charger control loop in the charger task.
*/
void vChrgCtrlTimerCallBack( TimerHandle_t xTimer )
{
	enum xCHRG_CMD 		xChrgCmd;

	( void )xTimer;

	/* Ask the charge controller to perform a charger control loop run. */
	xChrgCmd = CHRG_CTRL_LOOP;
	xQueueSend( xChrgCmdQueue, &xChrgCmd, 0 ); 
}
/*-----------------------------------------------------------*/


#if !defined ( TL500 ) && !defined ( TL510 )
	/* Set the charge detection to use an LPCOMP interrupt. 
	
	   The LPCOMP is set to detect any VCHRG voltage higher than about 1V. 
	   With the resistor divider, this means that the VCHRG_LPCOMP_PIN voltage is:
	   
	   			10k / (75k + 10k) * 1V = 118mV
	   
	   With the LPCOMP reference set to 
	   
	   VDD / 16 = 112.5mV --> threshold = 956mV
	   
	   */
	void vSetChargeDetectionToLPCOMP( void )
	{
		ret_code_t					xErrCode;
		nrf_drv_lpcomp_config_t 	xLpcompConfig = NRF_DRV_LPCOMP_DEFAULT_CONFIG;

		vADCChannelUnInit( VCHRG_ADC );
    	xLpcompConfig.input 		= VCHRG_LPCOMP_PIN;
		xLpcompConfig.hal.reference = NRF_LPCOMP_REF_SUPPLY_1_16;
		xLpcompConfig.hal.detection = NRF_LPCOMP_DETECT_UP;
		xLpcompConfig.hal.hyst 		= NRF_LPCOMP_HYST_50mV;

	    /* Initialize the LPCOMP driver. From this point LPCOMP on will be active and the
	       event handler will be executed when a voltage rise is detected. */
	    xErrCode = nrf_drv_lpcomp_init( &xLpcompConfig, vChrgIrqHandler );
	    APP_ERROR_CHECK( xErrCode );

	    nrf_drv_lpcomp_enable();		
	}
#endif
/*-----------------------------------------------------------*/

#if !defined ( TL500 ) && !defined ( TL510 )
	/* Set the charge detection to use the ADC. */
	void vSetChargeDetectionToADC( void )
	{
		nrf_drv_lpcomp_uninit();
		vADCChannelInit( VCHRG_ADC );
	}
#endif
/*-----------------------------------------------------------*/

/* Test, if the battery is low and enter the result in the static battery status variable.
   The function is called exclusively in the TMR task. Consequently, as TMR has the highest task
   priority, xBatVoltStatus cannot be accessed while this function is running. Therefore, access to
   xBatVoltStatus does not need to be protected against concurrent access.
*/
void vSetBatteryVoltageStatus( bool bProtectedAccess )
{
	short	sLocalBattVoltage;
	short	sBattDepletedThres;
	short	sBattEmptyThres;
	short	sBattLowThres;
	short	sBattFullThres;
	
	if ( bProtectedAccess == UNPROTECTED )
	{
		/* During startup, only unprotected accesses are possible, the NVDS cannot be accessed either.
		   Therefore, only use the default settings for voltage thresholds. */
		sLocalBattVoltage	= uiReadADC( VBAT_ADC, UNPROTECTED );
		sBattDepletedThres	= BATT_DEPLETED_THRES;
		sBattEmptyThres		= BATT_EMPTY_THRES;
		sBattLowThres		= BATT_LOW_THRES;
		sBattFullThres		= BATT_FULL_THRES;
	}
	else
	{
		sLocalBattVoltage	= sBattVoltage;
		sBattDepletedThres	= BATT_DEPLETED_THRES;
		sBattEmptyThres		= ( short )usConfigReadShort( &xNvdsConfig.usBattEmptyThres );
		sBattLowThres		= ( short )usConfigReadShort( &xNvdsConfig.usBattLowThres   );
		sBattFullThres		= ( short )usConfigReadShort( &xNvdsConfig.usBattFullThres  );
	}
	
	portENTER_CRITICAL();
	
	xBatVoltStatus.bBatteryLastGasp = ( sLocalBattVoltage < BATT_LAST_GASP_THRES );
	xBatVoltStatus.bBatteryDepleted = ( sLocalBattVoltage < sBattDepletedThres );
	xBatVoltStatus.bBatteryEmpty 	= ( sLocalBattVoltage < sBattEmptyThres );
	xBatVoltStatus.bBatteryLow 		= ( sLocalBattVoltage < sBattLowThres );
	xBatVoltStatus.bBatteryFull 	= ( sLocalBattVoltage > sBattFullThres );
	xBatVoltStatus.bBattery98Pct 	= ( sLocalBattVoltage > ( short )BATT_98PCT_THRES );
	
	portEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

/* Check the battery temperature and enter the result in the static battery status variable. 

   Temperature ranges:
																																temperature
   -----------------------------------------------------0C---------------------------------------------------------------------------->
   <-----bBatOpUnderTemp--->|<----bBatChrgUnderTemp--->|<-------normal charging------->|<---bBatChrgExtTemp--->|<---bBatOpOverTemp--->
                            |<-------------------------normal operation--------------------------------------->|
   bBatChrgExtTemp 		0				0							0							1						0
   bBatChrgUnderTemp   	1			 	1							0							0						0
   bBatOpOverTemp  		0				0 							0							0						1
   bBatOpUnderTemp 		1				0							0							0						0
						 
   
*/
void vSetBatteryTempStatus( bool bProtectedAccess )
{
	unsigned short			sNtc;
	unsigned portBASE_TYPE	uxI;
	
	/* Get the ADC reading for the NTC. Average over 4 measurements. */
	sNtc = 0;
	for ( uxI = 0; uxI < 8; uxI++ )
	{
		sNtc += uiReadADC( NTC_ADC, bProtectedAccess );
	}
	sNtc >>= 3;

	/* Set battery over/under temperature flags */
	portENTER_CRITICAL();
	xBatTempStatus.bBatOpUnderTemp 		= ( sNtc >  ( signed short )BATT_MIN_OP_TEMP );
	xBatTempStatus.bBatChrgUnderTemp 	= ( sNtc >= ( signed short )BATT_MIN_CHRG_TEMP );
	xBatTempStatus.bBatChrgExtTemp  	=     ( sNtc <  ( signed short )BATT_MAX_CHRG_TEMP )
										   && ( sNtc >= ( signed short )BATT_MAX_OP_TEMP );
	xBatTempStatus.bBatOpOverTemp  		= ( sNtc < ( signed short )BATT_MAX_OP_TEMP );
	portEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

/* Battery voltage read handler. Called periodically. */
void vVBatHandler( void )
{
	signed long	slTmpBattVoltage;
	signed		sAlpha;
	
	/* Get the battery voltage. */
	slTmpBattVoltage = uiReadADC( VBAT_ADC, PROTECTED );		

	/* Compensate for the ADC pin input leakage current. This current (0.01 ... 0.1uA) makes the measured voltage
	   look too low by a couple of mV. The value taken here is empiric. */
	slTmpBattVoltage += 8;
	
	/* Determine the filter constant. The filter speed is different for rising and falling edges. */
	if ( slTmpBattVoltage > sBattVoltage )
	{
		/* Rising edge. */
		sAlpha = ALPHA_RISING;
	}
	else
	{
		/* Falling edge. */
		sAlpha = ALPHA_FALLING;
	}
	
	/* Implement the 1st order IIR. The voltage is in 11-bit + sign fixed format. */
	slTmpBattVoltage = ( signed long )( 2047 - sAlpha ) * sBattVoltage + sAlpha * slTmpBattVoltage;
	sBattVoltage = slTmpBattVoltage >> 11;

	/* If the battery voltage is below the depletion level, stop the BLE. The GSM will check the status itself and stop also. 
	   Send a notification to the BLE only if there is a change in the state. */
	if ( bPrevBatteryDepleted != bBatteryDepleted() )
	{
		/* bBatteryDepleted() has changed: Take the corresponding action. */
		if ( bBatteryDepleted() && !xGetBleModuleState().bBleSuspended )
		{
			vNonBlockingSuspendAllBleActivity();
		}
		
		if ( !bBatteryDepleted() && xGetBleModuleState().bBleSuspended )
		{
			vResumeAllBleActivity();
		}
		
		bPrevBatteryDepleted = bBatteryDepleted();
	}
}
/*-----------------------------------------------------------*/

/* The device is entering FW_CONTROLLED charger mode: Estimate the initial charger current as percentage of the
   maximum charge current.
   
   The inital estimation is a simple linear approach: Full current at BATT_VOLT_MIN_REG (typ. 3.8V), 0 current at 
   BATT_VOLT_MAX_EXT_CHRG_THRES (typ. 4.0V).
*/
unsigned char ucEstimateInitialChrgCurrent( void )
{
	signed short 	sTmpBattVoltage;
	signed short 	sCtrlError;
	signed char		cInitialChargeSetting;
	
	/* Get the unfiltered battery voltage. We need the unfiltered version here as the filtered version reacts too slowly when the
	   device is being placed on the charger. */
	sTmpBattVoltage = uiReadADC( VBAT_ADC, PROTECTED );
	
	/* From that, the voltage control error. */
	sCtrlError = BATT_VOLT_MAX_EXT_CHRG_THRES - sTmpBattVoltage;
	
	/* Estimate the charge current reduction. Assume, that the battery internal resistance is about 10mOhm (a wild guess). 
	   delta(I) = sCtrlError / 10mOhm */
	if ( ( unsigned short )sTmpBattVoltage > BATT_VOLT_MAX_EXT_CHRG_THRES )
	{
		/* If the battery voltage is above 4.0V, do not even try to charge. Setting is thus 100%. */
		cInitialChargeSetting = BATT_CHRG_CURR_0MA;
	}
	else
	{
		if ( ( unsigned short )sBattVoltage < BATT_VOLT_MIN_REG )
		{
			/* If the battery voltage is below 3.9V, so full charge current is allowed. Setting is thus 0%. */
			cInitialChargeSetting = BATT_CHRG_CURR_NOM;
		}
		else
		{
			/* In between these extremes, the charge current must be regulated. */
			cInitialChargeSetting = BATT_CHRG_CURR_0MA + ( signed char )( ( long )sCtrlError * ( long )INITIAL_LOOP_CONST_NUM / ( long )INITIAL_LOOP_CONST_DENOM );
		}
	}
	
	/* Limit the calculated battery charge current to valid values. */
	cInitialChargeSetting = ( cInitialChargeSetting > BATT_CHRG_CURR_NOM ) ? BATT_CHRG_CURR_NOM : cInitialChargeSetting;
	cInitialChargeSetting = ( cInitialChargeSetting < BATT_CHRG_CURR_0MA ) ? BATT_CHRG_CURR_0MA : cInitialChargeSetting;
	
	return ( unsigned char )cInitialChargeSetting;
}						
/*-----------------------------------------------------------*/


/* Battery charge current control loop. Calculate the next charge current value. 

   usActBattChrgCurrent and xNewBattChrgSetting correspond to charge currents in percentage of the 
   maximum charge current.
*/
unsigned char ucCalcBattChrgSetting( unsigned short usActBattChrgCurrent, signed short sVoltageError )
{
	signed portBASE_TYPE	xNewBattChrgSetting;
	unsigned portBASE_TYPE	uxCurrentCorrStep;
	
	/* Calculate the new correction step width. */
	if ( abs( sVoltageError ) > BATT_VOLT_ERR_COARSE_REG )
	{
		uxCurrentCorrStep = COARSE_CHRG_CURR_STEP;
	}
	else
	{
		uxCurrentCorrStep = MIN_CHRG_CURR_STEP;
	}
	
	/* Correct the charging current. */
	if ( sVoltageError > 0 )
	{
		/* Battery voltage is too low. Increase the battery current. */
		xNewBattChrgSetting = usActBattChrgCurrent + uxCurrentCorrStep;
	}
	else
	{
		/* The battery voltage is too high. Decrease the battery current. */
		xNewBattChrgSetting = usActBattChrgCurrent - uxCurrentCorrStep;
	}
	
	/* Limit the calculated battery charge current to valid values. */
	xNewBattChrgSetting = ( xNewBattChrgSetting > BATT_CHRG_CURR_NOM ) ? BATT_CHRG_CURR_NOM : xNewBattChrgSetting;
	xNewBattChrgSetting = ( xNewBattChrgSetting < BATT_CHRG_CURR_0MA ) ? BATT_CHRG_CURR_0MA : xNewBattChrgSetting;
	
	return ( unsigned char )xNewBattChrgSetting;
}
/*-----------------------------------------------------------*/

/* Set charger to the requested mode. In CHRG_FW_CONTROLLED, ucChargeSetting corresponds to the percentage of the
   full charge current.

   The resulting charge current is roughly equivalent to
		TL500/510:		225mA * (ucChargeSetting/100)					0 <= ucChargeSetting <= 100
		TL501/502:		321mA * (ucChargeSetting/100) - 11mA			0 <= ucChargeSetting <= 75
		TL512:			204mA * (ucChargeSetting/100)					0 <= ucChargeSetting <= 100
*/
void vSetChargerMode( enum xCHRG_STATUS xNewChrgMode, unsigned char ucChargeSetting )
{
	if ( xNewChrgMode == CHRG_FW_CONTROLLED )
	{
		/* Manual (FW-controlled) mode. */
		if ( ucChargeSetting > BATT_CHRG_CURR_NOM )
		{
			ucChargeSetting = BATT_CHRG_CURR_NOM;
		}
		#if defined ( TL500 ) || defined ( TL510 )
			/* The TL500/510 always uses push-pull output. */
			vWritePwmDutyCycle( 100 - ucChargeSetting );
			nrf_gpio_cfg_output( BATT_CHRG );
			nrf_gpio_cfg( BATT_CHRG, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
		#elif defined ( TL501 ) || defined ( TL502 )
			/* For TL501/502, set the GPIO to high drive '1', no drive '0'. */
			vWritePwmDutyCycle( 100 - ucChargeSetting );
			nrf_gpio_cfg_output( BATT_CHRG );
			nrf_gpio_cfg( BATT_CHRG, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_D0H1, NRF_GPIO_PIN_NOSENSE);
		#else /* TL512 */
			/* The charger in the TL512 is more complicated to control. 
			   In function of the charge current setting, define the GPIO as pull high only or pull low only. 
			   	 0mA ...  55mA: 		GPIO pull high only, with a duty cycle between 35% and 0%. 
			   	55mA ... 204mA: 		GPIO pull low only, with a duty cycle between 100% and 0%. 
			   ucChargeSetting if mapped to the PWM as follow:
			     ucChargeSetting 0		PWM 35%			GPIO pull high only
				 ucChargeSetting 27		PWM 0%			GPIO pull high only or GPIO output disabled (== input)
				 ucChargeSetting 27		PWM 100%		GPIO pull low only or GPIO output disabled (== input)
				 ucChargeSetting 100	PWM 0%			GPIO pull low only 
			   The 26 setting can be achived in several ways. */
			if ( ucChargeSetting  < 26 )
			{
				/* Configure as disconnect when '0' but pull high when '1'. */
				vWritePwmDutyCycle( 35 - (int)( 1.35 * ucChargeSetting ) );
				nrf_gpio_cfg_output( BATT_CHRG );
				nrf_gpio_cfg( BATT_CHRG, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_D0H1, NRF_GPIO_PIN_NOSENSE);
			}
			else
			{
				/* Configure as pull low when '0' but disconnect when '1'. */
				vWritePwmDutyCycle( 100 - (int)( 1.35 * ( ucChargeSetting - 26 ) ) );
				nrf_gpio_cfg_output( BATT_CHRG );
				nrf_gpio_cfg( BATT_CHRG, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
			}
		#endif

		V_TRACE_PRINT_SHORT( TRACE_CHRG_BATTVOLTAGE, sBattVoltage, TRACE_UART );
		V_TRACE_PRINT_BYTE( TRACE_CHRG_BATTCHRGPWM, ucChargeSetting, TRACE_UART );
	}
	else
	{
		/* In any other mode, the parameter is ignored. */
		switch ( xNewChrgMode ) 
		{
			case CHRG_CTRL_OFF:		/* Set the control port for the case where the charge voltage is not available, i.e. low. */
									#if defined ( TL501 ) || defined ( TL502 ) || defined ( TL512 )
										nrf_gpio_cfg_input( BATT_CHRG, NRF_GPIO_PIN_NOPULL );
									#endif
									vStopPwm();
									V_TRACE_PRINT( TRACE_CHRG_OFF, TRACE_UART );
									break;

			case CHRG_ENABLED:		/* Enable charger in automatic mode, i.e. set the default current. */
									vWritePwmDutyCycle( 100 - BATT_CHRG_CURR_NOM );
									nrf_gpio_cfg_output( BATT_CHRG );
									#if defined ( TL512 )
										/* TL512: pull low only, no drive on high. */
										nrf_gpio_cfg( BATT_CHRG, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
									#elif defined ( TL500 ) || defined ( TL510 )
										/* TL500 and TL510: push/pull. */
										nrf_gpio_cfg( BATT_CHRG, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
									#else
										/* TL501 and TL502: push high only, no drive on low. */
										nrf_gpio_cfg( BATT_CHRG, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_D0H1, NRF_GPIO_PIN_NOSENSE);
									#endif
									V_TRACE_PRINT( TRACE_CHRG_ENABLED, TRACE_UART );
									break;

			case CHRG_DISABLED:
			default:				/* Disable charger, i.e. set the port to output and high. */
									vStopPwm();
									nrf_gpio_cfg_output( BATT_CHRG );
									nrf_gpio_pin_set( BATT_CHRG );
									nrf_gpio_cfg( BATT_CHRG, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
									V_TRACE_PRINT( TRACE_CHRG_DISABLED, TRACE_UART );
									break;
		}
	}
	
	xChrgMode = xNewChrgMode;
}
/*-----------------------------------------------------------*/

/* Charger handler. Called periodically. */
void vChargerHandler( void )
{
	enum xCTRL_EVENT 	xCtrlEvent;
	enum xCTRL_STATE	xState;
	enum xCHRG_CMD 		xChrgCmd;

	/* Set the for battery voltage status in xBatVoltStatus. */
	vSetBatteryVoltageStatus( PROTECTED );
	
	/* Get the system state from the Ctrl module. */
	xState = xGetCtrlState();
	
	/* Send charger state as event to the control state machine. */
	if ( !bDetectOnCharger( PROTECTED ) )
	{
		/* The device is not on the charger. If the system state does not correspond to this,
		   send an event to the control state machine. */
		if (    ( xState == CTRL_CHRG_NRML ) 
			 || ( xState == CTRL_CHRG_OOO )
		   )
		{
			/* Send event to the CTRL queue. */
			xCtrlEvent = CTRL_OFF_CHRG;
			xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );

			#if defined ( TL500 ) || defined ( TL501 ) || defined ( TL510 )
				xTimerChangePeriod( xChrgCtrlTimer, CHRG_CTRL_DET_INTERVAL, 0 );
			#else
				/* Send command to handle the off-charger transition to the CRHG command queue. */
				xChrgCmd = CHRG_OFF_CHRG;
				xQueueSend( xChrgCmdQueue, &xChrgCmd, 0 );
			#endif

		}
		
		portENTER_CRITICAL();
		xBatVoltStatus.bChargingAndBatteryFull = false;
		xBatVoltStatus.bCharging = false;
		portEXIT_CRITICAL();
		
		#if defined ( TL500 )
			/* Set charging mode to automatic for the next time the charger will be connected. */
			if ( xGetChargerStatus() != CHRG_ENABLED )
			{
				/* Set the charger mode to automatic. */
				vSetChargerMode( CHRG_ENABLED, 0 );
				NRF_LOG_DEBUG( "%i Charger enabled (normal mode).", ulReadRTC() );
			}
		#else
			/* TL501/502/512
			   Set charging mode to OFF. The charger will be enabled by FW the next time the VChrg is detected. */
			if ( xGetChargerStatus() != CHRG_CTRL_OFF )
			{
				/* Set charging mode to OFF. */
				vSetChargerMode( CHRG_CTRL_OFF, 0 );
				NRF_LOG_DEBUG( "%i Charger off (normal mode).", ulReadRTC() );
			}
		#endif
	}
	else
	{
		/* The device is on the charger. If the system state does not correspond to this,
		   send an event to the control state machine. */
		if (    ( xState != CTRL_CHRG_NRML ) 
			 && ( xState != CTRL_CHRG_OOO ) 
			 && ( xState != CTRL_PREALERT ) 
			 && ( xState != CTRL_PRESOS ) 
			 && ( xState != CTRL_ALERT ) 
			 && ( xState != CTRL_SOS ) 
		   )
		{
			xCtrlEvent = CTRL_ON_CHRG;

			/* Send event to the queue. */
			xQueueSend( xCtrlEventQueue, &xCtrlEvent, 0 );

			#if defined ( TL500 ) || defined ( TL510 )
				/* For TL500/510 and older devices, set the speed of the charge current loop control.
				   For TL501 and newer, this never changes as the detection is based on interrupts. */
				xTimerChangePeriod( xChrgCtrlTimer, CHRG_CTRL_LOOP_INTERVAL, 0 );
			#endif
		}
		
		portENTER_CRITICAL();
		xBatVoltStatus.bChargingAndBatteryFull = bGetBatteryEndOfCharge( PROTECTED );
		xBatVoltStatus.bCharging = true;
		portEXIT_CRITICAL();
		
		/* Test if the temperature is in the extended charging temperature range (above normal charging range but below the 
		   operating temperature limit). */
		if ( xBatTempStatus.bBatChrgExtTemp && !xBatTempStatus.bBatOpOverTemp )
		{
			/* We are in extended temperature range for battery charging. */
			/* Perform JEITA extended temperature range charging. */
			if ( xGetChargerStatus() != CHRG_FW_CONTROLLED )
			{
				NRF_LOG_DEBUG( "%i Entering FW-controlled mode", ulReadRTC() );
				NRF_LOG_FLUSH();

				/* Estimate the initial charge current. */
				ucActBattChrgSetting = ucEstimateInitialChrgCurrent();

				/* Set the charger mode to FW controlled with the initial charging current. */
				vSetChargerMode( CHRG_FW_CONTROLLED, ucActBattChrgSetting );
			}
			else
			{
				/* We are in the control loop. Check, if we are above or below the intended battery voltage.
				   Adjust the charging current accordingly. 
				   The charger task main loop takes care of calling vChargerHandler() with the speed rquired by the control
				   loop while on charge. */  
				unsigned char			ucNewBattChrgSetting;
				
				/* Calculate and set the new charge current. */
				ucNewBattChrgSetting = ucCalcBattChrgSetting( ucActBattChrgSetting, ( signed short )BATT_VOLT_MAX_EXT_CHRG_THRES - sBattVoltage );
				vSetChargerMode( CHRG_FW_CONTROLLED, ucNewBattChrgSetting );
				ucActBattChrgSetting = ucNewBattChrgSetting;
			}
		}
		else
		{
			/* Test, if the temperature is in the range where charging is absolutely prohibited. */
			if (    xBatTempStatus.bBatOpUnderTemp 
			     || xBatTempStatus.bBatChrgUnderTemp 
				 || xBatTempStatus.bBatOpOverTemp )
			{
				/* Disable the charger in this temperature range. */
				if ( xGetChargerStatus() != CHRG_DISABLED )
				{
					NRF_LOG_DEBUG( "%i Disabling charger: extreme temperature.", ulReadRTC() );
					NRF_LOG_FLUSH();
					
					vSetChargerMode( CHRG_DISABLED, 0 );
				}
			}
			else
			{
				/* The device is on the charger and the temperature is in the normal range where automatic charging is allowed. */
				#if defined ( TL500 ) || defined ( TL510 ) || defined ( TL512 )
					/* Check if there was a change in state. */
					if ( xGetChargerStatus() != CHRG_ENABLED )
					{
						vSetChargerMode( CHRG_ENABLED, 0 );

						NRF_LOG_DEBUG( "%i Enabling charger (normal mode).", ulReadRTC() );
						NRF_LOG_FLUSH();
					}
				#else
					/* TL501/502 */
					/* Enable automatic charging mode only if VChrg indicates that the charger is delivering more
					   current than supported by the Qi receiver, i.e. that the LiIon charger STC4054 is not in trickle
					   mode but the charge current control by PWM is not yet running. */
					if (    ( xGetChargerStatus() != CHRG_CTRL_OFF )
						 && ( xGetChargerStatus() != CHRG_ENABLED )
						 && ( !bTestMode )
					   )
					{
						/* At this point we do not know whether the STC4054 is in trickle charge mode or not.
						   Disable the FW control entirely. This way, the charge current is 30mA in trickle charge 
						   and (theoretically) 370mA otherwise. 370mA, however, is not provided by the Qi receiver so that VChrg
						   goes from 5V down to ~3.5V. This condition is detected in the next step and charge current limitation 
						   is enabled. */
						vSetChargerMode( CHRG_CTRL_OFF, 0 );

						NRF_LOG_DEBUG( "%i Charger set to off (normal mode).", ulReadRTC() );
						NRF_LOG_FLUSH();
						
						/* Give the VChrg a bit of time to settle. */
						vTaskDelay( 2 * portTICKS_PER_10MSEC );
					}
						
					/* While the charger is enabled, continuously monitor VChrg and enable current limiting by PWM if VChrg is less
					   than the nominal 5V. */
					if (    ( uiReadADC( VCHRG_ADC, PROTECTED ) < V_CHARGE_OK )
						 || ( uiReadADC( VBAT_ADC, PROTECTED ) > V_BAT_NORMAL_MODE )
					   )
					{
						if ( !bTestMode )
						{
							/* Normal mode, not in autotest on the charger. */
							if ( xGetChargerStatus() != CHRG_ENABLED )
							{
								vSetChargerMode( CHRG_ENABLED, 0 );

								NRF_LOG_DEBUG( "%i Enabling charger (normal mode).", ulReadRTC() );
								NRF_LOG_FLUSH();
							}
						}
						else
						{
							/* Autotest on the charger. Disable the charger as the PWM will interfere with the Bluetooth reception. */
							if ( xGetChargerStatus() != CHRG_DISABLED )
							{
								vSetChargerMode( CHRG_DISABLED, 0 );

								NRF_LOG_DEBUG( "%i Disabling charger (autotest).", ulReadRTC() );
								NRF_LOG_FLUSH();
							}
						}
					}
				#endif
			}
		}
	}
}
/*-----------------------------------------------------------*/

/* Obtain charger present state from ADC. */
bool bDetectOnCharger( bool bProtectedAccess )
{
	bool		bOnCharger;
	uint32_t	uiChrgVoltage;

	/* Check, if the charger voltage is above the minimum for on-charger detection. */
	uiChrgVoltage = uiReadADC( VCHRG_ADC, bProtectedAccess );
	
	/* Clamp the measured voltage to positive values. If the measured voltage is closed to 0,
	   an offset in the ADC might result in negative values. */
	if ( uiChrgVoltage > 0x8000 )
	{
		uiChrgVoltage = 0;
	}
	
	bOnCharger = ( uiChrgVoltage >= V_CHARGE_MIN );   
	return bOnCharger;
}
/*-----------------------------------------------------------*/

/* Obtain charger present state from variable. */
bool bGetOnCharger( void )
{
	return xBatVoltStatus.bCharging;
}
/*-----------------------------------------------------------*/

/* Obtain charger status. */
enum xCHRG_STATUS xGetChargerStatus( void )
{
	return xChrgMode;
}
/*-----------------------------------------------------------*/

/* Get battery full state when charging. */
bool bGetBatteryEndOfCharge( bool bProtectedAccess )
{
	/* Only check battery charger state when on charger. */
	if ( bDetectOnCharger( bProtectedAccess ) )
	{
		/* Check, if the charge control circuit indicates that the battery is being in normal charge. 
		   During end of charge, the controller's CHRG pin sinks 8...35uA, during charge the maximum voltage is 0.6V.
		   The controller's CHRG pin floats if there is insufficient voltage from the charger to launch the charge process.
		   We check here for 0.8V. */
		return ( uiReadADC( CHRG_STATE_ADC, bProtectedAccess ) >= CHRG_STATE_END_OF_CHARGE_MIN );
	}
	
	return false;
}
/*-----------------------------------------------------------*/

/* Return simultaneous charging state and battery full. */
bool bIsChargingAndBatteryFull( void )
{
	return xBatVoltStatus.bChargingAndBatteryFull;
}
/*-----------------------------------------------------------*/

/* Return battery last gasp status. */
bool bBatteryLastGasp( void )
{
	return xBatVoltStatus.bBatteryLastGasp;
}
/*-----------------------------------------------------------*/

/* Return battery depleted status. */
bool bBatteryDepleted( void )
{
	return xBatVoltStatus.bBatteryDepleted;
}
/*-----------------------------------------------------------*/

/* Return battery low status. */
bool bBatteryEmpty( void )
{
	return xBatVoltStatus.bBatteryEmpty;
}
/*-----------------------------------------------------------*/

/* Return battery low state. */
bool bBatteryLow( void )
{
	return xBatVoltStatus.bBatteryLow;
}
/*-----------------------------------------------------------*/

/* Return battery full state. */
bool bBatteryFull( void )
{
	return xBatVoltStatus.bBatteryFull;
}
/*-----------------------------------------------------------*/

/* Return battery full at 98% state. */
bool bBattery98Pct( void )
{
	return xBatVoltStatus.bBattery98Pct;
}
/*-----------------------------------------------------------*/

/* Return if temperature is outside charging conditions. */
bool bTemperatureOutsideChrg( void )
{
	return ( xBatTempStatus.bBatChrgUnderTemp || xBatTempStatus.bBatChrgExtTemp );
}
/*-----------------------------------------------------------*/

/* Return if temperature is outside operating conditions. */
bool bTemperatureOutsideOp( void )
{
	return ( xBatTempStatus.bBatOpUnderTemp || xBatTempStatus.bBatOpOverTemp );
}
/*-----------------------------------------------------------*/

/* Return if temperature is below operating conditions. */
bool bTemperatureBelowOp( void )
{
	return ( xBatTempStatus.bBatOpUnderTemp );
}
/*-----------------------------------------------------------*/

/* Return if temperature is above operating conditions. */
bool bTemperatureAboveOp( void )
{
	return ( xBatTempStatus.bBatOpOverTemp );
}
/*-----------------------------------------------------------*/


/* Check if battery voltage and temperature allow for booting the GSM module. If not, wait until both
   conditions are good.

   If the temperature is outside operating range, the BLE module is stopped too. It continues running, however,
   if only the voltage is too low. That way, the BLE continues transmitting until the voltage falls below 1.8V.
*/
void vCheckVoltageAndTemperature( void )
{
	bool 	bModulePwrOff;
	bool	bBatteryCritical;
	bool 	bTemperatureCritical;

	/* Determine if the battery is in critical state. */
	bBatteryCritical = bBatteryEmpty();
	bTemperatureCritical = bTemperatureOutsideOp();

	/* Get current GSM module power state. */
	bModulePwrOff = ( ( xGetGsmModuleState() & GSM_PWR_MSK ) == GSM_PWR_OFF );

	/* In any critical condition, switch first the GSM module off should it be on. */
	if ( bBatteryCritical || bTemperatureCritical )
	{
		/* If the battery is empty and the module is on, power it down now. */
		if ( bBatteryCritical )
		{
			V_TRACE_PRINT_SHORT( TRACE_GSM_LOW_BATTERY, sGetBattVoltage(), TRACE_UART_AND_FILE );
		}

		/* If the battery temperature is outside the operation limits and the module is on, switch it off. */
		if ( bTemperatureOutsideOp() )
		{
			V_TRACE_PRINT_SHORT( TRACE_GSM_EXCESSIVE_TEMPERATURE, sGetBatteryTemperature(), TRACE_UART_AND_FILE );
		}

		/* In any critical condition, switch first the GSM module off should it be on. */
		if ( !bModulePwrOff )
		{
			/* Switch off GSM module. */
			vGSMModulePowerDown( SOFT_POWER_OFF );
			bModulePwrOff = true;
		}
	}

	/* Now loop until the battery and temperature is good again. */
	while ( bBatteryCritical || bTemperatureCritical )
	{
		bBatteryCritical = bBatteryLow();
		bTemperatureCritical = bTemperatureOutsideOp();

		/* To recover from an empry battery status, the battery voltage must exceed the threshold for battery low. 
		   A module being on the charger might pass from battery empty status to just above the battery empty threshold.
		   Then, as soon as the GSM module is powered on, the voltage will drop down back to ~2.8V. */
		if ( bBatteryCritical )
		{
			V_TRACE_PRINT_SHORT( TRACE_GSM_LOW_BATTERY, sGetBattVoltage(), TRACE_UART_AND_FILE );
		}
		
		if ( bTemperatureCritical )
		{
			V_TRACE_PRINT_SHORT( TRACE_GSM_EXCESSIVE_TEMPERATURE, sGetBatteryTemperature(), TRACE_UART );
		}
		
		/* Wait a bit before testing the battery status again. */
		vTaskDelay( T_GSM_BAT_TEST );
	}
	
	/* If we were in a waiting loop before because of critical conditions,
	   push current voltage and temperature now for log purposes. */
	if ( bBatteryCritical || bTemperatureCritical )
	{
		V_TRACE_PRINT_SHORT( TRACE_CHRG_BATTVOLTAGE, sGetBattVoltage(), TRACE_UART_AND_FILE );
		V_TRACE_PRINT_SHORT( TRACE_CHRG_BATTTEMPERATURE, sGetBatteryTemperature(), TRACE_UART );
	}

	/* Now we are good to go. */
}
/*-----------------------------------------------------------*/

/* Return the current battery voltage. 
   Note that the sLocalVBat variable is updated in the TMR task.
   Hence, copying its value needs to be protected as it is a 16-bit variable. 
*/
short sGetBattVoltage( void )
{
	short volatile 	sLocalVBat;
	
	sLocalVBat = sBattVoltage;
	
	return sLocalVBat;
}
/*-----------------------------------------------------------*/

/* Return the current battery charge state estimation. 
   The charge estimation is based on the battery voltage and converted to charge state
   using the Prologium PLCB475255AANA typical V/C curve. The curve is piecewise linear approximated:
			voltage		V			C(%)
			4.35		2968		100
			4.28		2921		93.3
			4.1			2798		76.3
			3.99		2723		64.5
			3.93		2682		59.5
			3.59		2450		10.6
			3.52		2402		5.03
			3.41		2327		1.41
			3.3			2252		0
			
   The array values V are (voltage) / ADC_GAIN_VBAT / ADC_VREF * ADC_MAX_VALUE			/ 2 / 5 / 0.6 * 4095

   The returned charge level is fix point with 1 decimal place.
   
   TODO: Adapt the curve to different battery types.
*/
const short	sVoltageBoundary[] = { 2968, 2921, 2798, 2723, 2682, 2450, 2402, 2327, 2252 };
const short	sChargeBoundary[]  = { 1000,  933,  763,  645,  595,  106,  503,  141,    0 };

short sGetBattCharge( void )
{
	short volatile 				sLocalVBat;
	unsigned portBASE_TYPE		uxIdx;
	short						sCharge;
	
	sLocalVBat = sBattVoltage;
	sCharge = 0;
	
	for ( uxIdx = 1; uxIdx < sizeof( sVoltageBoundary ) / sizeof( sVoltageBoundary[ 0 ] ); uxIdx++ )
	{
		if ( sLocalVBat > sVoltageBoundary[ uxIdx ] )
		{
			sCharge = sChargeBoundary[ uxIdx ] + ( short )(   ( float )( sLocalVBat - sVoltageBoundary[ uxIdx ] )
												            / ( float )( sVoltageBoundary[ uxIdx - 1 ] - sVoltageBoundary[ uxIdx ] )
												            * ( float )( sChargeBoundary[ uxIdx - 1 ] - sChargeBoundary[ uxIdx ] )
														  );
			break;
		}		
	}
	
	return sCharge;
}
/*-----------------------------------------------------------*/

#if defined( TL500 ) || defined ( TL501 ) || defined ( TL502 )		/* 4.2V battery (Cameron Sino) with standard NTC. */

/* Get the NTC ADC value and calculate the battery temperature. 

   The parameters used in the following code are as follows:
		R1   = 43 kOhm		(resistance naming for TL510/512)
		RT0  = 10 kOhm
		kref = 1			(= Vcc / Vref. As the NTC signal is divided by 4 and the reference voltage is Vcc/4, kref = 1.)
		beta = 2950K
		Nmax = 2048
		T0   = 298.15K
		Toff = 273.15K
		
   The code uses 0.16 fixed-point format for the argument of the ln(). The ln() returns
   the value in 5.16 fixed-point format. To improve precision of the ln() calculation,
   the ln()'s argument is scaled with a factor of 1/8.
   The ln() is replaced by log2()/log2(e) (in another notation ld()/ld(e)).

   The exact formula is:

		T = T0 / ( 1 - T0/beta * ln[ RT0/R1 * (kref*Nmax/Nntc - 1) ] ) - Toff

		T = T0 / ( 1 - T0/(beta*ld(e)) * ld[ RT0/R1 * (kref*Nmax/Nntc - 1) ] ) - Toff

		T = beta*ld(e) / ( beta*ld(e)/T0 - ld[ RT0/R1 * (kref*Nmax/Nntc - 1) ] ) - Toff
   
   The function returns the temperature in 10 * decigrade.
*/
short sGetBatteryTemperature( void )
{
	unsigned short			sNtc;
	long					lY;
	long					lZ;
	unsigned portBASE_TYPE	uxI;
	
	/* Get the ADC reading for the NTC. Average over 4 measurements. */
	sNtc = 0;
	for ( uxI = 0; uxI < 8; uxI++ )
	{
		sNtc += uiReadADC( NTC_ADC, PROTECTED );
	}
	sNtc >>= 3;
	
	/* y = ((kref * Nmax * 0x10000) / Nntc - 0x10000) */
	lY = 0xfff0000 / sNtc - 0x10000;
	
	/* Multiply with R0/R1: z = z * R0 / R1. */
	lY = lY * 10;
	lY = lY / 43;
	
	/* Scale with a factor of 1/8: z = z/8. */
	lY = lY >> 3;
	
	/* z = ld(y) */
	lZ = -nlog2_16( ( unsigned short )lY );
	
	/* Correct the scaling factor of 8:
	   z = ld(y/8) + ld(8) = ld(y) + 3 */
	lZ = lZ + 0x30000;
	
	/* z = (beta * ld(e))/T0 - ld(y) = 14.27456 - ld(y) */
	lZ = 935495 - lZ;
	
	/* z = beta * ld(e) / (beta * ld(e) / T0 - ld(y)) = 4255.95 / (beta * ld(e) / T0 - ld(y)) */
	lZ = 2789179392 / lZ;
	
	/* z = z - 273.15 */
	lZ = lZ - 2731;
	
	return ( short )lZ;
}
/*-----------------------------------------------------------*/

#else						/* TL510/512 with battery cell and NTC NCU15XH103F6SRC mounted on PCB. */

/* Get the NTC ADC value and calculate the battery temperature. 

   The parameters used in the following code are as follows:
		R1   = 43 kOhm		(resistance naming for TL510)
		RT0  = 10 kOhm
		kref = 1			(= Vcc / Vref. As the NTC signal is divided by 4 and the reference voltage is Vcc/4, kref = 1.)
		beta = 3380K
		Nmax = 2048
		T0   = 298.15K
		Toff = 273.15K
		
   The code uses 0.16 fixed-point format for the argument of the ln(). The ln() returns
   the value in 5.16 fixed-point format. To improve precision of the ln() calculation,
   the ln()'s argument is scaled with a factor of 1/8.
   The ln() is replaced by log2()/log2(e) (in another notation ld()/ld(e)).

   The exact formula is:

		T = T0 / ( 1 - T0/beta * ln[ RT0/R1 * (kref*Nmax/Nntc - 1) ] ) - Toff

		T = T0 / ( 1 - T0/(beta*ld(e)) * ld[ RT0/R1 * (kref*Nmax/Nntc - 1) ] ) - Toff

		T = beta*ld(e) / ( beta*ld(e)/T0 - ld[ RT0/R1 * (kref*Nmax/Nntc - 1) ] ) - Toff
   
   The function returns the temperature in 10 * decigrade.
*/
short sGetBatteryTemperature( void )
{
	unsigned short			sNtc;
	long					lY;
	long					lZ;
	unsigned portBASE_TYPE	uxI;
	
	/* Get the ADC reading for the NTC. Average over 4 measurements. */
	sNtc = 0;
	for ( uxI = 0; uxI < 8; uxI++ )
	{
		sNtc += uiReadADC( NTC_ADC, PROTECTED );
	}
	sNtc >>= 3;
	
	/* y = ((kref * Nmax * 0x10000) / Nntc - 0x10000) */
	lY = 0xfff0000 / sNtc - 0x10000;
	
	/* Multiply with R0/R1: z = z * R0 / R1. */
	lY = lY * 10;
	lY = lY / 43;
	
	/* Scale with a factor of 1/8: z = z/8. */
	lY = lY >> 3;
	
	/* z = ld(y) */
	lZ = -nlog2_16( ( unsigned short )lY );
	
	/* Correct the scaling factor of 8:
	   z = ld(y/8) + ld(8) = ld(y) + 3 */
	lZ = lZ + 0x30000;
	
	/* z = (beta * ld(e))/T0 - ld(y) = 16.35522 - ld(y) */
	lZ = 1071856 - lZ;
	
	/* z = beta * ld(e) / (beta * ld(e) / T0 - ld(y)) = 4876.31 / (beta * ld(e) / T0 - ld(y)) */
	lZ = 3195738022 / lZ;
	
	/* z = z - 273.15 */
	lZ = lZ - 2731;
	
	return ( short )lZ;
}
#endif
/*-----------------------------------------------------------*/

#if !defined ( TL500 ) && !defined ( TL510 )
	/* Charger interrupt routine for detecting low-to-high transitions on the VCHRG_SENSE pin. */
	static void vChrgIrqHandler( nrf_lpcomp_event_t xLpcompEvent )
	{
		signed portBASE_TYPE 	xHigherPriorityTaskWoken = false;
		enum xCHRG_CMD			xChrgCmd;

		traceINTERRUPT_IN();

		if ( xLpcompEvent == NRF_LPCOMP_EVENT_UP )
		{
			/* We got an on-charge event. 
			Ask the charge controller to switch state to on-charge. */
			xChrgCmd = CHRG_ON_CHRG;
			xQueueSendFromISR( xChrgCmdQueue, &xChrgCmd, &xHigherPriorityTaskWoken ); 

			traceINTERRUPT_OUT();
			
			if( xHigherPriorityTaskWoken != false )
			{
				/* Call the interrupt safe yield function here. */
				portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
			}	
		}
	}
#endif
/*-----------------------------------------------------------*/

/* 
 * Charger handler task.
 */
static portTASK_FUNCTION( vChrgTask, pvParameters )
{
	enum xCHRG_CMD			xChrgCmd;
	
	/* Just to stop compiler warnings. */
	( void ) pvParameters;
	
	vTaskSetApplicationTaskTag( NULL, ( void * ) CHRG_TASK_TAG );	
	
	/* Wait untile the configuration handler is initialised. */
	while ( !bCheckConfigInitialised() || !bCheckTraceInitialised() )
	{
		vTaskDelay( 1 );
	}

	/* Start the battery measurement timer. 
	   Make sure that the first measurement is run as soon as possible. */
	( void )xTimerChangePeriod( xBattMeasTimer, usConfigReadShort( &xNvdsConfig.usBattLoopInt ) * portTICKS_PER_100MSEC, 0 );	   
	( void )xTimerStart( xBattMeasTimer, portMAX_DELAY );
	vBattMeasTimerCallBack( ( TimerHandle_t )0 );

	/* Configure and start the charge control timer. */
	#if defined ( TL500 ) || defined ( TL501 ) || defined ( TL510 )
		if ( bGetOnCharger() )
		{
			/* For TL500/510 and older devices, set the speed of the charge current loop control.
			   For TL501 and newer, this never changes as the detection is based on interrupts. */
			xTimerChangePeriod( xChrgCtrlTimer, CHRG_CTRL_LOOP_INTERVAL, 0 );
		}
		else
		{
			/* For TL500/510 and older devices, set the speed of the charge detection loop. */
			xTimerChangePeriod( xChrgCtrlTimer, CHRG_CTRL_DET_INTERVAL, 0 );
		}
	#endif

	#if defined ( TL500 ) || defined ( TL510 )
		/*	Make sure that the first control loop is run as soon as possible. */
		( void )xTimerStart( xChrgCtrlTimer, portMAX_DELAY );
	#else
		/* For TL501 and later, the charge control loop is not required to detect on-charge. Consequently, the loop
		   timer needs to run only while on-charge. */
		if ( bGetOnCharger() )
		{
			( void )xTimerStart( xChrgCtrlTimer, portMAX_DELAY );
		}
	#endif

	NRF_LOG_INFO( "Task started." );
	NRF_LOG_FLUSH();

	/* Task main loop. */
    while ( true )
	{
		/* Wait until there is a command to treat or until the queue receive times out. */
		if ( xQueueReceive( xChrgCmdQueue, &xChrgCmd, portMAX_DELAY ) != errQUEUE_EMPTY )
		{
			switch ( xChrgCmd )
			{
				case CHRG_ON_CHRG:			/* On charge detected. */
											NRF_LOG_DEBUG( "%i On-charger detected.", ulReadRTC() );
											NRF_LOG_FLUSH();
											#if !defined ( TL500 ) && !defined ( TL510 )
												vSetChargeDetectionToADC();
												vChargerHandler();
												( void )xTimerStart( xChrgCtrlTimer, portMAX_DELAY );
												#endif
											break;

				case CHRG_OFF_CHRG:			/* Off charge detected. */
											NRF_LOG_DEBUG( "%i Off-charger detected.", ulReadRTC() );
											NRF_LOG_FLUSH();
											#if !defined ( TL500 ) && !defined ( TL510 )
												vSetChargeDetectionToLPCOMP();
												( void )xTimerStop( xChrgCtrlTimer, portMAX_DELAY );
											#endif
											break;

				case CHRG_CTRL_LOOP:		/* In case the device is on the charger, call the charger handler to run the control loop. */
											NRF_LOG_DEBUG( "%i Charge control loop run.", ulReadRTC() );
											NRF_LOG_FLUSH();
											vChargerHandler();
											break;

				case CHRG_BAT_MEAS:			/* Measure battery voltage and temperature. */
											NRF_LOG_DEBUG( "%i Battery measurement run.", ulReadRTC() );
											NRF_LOG_FLUSH();

											/* Reload the timer in case its value had changed. */
											( void )xTimerChangePeriod( xBattMeasTimer, usConfigReadShort( &xNvdsConfig.usBattLoopInt ) * portTICKS_PER_100MSEC, 0 );	   

											/* Perform a battery measurement and update the voltage filter. */
											vVBatHandler();

											/* Set the for battery voltage status in xBatVoltStatus. */
											vSetBatteryVoltageStatus( PROTECTED );

											/* Set the battery temperature status in xBatTempStatus. */
											vSetBatteryTempStatus( PROTECTED );

											/* Check if BLE status and required status are aligned. */
											vNonBlockingAlignBleActivitySuspension();
											
											/* If the temperature is outside the operational specification, stop the BLE. The GSM will check the status itself and stop also. 
											Send a notification to the BLE only if there is a change in the state. */
											if ( bPrevTemperatureOutsideOp != bTemperatureOutsideOp() )
											{
												/* bTemperatureOutsideOp() has changed: Take the corresponding action. */
												if ( bTemperatureOutsideOp() && !xGetBleModuleState().bBleSuspended )
												{
													vNonBlockingSuspendAllBleActivity();
												}
												
												if ( !bTemperatureOutsideOp() && xGetBleModuleState().bBleSuspended )
												{
													vResumeAllBleActivity();
												}
												
												bPrevTemperatureOutsideOp = bTemperatureOutsideOp();
											}
											break;

				default:					break;
			}
		}
	}
}
/*-----------------------------------------------------------*/
