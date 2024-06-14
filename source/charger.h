/*
 * TL500 Tracker Firmware
 *
 * Charger driver header file
 *
 */
 
#ifndef CHRG_H
#define CHRG_H

#define chrgCMD_QUEUE_SIZE				( 8 )

/* Periodicity of the charge current control loop in units of CHRG_SEC_WRAP. */
#if defined ( TL500 ) || defined ( TL501 ) || defined ( TL510 )
	/* For TL501 and older devices, the charge detection is based on polling. To guarantee a fast detection,
	   the detection loop must run once per second. */
	#define	CHRG_CTRL_DET_INTERVAL		( ( TickType_t )( 1 * portTICKS_PER_SEC ) )
#endif
/* Once on the charger, the control loop determines the charge current. That loop only needs to run once
   every eight seconds. */   
#define	CHRG_CTRL_LOOP_INTERVAL			( ( TickType_t )( 8 * portTICKS_PER_SEC ) )

/* Periodicity of the battery voltage measurement call in 100 millisecond steps. */
#define BATT_MEAS_INTERVAL				( ( TickType_t )( 80 ) )

/* Minimum voltage to detect charging. */
#define V_BAT_NORMAL_MODE				( 2184 )					/*  3.2V / ADC_GAIN_VBAT / ADC_VREF * ADC_MAX_VALUE = 2184 */
#define V_CHARGE_MIN					( 1606 )					/*  2.0V / ADC_GAIN_VCHRG / ADC_VREF * ADC_MAX_VALUE = 1606 */
#define V_CHARGE_OK						( 3453 )					/*  4.2V / ADC_GAIN_VCHRG / ADC_VREF * ADC_MAX_VALUE = 3453 */

#define BATT_DEPLETED_THRES				( ( unsigned short )1365 )	/* Battery fully depleted voltage limit in Volts (below this voltage no operation 
																	   is allowed, even while on charger): 
																		2.0V / ADC_GAIN_VBAT / ADC_VREF * ADC_MAX_VALUE = 1365 						*/
#define BATT_EMPTY_THRES				( ( unsigned short )2252 )	/* Battery empty voltage limit in Volts (determined by QTEL operational limit): 
																		3.3V / ADC_GAIN_VBAT / ADC_VREF * ADC_MAX_VALUE = 2252 						*/
#define BATT_LAST_GASP_THRES			( ( unsigned short )2273 )	/* Battery voltage limit for last gasp operations in Volts:
																		3.33V / ADC_GAIN_VBAT / ADC_VREF * ADC_MAX_VALUE = 2273						*/
#define BATT_LOW_THRES					( ( unsigned short )2491 )	/* Battery low voltage limit in Volts: 20%
																		3.65V / ADC_GAIN_VBAT / ADC_VREF * ADC_MAX_VALUE = 2491						*/
#if defined ( TL510 )												/* Battery cell with 4.35V max. voltage. 										*/
	#define BATT_FULL_THRES				( ( unsigned short )2784 )	/* Battery full voltage limit in Volts: 80%
																		4.08V / ADC_GAIN_VBAT / ADC_VREF * ADC_MAX_VALUE = 2784						*/
	#define BATT_98PCT_THRES			( ( unsigned short )2901 )	/* Battery full voltage limit in Volts: 98% 
																		4.25V / ADC_GAIN_VBAT / ADC_VREF * ADC_MAX_VALUE = 2901						*/
#else
	#define BATT_FULL_THRES				( ( unsigned short )2682 )	/* Battery full voltage limit in Volts: 80%
																		3.93V / ADC_GAIN_VBAT / ADC_VREF * ADC_MAX_VALUE = 2682						*/
	#define BATT_98PCT_THRES			( ( unsigned short )2798 )	/* Battery full voltage limit in Volts: 98% 
																		4.10V / ADC_GAIN_VBAT / ADC_VREF * ADC_MAX_VALUE = 2798						*/
#endif
																		
/* During end of charge, the controller's CHRG pin sinks 8...35uA, during charge the maximum voltage is 0.6V.
   The controller's CHRG pin floats if there is insufficient voltage from the charger to launch the charge process.
   To identify the battery full state, the CHRG_STATE pin voltage must be above 0.8V and below VCC -8uA* R6 (e.g. 1.536V	*/ 
#define CHRG_STATE_END_OF_CHARGE_MIN	( ( unsigned short ) 1365 )	/* 0.8 / ADC_GAIN_CHRG_STATE / ADC_VREF * ADC_MAX_VALUE = 1365 */

/* NTC voltage thresholds for different battery temperatures.
		The battery NTC is connected to VCC (1.8V nominal) via R1 (43K).
		for TL500/501/502:
			NTC beta:		beta = ln(Rt1 / Rt2) / (1/t1 - 1/t2) = 2950K (measured for Canon NB-11L (values used), 3435 from datasheet)
		for TL510/512:
			NTC beta:		beta = ln(Rt1 / Rt2) / (1/t1 - 1/t2) = 3380K (from datasheet)
		ADC reference:		Vref = VCC/4 = 1.8V / 4
		By using VCC/4 as reference, any variation in VCC cancels out.
		
		TL500/501/502:
		t2/°C	Rt2/kOhm	Rt2/(R1+Rt2)	Vntc/Vref/gain	N(ADC)	N(ADC)
		------------------------------------------------------------------
		-20		58.06		0.575			0.575			2352	0x930
		0		24.73		0.365			0.365			1495	0x5D7
		10		16.89		0.282			0.282			1154	0x482
		25		10			0.189			0.189			772		0x304
		45		5.37		0.111			0.111			454		0x1C6
		60		3.54		0.076			0.076			311		0x137

		TL510/512:
		t2/°C	Rt2/kOhm	Rt2/(R1+Rt2)	Vntc/Vref/gain	N(ADC)	N(ADC)
		------------------------------------------------------------------
		-20		75.02		0.636			0.636			2603	0xA2B
		0		28.22		0.396			0.396			1623	0x656
		10		18.23		0.298			0.298			1219	0x4C3
		25		10.00		0.189			0.189			773		0x304
		45		4.90		0.102			0.102			419		0x1A3
		60		3.04		0.066			0.066			270		0x10E

*/
#if defined ( TL510 ) || defined ( TL512 )
	#define BATT_MAX_OP_TEMP			( ( unsigned short )270 )	/*  60°C: Highest temperature at which the device may operate (battery discharge). */
	#define BATT_MAX_CHRG_TEMP			( ( unsigned short )419 )	/*  45°C: Highest temperature at which the battery may be unconditionally charged. 
																			  Extended temperature range charging (charging voltage limited to 4.0V) possible up to 60°C. */
	#define BATT_MIN_CHRG_TEMP			( ( unsigned short )1623 )	/*   0°C: Lowest temperature at which the battery may be charged. */
	#define BATT_MIN_OP_TEMP			( ( unsigned short )2603 )	/* -20°C: Lowest temperature at which the device may operate (battery discharge). */
#else
	#define BATT_MAX_OP_TEMP			( ( unsigned short )311 )	/*  60°C: Highest temperature at which the device may operate (battery discharge). */
	#define BATT_MAX_CHRG_TEMP			( ( unsigned short )454 )	/*  45°C: Highest temperature at which the battery may be unconditionally charged. 
																			  Extended temperature range charging (charging voltage limited to 4.0V) possible up to 60°C. */
	#define BATT_MIN_CHRG_TEMP			( ( unsigned short )1495 )	/*   0°C: Lowest temperature at which the battery may be charged. */
	#define BATT_MIN_OP_TEMP			( ( unsigned short )2352 )	/* -20°C: Lowest temperature at which the device may operate (battery discharge). */
#endif
	
#define BATT_VOLT_MAX_EXT_CHRG_THRES	( ( unsigned short )2730 )	/* Maximum battery charging voltage in extended temperature range in Volts: 4.0V / ADC_GAIN_VBAT / ADC_REF * ADC_MAX_VALUE = 2730 */
#define BATT_VOLT_MIN_REG				( ( unsigned short )2593 )	/* Minimum battery voltage for charging regulation: 3.8V / ADC_GAIN_VBAT / ADC_REF * ADC_MAX_VALUE = 2593 */
#define BATT_VOLT_ERR_COARSE_REG		( ( unsigned short ) 102 )	/* Battery voltage error for coarse charging regulation: 0.15V / ADC_GAIN_VBAT / ADC_REF * ADC_MAX_VALUE = 102 */

#define BATT_CHRG_CURR_0MA				( ( unsigned char )   0 )	/* Percentage of full charge current to obtain 0mA battery charge current. */
#if defined ( TL500 ) || defined ( TL510 ) || defined ( TL512 )
	#define BATT_CHRG_CURR_NOM			( ( unsigned char ) 100 )	/* Percentage of full charge current to obtain 225mA battery charge current (TL500/510/512). */
#else
	#define BATT_CHRG_CURR_NOM			( ( unsigned char )  72 )	/* Percentage of full charge current to obtain 225mA battery charge current (TL501/502). */
#endif
#define MIN_CHRG_CURR_STEP				( ( unsigned char )   1 )	/* Minimum step width for controlling the battery charge current: 2.25mA (225mA / 100) */
#define COARSE_CHRG_CURR_STEP			( ( unsigned char )  10 )	/* Coarse step width for controlling the battery charge current: 22.5mA. */
#define INITIAL_LOOP_CONST_NUM			( 1500 )					/* Constant to estimate the current control voltage from the battery voltage error. */
#define INITIAL_LOOP_CONST_DENOM		( 2048 )					/* Estimated as: 		C = 100% * sError[V] / (4.0V - 3.8V)
																			with:			sError[num] = sError[V] / ADC_GAIN_VBAT / ADC_VREF * ADC_MAX_VALUE
																										= sError[V] / 2 / 5 / 0.6V * 4095
																										= sError[V] * 682
																							C = 100 * sError[num] / 682 / 0.2
																							  = 0.7326 * sError[num]										*/																	
#define ALPHA_RISING					( 819 )						/* 0.4 */
#define ALPHA_FALLING					(  41 )						/* 0.02 */										   
/*-----------------------------------------------------------*/

/* Battery charger control status. */
enum xCHRG_STATUS
{
	CHRG_CTRL_OFF,				/* The charger is not powered. Set the control port hi-Z to avoid having leakage current. */
	CHRG_DISABLED,				/* The charger is completely disabled. */
	CHRG_ENABLED,				/* The charger is enabled and autonomously controlling the charging process. */
	CHRG_FW_CONTROLLED			/* The charger is controlled by FW. This mode is used in JEITA / IEC72133-2 extended temperature charging (45°C ... 60°C). */
};

/* Battery status variable definition. */
struct xBAT_V_STATUS
{
	bool	bBatteryLastGasp		: 1;
	bool	bBatteryDepleted		: 1;
	bool	bBatteryEmpty 			: 1;
	bool	bBatteryLow 			: 1;
	bool	bBatteryFull 			: 1;
	bool	bBattery98Pct 			: 1;
	bool	bChargingAndBatteryFull : 1;
	bool	bCharging 				: 1;
};

struct xBAT_T_STATUS
{
	bool	bBatChrgExtTemp			: 1;
	bool	bBatChrgUnderTemp		: 1;
	bool	bBatOpOverTemp			: 1;
	bool	bBatOpUnderTemp			: 1;
};

/* Commands to the CHRG task. */
enum xCHRG_CMD
{
	CHRG_BAT_MEAS,						/* Measure battery voltage and temperature. */
	CHRG_ON_CHRG,						/* On-chgarger detected. */ 
	CHRG_OFF_CHRG,						/* Off-charger detected. */
	CHRG_CTRL_LOOP						/* Trigger one run of the charge control loop. */
};
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Initialise the charger driver. The ADC needs to be initialised before. */
extern void vChargerInit( UBaseType_t uxPriority );

/* Obtain charger state. */
extern bool bGetOnCharger( void );

/* Obtain charger status. */
extern enum xCHRG_STATUS xGetChargerStatus( void );

/* Return simultaneous charging state and battery full. */
extern bool bIsChargingAndBatteryFull( void );

/* Return battery last gasp status. */
extern bool bBatteryLastGasp( void );

/* Return battery depleted status. */
extern bool bBatteryDepleted( void );

/* Return battery low status. */
extern bool bBatteryEmpty( void );

/* Return battery low state. */
extern bool bBatteryLow( void );

/* Return battery full state. */
extern bool bBatteryFull( void );

/* Return battery full at 98% state. */
extern bool bBattery98Pct( void );

/* Return if temperature is outside charging conditions. */
extern bool bTemperatureOutsideChrg( void );

/* Return if temperature is outside operating conditions. */
extern bool bTemperatureOutsideOp( void );

/* Return if temperature is below operating conditions. */
extern bool bTemperatureBelowOp( void );

/* Return if temperature is above operating conditions. */
extern bool bTemperatureAboveOp( void );

/* Check if battery voltage and temperature allow for booting the GSM module. If not, wait until both
   conditions are good. */
extern void vCheckVoltageAndTemperature( void );

/* Get the current battery voltage. */
extern short sGetBattVoltage( void );

/* Return the current battery charge state estimation. */
extern short sGetBattCharge( void );

/* Get the battery temperature. */
extern short sGetBatteryTemperature( void );
/*-----------------------------------------------------------*/

#endif