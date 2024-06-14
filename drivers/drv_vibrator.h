/*
 * Tracker Firmware
 *
 * Vibrator driver header file
 *
 */
 
#ifndef VIBRATOR_H
#define VIBRATOR_H
/*-----------------------------------------------------------*/

/* Conversion constant. */
/* Time in 100ms during which the prealert vibrator in on. */
#define PREALERT_VIBRATOR_ON_TIME		( 2 )

/* Time in 100ms during which the prealert vibrator in off. */
#define PREALERT_VIBRATOR_OFF_TIME		( 8 )

/* Time lag for resetting the motor-turning status variable. This lag is necessary as the motor 
   continues turning a bit after having been switched off. Typically, a motor continues turning
   for about 33ms after having turned off the power supply. */
#define VIBR_MOTOR_OFF_LAG 				( 0.6 * portTICKS_PER_SEC )	/* s */

/* Threshold for passing the vibrator self-test. The value corresponds to the minimum acceleration
   difference to be achieved on at least one of the three axes x/y/z while the vibrator is on.
   The accelerometer is set to 8g full range.
		VIBR_SELF_TEST_THR = 0.1g / 8g * 32767 = 409.58
*/
#define VIBR_SELF_TEST_THR			0x199
/*-----------------------------------------------------------*/

/* Public function prototypes. */
/* Start vibrating in Pre-Alert state to notify the user of an imminent alert message. */
extern void vStartPrealertVibrate( void );

/* Let the vibrator stop emitting pre-alert. */
extern void vStopPrealertVibrate( void );

/* Perform a commanded, custom vibration sequence. */
extern void vCustomVibrate( unsigned portBASE_TYPE uxVibrationParamOn, unsigned portBASE_TYPE uxVibrationParamOff, unsigned portBASE_TYPE uxVibrationParamRep );

/* Vibrator driver init. */
extern void vVibratorInit( void );

/* Get status of the vibration motor. */
extern bool bVibrMotorIsTurning( void );

/* Vibrator self-test. */
extern bool bVibrTest( void );
/*-----------------------------------------------------------*/

/* Global variables. */
/* Indicator set if the vibration motor had been activated at least once. The indication is sent in the
   next IND field and reset afterwards. */
extern bool bVibrationMotorOn;
/*-----------------------------------------------------------*/

#endif