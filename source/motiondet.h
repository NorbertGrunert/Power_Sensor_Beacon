/*
 * Tracker Firmware
 *
 * Motion detection header file
 *
 */
 
#ifndef MOTIONDET_H
#define MOTIONDET_H

#include <stdbool.h>

#include "ctrl.h"


/* Initial accelerometer settings. */
/* Accelerator sample rate. */
#define SAMPLE_RATE						( 400 )											/* Samples per second = Hz */
#define RANGE							( 0x7FFF / 8 )									/* Codes per g for +/-8g range for 12-bit words. */
#define RANGE_8BIT						( 0x7F / 8 )									/* Codes per g for +/-8g range for 8-bit words. */

#define RIGHT_FOOT						( 0 )
#define LEFT_FOOT						( 1 )

/* Conmmon default configuration for both STEP_DET and SOS_DET. */
#define STEP_SOS_WAKEUP_ACC				( 0x05 )		/* 0.314g * RANGE_8BIT */		/* Common wake-up acceleration for both step and SOS detection. */


/* Default configuraton for SOS_DET. */
#define SOS_ACC_THRES					( 0x4000 )		/* 4.0g * RANGE */				/* side acceleration for SOS tap. */
#define SOS_X_ACC_THRES					( 0x0333 )		/* 0.2g * RANGE */				/* min. acceleration during window before wake-up event for SOS tap. */
#define SOS_X_ACC_HIST_WIN				( 0b00000000111100000000000000000000 )		    /* Take acceleration values in history FIFO from 20 +1 to 23 values before the trigger.
										//  0   0	F   0   0   0   0   0 
																						   Note: Normally the left window is 31 but some leeway is needed for the trigger latency. */
#define SOS_Z_ACC_HI_THRES				( 0x2000 )		/* 2.0g * RANGE */				/* max. acceleration during window before wake-up event for SOS tap. */
#define SOS_Z_ACC_HI_HIST_WIN			( 0b00000000111111111111100000000000 )		    /* Take acceleration values in history FIFO from 11 +1 to 23 values before the trigger. */
 										//  0   0   F   F   F   8   0   0   
#define SOS_Z_ACC_LO_THRES				( 0x0999 )		/* 0.6g * RANGE */				/* min. acceleration during window before wake-up event for SOS tap. */
#define SOS_Z_ACC_LO_HIST_WIN			( 0b00000000111111111111100000000000 )		    /* Take acceleration values in history FIFO from 11 +1 to 23 values before the trigger. */
 										//  0   0   F   F   F   8   0   0   			
																						/* Note: Normally the left window is 31 but some leeway is needed for the trigger latency. */
#define SOS_MIN_INTERVAL				( 0.16 * portTICKS_PER_SEC )					/* seconds */
#define SOS_MAX_INTERVAL				( 2.5 * portTICKS_PER_SEC )						/* seconds */
#define SOS_NUM_PEAKS					( 3 )

                                                                        				
/* Default configuraton for STEP_DET. */
#define STEP_ACC_THRES					( 0x1800 )		/* 1.5g * RANGE */				/* acceleration for step. */
#define STEP_Y_ACC_THRES				( 0x04CC )		/* 0.3g * RANGE */				/* min. acceleration during window before wake-up event for step. */
#define STEP_Y_ACC_HIST_WIN				( 0b00000000000000111111111111100000 )		    /* Take acceleration values in history FIFO from 5 +1 to 17 values before the trigger. 										//  0   0   F   F   F   F   E   0   
										//  0   0	0   3   F   F   E   0 
																						   Note: Normally the left window is 31 but some leeway is needed for the trigger latency. */
#define STEP_MIN_INTERVAL				( 0.7 * portTICKS_PER_SEC )						/* seconds */
#define STEP_MAX_INTERVAL				( 3.5 * portTICKS_PER_SEC )						/* seconds */
#define ACTIVE_NUM_STEPS				( 6 )


/* Default configuraton for MOTION_DET. */
#define MOTION_ACC_THRES				( 0x02 )		/* 0.12g * RANGE_8BIT */		/* Minimum acceleration to detect on movement. 
																						   ATTENTION: Thresholds settings resulting in 0x01 do not work (LIS3DH bug)! */
#define MOTION_HOLDOFF					( 1 * portTICKS_PER_SEC )						/* seconds Holdoff after motion interrupt (i.e. the interval during which no further 
																						   motion interrupts are accepted. ) */
                                                                            			
/* Default configuraton for final abnormal position detection. */
#define ABNORMAL_POS_Z_ACC_THRES		( 0x7FF )										/* The value is calculated from 1g * cos(tilt angle) = 0x7FFF / 8 * cos(tilt angle).
																						   0x600 corresponds to 68 degrees, 0x7FF to 60 degrees. */
#define ABNORMAL_POS_SAMPLE_RATE_NRML	( 2.5 )											/* Samples per second = Hz in all states except SLEEP */
#define ABNORMAL_POS_SAMPLE_RATE_SLEEP	( 0.2 )											/* Samples per second = Hz in SLEEP */ 
#define ABNORMAL_POS_TIMER_NRML			( 1 / ABNORMAL_POS_SAMPLE_RATE_NRML * portTICKS_PER_SEC )
#define ABNORMAL_POS_TIMER_SLEEP		( 1 / ABNORMAL_POS_SAMPLE_RATE_SLEEP * portTICKS_PER_SEC )
#define ABNORMAL_DURATION				( 20  * ABNORMAL_POS_SAMPLE_RATE_NRML )			/* sample rate */
#define ABNORMAL_MAX_ORIENTATION_CHG	( 0x0130 )		/* 0.074g * RANGE */			/* Maximum variation on any axis between two acceleration samples allowed for 
																						   abnormal position detection. Excess on any axis will terminate detection. 
																						   Depending on the initial tilt angle and with a sample rate of 2.5Hz, this results
																						   in a angular speed of between 14.3deg/s and 55.4deg/s. */
#define MIN_ORIENTATION_CHG_CNT			( 2 )											/* Minimum number of consecutive values exceeding the threshold to act on them */
 
#define NOABN_EN						(  0 )											/* No-abnormal position detection (NOABN) command feature is disabled by default. */
#define NOABN_NUM_PEAKS					(  5 )
#define NOABN_MAX_PERIOD				( 20 * 60 )										/* Maximum duration of a NOABN period. */
#define ABN_POS_DET_WINDOW 				(  1 * 60 )										/* Time window after having detected an abnormal position during which the next abnormal
																						   position needs to be detected to maintained a NOABN period. */
#define CMD_ACK_VIBR_ON					(   3 )											/* Command acknowledge vibration: on 300ms */
#define CMD_ACK_VIBR_OFF				(   1 )											/* Command acknowledge vibration: off 100ms */
#define CMD_ACK_VIBR_REP				(  15 )											/* Command acknowledge vibration: repetitions */

/* Default configuraton for STILL_DET. */
#define STILL_DURATION					( 20 )											/* seconds */
                                    
/* Default configuraton for SLEEP_DET. */
#define SLEEP_DURATION					( 600 )											/* seconds (must be smaller than 600 seconds to avoid overrun) */

/* Default configuraton for IMMOBILITY_DET. */
#define IMMOBILITY_DURATION				( 40 )											/* seconds */

/* Acceleration and duration for ALERT_CNCL_DET. */
#define ALERT_CNCL_BY_TAP				( 0 )											/* Alert cancel done by horizontal tap. */
#define ALERT_CNCL_BY_MVMT				( 1 )											/* Alert cancel done by any movement. */	
#define ALERT_CNCL_ACC_THRES			( 0x10 )		/* 1g * RANGE_8BIT */				
#define ALERT_CNCL_Z_ACC_THRES			( 0x0B33 )		/* 0.7g * RANGE */				/* Maximum acceleration during window before wake-up event for alert cancel. */
#define ALERT_CNCL_Z_ACC_HIST_WIN		( 0b00000011111100000000000000000000 )		    /* Take acceleration values in history FIFO from 20 +1 to 26 values before the trigger. 
										//  0   3	F   0   0   0   0   0 
																						   Note: Normally the left window is 28 but some leeway is needed for the trigger latency. */
#define ALERT_CNCL_MIN_INTERVAL			( 0.16 * portTICKS_PER_SEC )					/* seconds */
#define ALERT_CNCL_MAX_INTERVAL			( 2.0 * portTICKS_PER_SEC )						/* seconds */
#define ALERT_CNCL_NUM_PEAKS			(  3 )
#define ALERT_CNCL_TIMEOUT				( 30 )											/* seconds */
#define SOS_CNCL_TIMEOUT				(  8 )											/* seconds */
/*-----------------------------------------------------------*/

/* Event types for step detection. The definition the different types  */
enum xSTEP_EVENT
{
	STEP_REGULAR		=  0,	/* Regular step detected. */
	STEP_TOO_FAR		=  1	/* From Standby state, resume normal operation. */
};
/*-----------------------------------------------------------*/

/* Public global variables. */
/* Absolute step count to be sent to server. */
extern unsigned short 	usAbsStepCount;

/* Shared variable (write in this module, read from LED module) indicating tilted device attitude. */
extern bool				bDeviceTilted;

/* Variables controlling the abnormal position detection diabled feature. */
extern bool				bNoAbnActive;

/* Variables controlling the abnormal position detection disabled feature. */
extern bool				bNoAbnActive;
/*-----------------------------------------------------------*/

/* Public function prototypes. */
/* Motion detection init. */
extern void vMDInit( void );

/* Start motion detection. */
extern void vMDStartDetection( void );

/* Disable motion detection logic. */
extern void vMDStopDetection( void );

/* Set the accelerometer to motion detection. */
extern void vMDConfigureMotion( void );

/* Set the accelerometer to step detection. TODO: Make local. */
extern void vMDConfigureStepSOS( void );

/* Set the accelerometer to abnormal position detection. */
extern void vMDConfigureAbnormal( void );

/* Set accelerometer to movement detection in SLEEP mode. */
extern void vMDEnterSleep( void );

/* Configure the device for STILL state. */
extern void vMDEnterStill( void );

/* Set the accelerometer to MOTION/STILL detection. */
extern void vMDEnterInactive( void );

/* Set accelerometer to INACT_DET. */
extern void vMDEnterActive( void );

/* Set the accelerometer to ALERT_CANCEL detection. */
extern void vMDEnterPreAlert( enum xCTRL_STATE xNewCtrlState );

/* Set the accelerometer to STEP detection. */
extern void vMDEnterAlert( void );

/* Motion detection init. */
void vMDInit( void );

#endif