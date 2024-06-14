/*
 * Tracker Firmware
 *
 * DAC driver
 *
 */
#ifndef DRV_PWM_H
#define DRV_PWM_H

#include "FreeRTOS.h"
#include "custom_board.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */

/* Configure the PWM. */
extern void vPWMInit( void );

/* Enable a PWM channel. */
extern void vPWMEnableChannel( unsigned portBASE_TYPE uxDACChannelEnable );

/* Disable a PWM channel. */
extern void vPWMDisableChannel( unsigned portBASE_TYPE uxDACChannelEnable );

/* Write a PWM channel duty cycle. */
extern void vWritePwmDutyCycle( unsigned char ucPwmCycle );

/* Read a PWM channel duty cycle. */
unsigned char ucReadPwmDutyCycle( void );

/* Stop the PWM. */
void vStopPwm( void );

/* Return the stopped status of the  PWM. */
bool bPwmIsStopped( void );

#endif