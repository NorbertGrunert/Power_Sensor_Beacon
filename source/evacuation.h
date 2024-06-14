/*
 * Tracker Firmware
 *
 * Evacuation module header file
 *
 */
 
#ifndef EVACUATION_H
#define EVACUATION_H
/*-----------------------------------------------------------*/

/* Initialisation value of the evacuation ID. With this value, all incoming evacuation requests
   are accepted. */
#define EVAC_ID_UNSPECIFIC				( 255 )

/* Minimal Evacuation ID distance for a valid alert. If a new ID lower than the current one
   arrives, it is declared stale (i.e. already seen before) if the difference between its ID
   and the last one seen before is less that this value. */
#define MIN_EVAC_ID_DIST				( 127 )

/* Vibration parameters for evacuation alert: on-time (*100ms), off-time (*100ms), repetitions */
#define EVAC_VIBR_ON 					( 30 )						/* 3 sec */
#define EVAC_VIBR_OFF					(  5 )						/* 500ms */
#define EVAC_VIBR_REP					(  6 )
/*-----------------------------------------------------------*/

/* Public function prototypes. */
/* Initialise the evacuation module. */
extern void vEvacuationInit( void );

/* Indicate an evacuation. */
extern void vIndicateEvacuation( unsigned portBASE_TYPE uxNewEvacuationID );

/* Re-initisalise the evacuation ID and stop an ongoing evacuation. With the reset value, all incoming
   requests are accepted. */
extern void vStopTxEvacuation( void );

/* Reset evacuation. */
extern void vResetEvacuation( void );

/* Obtain the EVACUATION state. Returns true, if evacuation is ongoing. */
extern bool bGetEvacOngoing( void );

/* Get the evacuation ID. */
extern unsigned portBASE_TYPE uxGetEvacId( void );
/*-----------------------------------------------------------*/

#endif
