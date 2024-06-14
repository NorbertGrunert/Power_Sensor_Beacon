/*
 * Tracker Firmware
 *
 * Evacuation module.
 *
 */

/* Standard include files. */
#include <stdlib.h>
#include <stdbool.h>

#include "tracker.h"

/* nRF SDK include files */
#define NRF_LOG_MODULE_NAME 		EVAC
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* Device specific include files. */
#include "drv_accelerometer.h"
#include "drv_nvm.h"
#include "drv_uart.h"
#include "drv_vibrator.h"

#include "ble_ctrl.h"
#include "ctrl.h"
#include "config.h"
#include "custom_board.h"
#include "evacuation.h"
#include "gsm.h"
#include "parser.h"
#include "trace.h"
#include "tracemsg.h"
#include "utils.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Initialise the evacuation module. */
void vEvacuationInit( void );

/* Start an evacuation. */
void vIndicateEvacuation( unsigned portBASE_TYPE uxNewEvacuationID );

/* Re-initisalise the evacuation ID and stop an ongoing evacuation. With the reset value, all incoming
   requests are accepted. */
void vStopTxEvacuation( void );

/* Reset evacuation on going on-charge. */
void vResetEvacuation( void );

/* Obtain the EVACUATION state. Returns true, if evacuation is ongoing. */
bool bGetEvacOngoing( void );

/* Get the evacuation ID. */
unsigned portBASE_TYPE uxGetEvacId( void );
/*-----------------------------------------------------------*/

/* Global variables. */
/* Evacuation ID to make sure a virbration is only given once per evacuation alert. */
volatile unsigned portBASE_TYPE	uxEvacuationID;

/* Evacuation is ongoing. */
volatile bool					bEvacOngoing;
/*-----------------------------------------------------------*/

/* Initialise the evacuation module. */
void vEvacuationInit( void )
{
	/* Initialise evacuation ID. */
	uxEvacuationID = EVAC_ID_UNSPECIFIC;
	
	bEvacOngoing = false;
}
/*-----------------------------------------------------------*/

/* Re-initisalise the evacuation ID and stop transmitting an ongoing evacuation via BLE. With the reset value, all incoming
   requests are accepted. */
void vStopTxEvacuation( void )
{
	enum xBLE_CMD			xBleCmd;
	
	/* Reset evac ongoing flag. */
	bEvacOngoing = false;

	if ( ( unsigned char )( usConfigReadShort( &xNvdsConfig.usBleEnable ) ) == true )
	{
		/* Request stopping the BLE EVAC beacon, if active. */
		xBleCmd = BLE_SR_EVAC_STOP;
		xQueueSend( xBleCmdQueue, &xBleCmd, bleCMD_BLOCKTIME ); 
	}
}
/*-----------------------------------------------------------*/
	
/* Indicate an evacuation.

   Check, if the evacuation alert associated with the given evacuation ID has already been given.
   If yes, ignore the new request. If not, store the new ID and give alert. 
   Finding out if the evacuation alert is stale is a bit tricky. Basically, the uxNewEvacuationID
   has to be higher than the previous uxEvacuationID, else we are receiving a stale alert via BT
   from another device. The trickiness comes from the fact that the variable is only 8bit long and we use
   only 8bit-1, i.e. mod255, to keep the init value of 255 = EVAC_ID_UNSPECIFIC reserved.
   With uxEvacuationID == EVAC_ID_UNSPECIFIC, any incoming evacuation request is honored.
   So uxNewEvacuationID=3 and uxEvacuationID=250 is valid while uxNewEvacuationID=3 and uxEvacuationID=5
   is not. Any alert coming with an evacuation ID with a distance less then MIN_EVAC_ID_DIST is refused
   as this is considered to be a stale alert/
   Test with MIN_EVAC_ID_DIST = 10:		
		uxNewEvacuationID =  4, uxEvacuationID = 3:     4 <= 3   &&   3 - 4 < 10 --> false --> valid alert
		uxNewEvacuationID =  3, uxEvacuationID = 250:   3 <= 250 && 250 - 3 < 10 --> false --> valid alert
		uxNewEvacuationID =  0, uxEvacuationID = 254:   0 <= 254 && 254 - 0 < 10 --> false --> valid alert
		uxNewEvacuationID =  1, uxEvacuationID = 11:    1 <= 11  &&  11 - 1 < 10 --> false --> valid alert
		
		uxNewEvacuationID =  1, uxEvacuationID = 9:     1 <= 9   &&   9 - 1 < 10 --> true  --> false alert
		uxNewEvacuationID =  3, uxEvacuationID = 3:     3 <= 3   &&   3 - 3 < 10 --> true  --> false alert

   An evacuation ID may not be reused until the module has either been reset or put onto the charger.
*/
void vIndicateEvacuation( unsigned portBASE_TYPE uxNewEvacuationID )
{
	enum xBLE_CMD			xBleCmd;
	
	if (   ( uxNewEvacuationID <= uxEvacuationID ) 
		&& ( ( signed short )uxEvacuationID - ( signed short )uxNewEvacuationID < ( signed short )MIN_EVAC_ID_DIST ) 
		&& !( uxEvacuationID == EVAC_ID_UNSPECIFIC ) )
	{
		/* Alert already given so ignore the request. */
		return;
	}
	
	V_TRACE_PRINT_BYTE( TRACE_VIBR_EVACUATION, uxNewEvacuationID, TRACE_UART );
	
	/* Set evac ongoing flag. */
	bEvacOngoing = true;
	
	/* Store the evacuation ID. */
	uxEvacuationID = uxNewEvacuationID;
	
	/* Start sending a distress beacon with evacuation alert message, if BLE is enabled. */
	if ( ( unsigned char )( usConfigReadShort( &xNvdsConfig.usBleEnable ) ) == true )
	{
		xBleCmd = BLE_SR_EVAC_START;
		xQueueSend( xBleCmdQueue, &xBleCmd, 0 ); 
	}
	
	/* Start the vibration. */
	vCustomVibrate( ucConfigReadByte( &xNvdsConfig.ucEvacVibrOn ), 
				    ucConfigReadByte( &xNvdsConfig.ucEvacVibrOff ), 
					ucConfigReadByte( &xNvdsConfig.ucEvacVibrRep ) );
}
/*-----------------------------------------------------------*/

/* Reset evacuation on going on-charge. */
void vResetEvacuation( void )
{
	uxEvacuationID = EVAC_ID_UNSPECIFIC;
	bEvacOngoing = false;
}
/*-----------------------------------------------------------*/

/* Get the evacuation ID. */
bool bGetEvacOngoing( void )
{
	/* Initialise evacuation ID. */
	return bEvacOngoing;
}
/*-----------------------------------------------------------*/
/* Get the evacuation ID. */
unsigned portBASE_TYPE uxGetEvacId( void )
{
	/* Initialise evacuation ID. */
	return uxEvacuationID;
}
/*-----------------------------------------------------------*/