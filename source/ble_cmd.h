/*
 * Tracker Firmware
 *
 * BLE Relayed Command header file
 *
 */ 
#ifndef BLE_CMD_H
#define BLE_CMD_H

/*-----------------------------------------------------------*/

/* Public Function prototypes. */
/* Initialize the BLE command handler. */
extern void vInitBleCmdHandler( void );

/* Parse the BLE command given as part of the BLE relay command. */
enum xAT_MSG_ID xParseBleCmd( signed char **ppcCmdStrg );

/* Handle a command relayed from the server and received via BLE. 
   The same command can only be treated once every 60 seconds. 
*/
extern void vTreatBleCmd( enum xAT_MSG_ID uxRelayedCmdIdx, unsigned short usRelayedCmdParam );
/*-----------------------------------------------------------*/

/* Global variables. */
/* BLE relayed command index and parameter. */
extern enum xAT_MSG_ID		xBleRlyCmd;
extern unsigned short 		usBleRlyCmdParam;
/*-----------------------------------------------------------*/

#endif