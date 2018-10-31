/*
 * Tracker Firmware
 *
 * Parser header file
 *
 */ 
#ifndef BLE_PARSER_H
#define BLE_PARSER_H

#include "queue.h"

#define bleParserAT_CMD_QUEUE_SIZE	( 8 )
#define bleParserAT_CMD_BLOCKTIME	( ( TickType_t ) ( 5 * portTICKS_PER_SEC ) )	/* wait max. 5 second for At response indication queue to become available */

#define bleParserMAX_BCN_ENTRIES	( 5 )

/* Enumerate type containing identifiers for all AT messages to be received from the GSM/GPS device. */
enum xAT_MSG_ID
{
	AT_NOMSG,					
	AT_AT, 									
};

/* Definition of a structure field used for parsing and classifying incoming messages. */
struct xAT_CMD 
{
	char				*pcAtCommand;
	void				( *prvParamParser )( unsigned portBASE_TYPE xStrgIdx );
	enum xAT_MSG_ID		xAtMsgId;	
};


/* Public function prototypes. */
extern void vBleParserInit( UBaseType_t uxPriority );

/* Global variables. */
/* AT response indicator queue handle. */
extern QueueHandle_t 			xBleParserAtCmdQueue; 

#endif