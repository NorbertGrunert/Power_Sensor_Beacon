/*
 * BLE interface device
 *
 * General header file for misc. definitions
 *
 */
 
#ifndef MAIN_H
#define MAIN_H

#include "FreeRTOS.h"

#define VERSION						"NINA B3 V0.1"

#define	portTICKS_PER_SEC			( configTICK_RATE_HZ )
#define	portTICKS_PER_MIN			( 60 * portTICKS_PER_SEC )

/* Task stack sizes. The idle task and timer stack sizes are defined in FreeRTOSConfig.h */
#define bleSTACK_SIZE					( ( unsigned short ) 200 )
#define bleParserSTACK_SIZE				( ( unsigned short ) 200 )


/* Task priority definitions. */
#define SD_TASK_PRIORITY			( tskIDLE_PRIORITY + 4 )
#define BLE_ADHANDLER_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define BLE_PARSER_TASK_PRIORITY	( tskIDLE_PRIORITY + 3 )
#define LEDTGL_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
/* Timer priority is + 4 ! Defined in FreeRTOSConfig.h. */


enum xMEM_TYPE
{
	ROM,				/* Flash memory. */
	RAM,				/* RAM */
	EEPROM				/* EEPROM */
};

/* Flags for some data copy routines in different modules to translate apostrophes to quotation marks. */
#define TRANSLATE_APOSTR		true
#define NO_TRANSLATE_CHR		false

/* Helper macros to print defines during compile time. */
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)


#endif