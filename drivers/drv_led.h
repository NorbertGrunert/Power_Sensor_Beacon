/*
 * Tracker Firmware
 *
 * LED driver header file
 *
 */
 
#ifndef LED_H
#define LED_H

#define ledON_INACTIVE		( ( TickType_t ) (  0.02 * portTICKS_PER_SEC ) )		/* INACTIVE:	off 2s, on 20ms, 1 flash */	
#define ledOFF_INACTIVE    	( ( TickType_t ) (  2    * portTICKS_PER_SEC ) )	
#define ledFC_INACTIVE 		( 1 )	

#define ledON_ACTIVE		( ( TickType_t ) (  0.02 * portTICKS_PER_SEC ) )		/* ACTIVE:		off 2s, on 20ms, 2 flashes */	
#define ledOFF_ACTIVE    	( ( TickType_t ) (  2    * portTICKS_PER_SEC ) )	
#define ledFC_ACTIVE   		( 2 )	

#define ledON_STILL			( ( TickType_t ) (  0.02 * portTICKS_PER_SEC ) )		/* STILL:		off 2s, on 20ms, 3 flashes */	
#define ledOFF_STILL    	( ( TickType_t ) (  2    * portTICKS_PER_SEC ) )	
#define ledFC_STILL    		( 3 )	

#define ledON_SLEEP			( ( TickType_t ) (  0.01 * portTICKS_PER_SEC ) )		/* SLEEP:		off 10s, on 10ms, 4 flashes */	
#define ledOFF_SLEEP    	( ( TickType_t ) ( 10    * portTICKS_PER_SEC ) )	
#define ledFC_SLEEP    		( 4 )	


#define ledON_STBY			( ( TickType_t ) (  0.02 * portTICKS_PER_SEC ) )		/* STANDBY:		off 10s, on 20ms, 1 flash */
#define ledOFF_STBY	    	( ( TickType_t ) ( 10    * portTICKS_PER_SEC ) )		/* same for STANDBY_CHRG. */
#define ledFC_STBY	    	( 1 )	

#define ledON_OOO			( ( TickType_t ) (  0.02 * portTICKS_PER_SEC ) )		/* OOO:			off 4s, on 20ms, 2 flashes */
#define ledOFF_OOO  	 	( ( TickType_t ) (  4    * portTICKS_PER_SEC ) )	 
#define ledFC_OOO	    	( 2 )	

#define ledON_CHRG_NRML		( ( TickType_t ) (  1    * portTICKS_PER_SEC ) )		/* CHRG_NRML:	off 6s, on 1s, 1 flash */
#define ledOFF_CHRG_NRML	( ( TickType_t ) (  6    * portTICKS_PER_SEC ) )	 
#define ledFC_CHRG_NRML    	( 1 )	

#define ledON_CHRG_OOO		( ( TickType_t ) (  1    * portTICKS_PER_SEC ) )		/* CHRG_OOO:	off 6s, on 1s, 2 flashes */
#define ledOFF_CHRG_OOO		( ( TickType_t ) (  6    * portTICKS_PER_SEC ) )	 
#define ledFC_CHRG_OOO    	( 2 )	


#define ledON_PREALERT		( ( TickType_t ) (  0.1  * portTICKS_PER_SEC ) )		/* PREALERT:	off 0.3s, on 0.1s, 1 flash */
#define ledOFF_PREALERT    	( ( TickType_t ) (  0.3  * portTICKS_PER_SEC ) )	
#define ledFC_PREALERT 		( 1 )	

#define ledON_ALERT			( ( TickType_t ) (  0.1  * portTICKS_PER_SEC ) )		/* ALERT:		off 0.3s, on 0.1s, 3 flashes */
#define ledOFF_ALERT    	( ( TickType_t ) (  0.3  * portTICKS_PER_SEC ) )	
#define ledFC_ALERT    		( 3 )	

#define ledON_PRESOS		( ( TickType_t ) (  0.1  * portTICKS_PER_SEC ) )		/* PRESOS:		off 0.3s, on 0.1s, 2 flash */
#define ledOFF_PRESOS    	( ( TickType_t ) (  0.3  * portTICKS_PER_SEC ) )	
#define ledFC_PRESOS 		( 2 )	

#define ledON_SOS			( ( TickType_t ) (  0.1  * portTICKS_PER_SEC ) )		/* SOS:			off 0.3s, on 0.1s, 4 flashes */
#define ledOFF_SOS	    	( ( TickType_t ) (  0.3  * portTICKS_PER_SEC ) )	
#define ledFC_SOS    		( 4 )	

#define ledON_BATEMPTY		( ( TickType_t ) (  0.01 * portTICKS_PER_SEC ) )		/* Battery low:	off 6s, on 10ms, 1 flash */
#define ledOFF_BATEMPTY    	( ( TickType_t ) (  6    * portTICKS_PER_SEC ) )	
#define ledFC_BATEMPTY   	( 1 )	

#define ledON_BATDISPLAY	( ( TickType_t ) (  0.03 * portTICKS_PER_SEC ) )		/* Battery charge display:	off 100ms, on 30ms */
#define ledOFF_BATDISPLAY   ( ( TickType_t ) (  0.1  * portTICKS_PER_SEC ) )	

#define ledON_OUTSIDEOPTEMP	( ( TickType_t ) (  0.01 * portTICKS_PER_SEC ) )		/* Temperature:	off 3s, on 10ms, 1 flash */
#define ledOFF_OUTSIDEOPTEMP (( TickType_t ) (  3    * portTICKS_PER_SEC ) )	
#define ledFC_OUTSIDEOPTEMP ( 1 )	



#define ledFLASH_PAUSE    	( ( TickType_t ) (  0.2  * portTICKS_PER_SEC ) )		/* Pause between flashes: 100ms */
/*-----------------------------------------------------------*/

#define	ledBLUE_LED_REQ		( 0x001 )
#define	ledBLUE_BLE_REQ		( 0x002 )
#define	ledBLUE_GSM_REQ		( 0x004 )
#define	ledRED_LED_REQ 		( 0x008 )
#define	ledRED_BLE_REQ		( 0x010 )
#define	ledRED_GSM_REQ 		( 0x020 )
#define	ledGREEN_GSM_REQ	( 0x040 )
#define	ledGREEN_BLE_REQ	( 0x080 )
#define	ledGREEN_LED_REQ	( 0x100 )
#define	ledEXCL_LED_REQ		( 0x200 )                        
/*-----------------------------------------------------------*/

/* Global variables. */

/* Blue LED requests. This variable allows several requesters in different tasks to control the blue LED. The LED status is 
   then a OR-combination of the individual requests. */
extern unsigned short	usLedReq;
/*-----------------------------------------------------------*/

/* Function prototypes. */
/* Switch on the red LED. */
extern void vSwitchOnLedRed( unsigned short usRequester );

/* Switch off the red LED. */
extern void vSwitchOffLedRed( unsigned short usRequester );

/* Switch on the green LED. */
extern void vSwitchOnLedGreen( unsigned short usRequester );

/* Switch off the green LED. */
extern void vSwitchOffLedGreen( unsigned short usRequester );

/* Switch on the blue LED. */
extern void vSwitchOnLedBlue( unsigned short usRequester );

/* Switch off the blue LED. */
extern void vSwitchOffLedBlue( unsigned short usRequester );

/* Let the LED start blinking. */
extern void vStartLedBlinking( void );

/* Let the LED stop blinking. */
extern void vStopLedBlinking( void );

/* LED driver init. */
extern void vLedInit( void );
/*-----------------------------------------------------------*/

#endif