/*
 * Tracker Firmware
 *
 * AES Cipher block driver.
 *
 */ 
 
 /* Standard include files. */
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"

#include "tracker.h"

/* nRF SDK include files */
#include "ssi_pal_types.h"
#include "ssi_pal_mem.h"
#include "sns_silib.h"
#include "ssi_aes.h"
#include "crys_aesccm.h"

#define NRF_LOG_MODULE_NAME 		DRV_AES
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* Device specific include files. */
#include "drv_AES.h"
#include "drv_nvm.h"
#include "rtc.h"
/*-----------------------------------------------------------*/

/* Function prototypes. */
extern void vAESInit( void );
extern void vAES_CBC_EncryptStart( uint8_t *pucKey );
extern bool bAES_CBC_EncryptBlock( uint8_t *pucPlaintext, uint8_t *pucCiphertext, int8_t cLength );
extern void vAES_CBC_EncryptStop( void );
extern bool bAES_CBC_DecryptBlock( uint8_t *pucCiphertext, uint8_t *pucPlaintext );
/*-----------------------------------------------------------*/

/* Local static variables. */
/* Initialisation vector used during Cipher Block Chaining. */
const uint8_t		 	ucIv[ AES_BLOCK_LENGTH ] = { 0x0A, 0x82, 0x59, 0x3C, 0xFF, 0x00, 0x7D, 0xEA,
			  									     0x2A, 0x90, 0x77, 0x53, 0xBE, 0x5A, 0xA0, 0x55 };
													 
/* Key used when the AES shall decrypt. This key is a modified version of the key. */
uint8_t  				ucLastSubkey[ AES_BLOCK_LENGTH ];
/*-----------------------------------------------------------*/

/* Global variables. */
const signed char 		cAesDefaultKey0[ AES_BLOCK_LENGTH ] = { 0x94, 0x74, 0xB8, 0xE8, 0xC7, 0x3B, 0xCA, 0x7D,
															    0x28, 0x34, 0x76, 0xAB, 0x38, 0xCF, 0x37, 0xC2 };
													 
const signed char 		cAesDefaultKey1[ AES_BLOCK_LENGTH ] = { 0x8A, 0x63, 0x17, 0xA5, 0xFC, 0x80, 0xD2, 0x55,
															    0x49, 0x68, 0x15, 0x13, 0xD2, 0x1B, 0x72, 0x33 };

/* CC310 context. */																  
SaSiAesUserContext_t    xContextID;

/* Mutex handle to protect AES access. */
SemaphoreHandle_t 		xMutexAES = NULL;

/* Define random generator workspace. */
CRYS_RND_WorkBuff_t		*pxRndWorkBuff;
CRYS_RND_State_t		*pxRndState;

#if defined(__CC_ARM)
CRYS_RND_State_t		rndState = {0};
CRYS_RND_WorkBuff_t		rndWorkBuff = {0};
#else
CRYS_RND_State_t		xRndState;
CRYS_RND_WorkBuff_t		xRndWorkBuff;
#endif
/*-----------------------------------------------------------*/

/* Initialise access arbitration to the AES. */
void vAESInit( void )
{
	SaSiStatus 				xSaSiStatus;

    pxRndState = &xRndState;
    pxRndWorkBuff = &xRndWorkBuff;
	
	/* Create a mutex to protect access to the AES so that it can be used from different tasks - notably BLE and BPR. */
	xMutexAES = xSemaphoreCreateMutex();

    /* Init SaSi library. */
    xSaSiStatus = SaSi_LibInit();
	if ( xSaSiStatus != SA_SILIB_RET_OK )
	{
		NRF_LOG_ERROR( "SaSi_LibInit failed with error 0x%x\n", xSaSiStatus );
		NRF_LOG_FLUSH();
		ASSERT( false );
	}
	
	/* Initialise CryptoCell random generator. */
    xSaSiStatus = CRYS_RndInit( pxRndState, pxRndWorkBuff );
    if ( xSaSiStatus != SA_SILIB_RET_OK ) 
	{
		NRF_LOG_ERROR( "CRYS_RndInit failed with error 0x%x\n", xSaSiStatus );
		NRF_LOG_FLUSH();
		ASSERT( false );
    }
	
	NRF_LOG_INFO( "Module initialised." );
	NRF_LOG_FLUSH();
}
/*-----------------------------------------------------------*/

/* Prepare AES CBC encryption. The ARM cryptocell 310 engine is initialised here and the mutex is taken. 
 *
 *  	pucKey          	Pointer to the key.
 */
void vAES_CBC_EncryptStart( uint8_t *pucKey )
{
	SaSiAesUserKeyData_t    xKeyData;	
	SaSiStatus 				xSaSiStatus;

	/* Obtain access to the AES. */
	configASSERT( xSemaphoreTake( xMutexAES, TO_MUTEX_AES ) );

	/* Call non-integrated APIs - first SaSi_AesInit */
	xSaSiStatus = SaSi_AesInit( &xContextID, SASI_AES_ENCRYPT, SASI_AES_MODE_CBC, SASI_AES_PADDING_NONE );	
	if ( xSaSiStatus != SA_SILIB_RET_OK )
	{
		NRF_LOG_ERROR( "%i SaSi_AesInit failed with error 0x%x\n", ulReadRTC(), xSaSiStatus );
		NRF_LOG_FLUSH();
		ASSERT( false );
	}
	
	/* Set the IV. */
	xSaSiStatus  = SaSi_AesSetIv( &xContextID, ( signed char * )ucIv );	
	if ( xSaSiStatus  != SA_SILIB_RET_OK )
	{
		NRF_LOG_ERROR( "%i SaSi_AesSetIv failed with error 0x%x\n", ulReadRTC(), xSaSiStatus );
		NRF_LOG_FLUSH();
		ASSERT( false );
	}
	
	/* Set key */
	xKeyData.pKey = pucKey;
	xKeyData.keySize = AES_BLOCK_LENGTH;
	xSaSiStatus  = SaSi_AesSetKey( &xContextID, SASI_AES_USER_KEY, &xKeyData, sizeof( xKeyData ) );	
	
	if ( xSaSiStatus  != SA_SILIB_RET_OK )
	{
		NRF_LOG_ERROR( "%i SaSi_AesSetKey failed with error 0x%x\n", ulReadRTC(), xSaSiStatus );
		NRF_LOG_FLUSH();
		ASSERT( false );
	}
}
/*-----------------------------------------------------------*/

/* Polled function that does AES CBC encryption on a single 128-bit data block.
 * Encryption might be in-place. Initialisation is assumed to have taken place before.
 *
 * This code is blocking and will dead lock if no interrupt flags are set.
 *
 *  	pucPlaintext    	Pointer to the pucPlaintext that shall be encrypted.
 *  	pucCiphertext   	Pointer to where in memory the pucCiphertext (answer) shall be stored.
 *  	cLength  			The number of bytes to encrypt.
 *
 *		returns true	The AES CBC encryption was successful.
 *		returns false   The AES CBC encryption was not successful.
 */
bool bAES_CBC_EncryptBlock( uint8_t *pucPlaintext, uint8_t *pucCiphertext,
                            int8_t cLength )
{
	uint8_t		ucDataInBuff[ AES_BLOCK_LENGTH ];
	SaSiStatus 	xSaSiStatus;
	
	/* Copy the plaintext data to the input buffer and pad with 0. */
	SaSi_PalMemSetZero( ucDataInBuff, AES_BLOCK_LENGTH );
	SaSi_PalMemCopy( ucDataInBuff, pucPlaintext, cLength );	

	xSaSiStatus  = SaSi_AesBlock( &xContextID,
						 ucDataInBuff,
						 SASI_AES_BLOCK_SIZE_IN_BYTES,
						 pucCiphertext );

	if ( xSaSiStatus  != SA_SILIB_RET_OK )
	{
		NRF_LOG_ERROR( "%i CRYS_AES_Block failed with error 0x%x\n", ulReadRTC(), xSaSiStatus );
		NRF_LOG_FLUSH();
		
		return false;
	}

	return true;
}
/*-----------------------------------------------------------*/

/* Stop AES CBC encryption. The AES engine is shut down here and the mutex is given.
 */
void vAES_CBC_EncryptStop( void )
{
	SaSiStatus 	xSaSiStatus;	
	
	xSaSiStatus = SaSi_AesFree( &xContextID );
	
	if ( xSaSiStatus  != SA_SILIB_RET_OK )
	{
		NRF_LOG_ERROR( "%i Failed to free CRYS_AES_Block resources\n", ulReadRTC(), xSaSiStatus );
		NRF_LOG_FLUSH();
		ASSERT( false );
	}

	/* Release AES mutex. */
	xSemaphoreGive( xMutexAES );
}
/*-----------------------------------------------------------*/


/* Prepare AES CBC dencryption. The ARM cryptocell 310 engine is initialised here and the mutex is taken. 
 *
 *  	pucKey          	Pointer to the key.
 */
void vAES_CBC_DecryptStart( uint8_t *pucKey )
{
	SaSiAesUserKeyData_t    keyData;	
	SaSiStatus 				xSaSiStatus;

	/* Obtain access to the AES. */
	configASSERT( xSemaphoreTake( xMutexAES, TO_MUTEX_AES ) );
	
	/* Call non-integrated APIs - first SaSi_AesInit */
	xSaSiStatus  = SaSi_AesInit( &xContextID, SASI_AES_MODE_CBC, SASI_AES_DECRYPT, SASI_AES_PADDING_NONE );
	
	if ( xSaSiStatus  != SA_SILIB_RET_OK )
	{
		NRF_LOG_ERROR( "%i SaSi_AesSetIv failed with error 0x%x\n", ulReadRTC(), xSaSiStatus  );
		NRF_LOG_FLUSH();
		ASSERT( false );
	}
	
	/* Set the IV. */
	xSaSiStatus  = SaSi_AesSetIv( &xContextID, ( signed char * )ucIv );
	
	if ( xSaSiStatus  != SA_SILIB_RET_OK )
	{
		NRF_LOG_ERROR( "%i SaSi_AesSetIv failed with error 0x%x\n", ulReadRTC(), xSaSiStatus  );
		NRF_LOG_FLUSH();
		ASSERT( false );
	}
	
	/* Set key */
	keyData.pKey = pucKey;
	keyData.keySize = AES_BLOCK_LENGTH;
	xSaSiStatus  = SaSi_AesSetKey( &xContextID, SASI_AES_USER_KEY, &keyData, sizeof( keyData ) );	
	
	if ( xSaSiStatus  != SA_SILIB_RET_OK )
	{
		NRF_LOG_ERROR( "%i SaSi_AesSetKey failed with error 0x%x\n", ulReadRTC(), xSaSiStatus  );
		NRF_LOG_FLUSH();
		ASSERT( false );
	}
}
/*-----------------------------------------------------------*/

/* Polled function that does AES CBC decryption on a single block.
 * Decryption might be in-place.
 *
 * Note: This code is blocking and will dead lock if no interrupt flags are set.
 *
 *		pucCiphertext   Pointer to the pucCiphertext that shall be decrypted.
 *		pucPlaintext    Pointer to where the pucPlaintext (answer) shall be stored.
 *
 *		returns true	If the AES CBC decryption was successful.
 *		returns false	If the AES CBC decryption was not successful.
 */
bool bAES_CBC_DecryptBlock( uint8_t *pucCiphertext, uint8_t *pucPlaintext )
{
	uint8_t		ucDataInBuff[ AES_BLOCK_LENGTH ];
	SaSiStatus 	xSaSiStatus;
	
	/* Copy the plaintext data to the input buffer and pad with 0. */
	xSaSiStatus  = SaSi_AesBlock( &xContextID,
								  pucCiphertext,
								  SASI_AES_BLOCK_SIZE_IN_BYTES,
								  pucPlaintext );

	if ( xSaSiStatus  != SA_SILIB_RET_OK )
	{
		NRF_LOG_ERROR( "%i CRYS_AES_Block decrypt failed with error 0x%x\n", ulReadRTC(), xSaSiStatus  );
		NRF_LOG_FLUSH();
		
		return false;
	}

	return true;
}
/*-----------------------------------------------------------*/
