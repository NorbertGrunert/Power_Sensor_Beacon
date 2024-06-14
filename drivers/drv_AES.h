/*
 * Tracker Firmware
 *
 * AES driver
 *
 */ 
#ifndef DRV_AES_H
#define DRV_AES_H
/*-----------------------------------------------------------*/

/* nRF SDK include files */
#include "ssi_aes.h"
/*-----------------------------------------------------------*/

/* Length of one block. Always 128-bits (16 bytes). */
#define AES_BLOCK_LENGTH			SASI_AES_BLOCK_SIZE_IN_BYTES

/* Time-out (s) for gaining access to the AES. */
#define TO_MUTEX_AES				( 10 * portTICKS_PER_SEC ) 		
/*-----------------------------------------------------------*/

/* Public variables. */
extern const signed char cAesDefaultKey0[ AES_BLOCK_LENGTH ];
extern const signed char cAesDefaultKey1[ AES_BLOCK_LENGTH ];

/*-----------------------------------------------------------*/

/* Function prototypes. */
extern void vAESInit( void );
extern void vAES_CBC_EncryptStart( uint8_t *pucKey );
extern bool bAES_CBC_EncryptBlock( uint8_t * pucPlaintext, uint8_t * pucCiphertext, int8_t cLength );
extern void vAES_CBC_EncryptStop( void );
extern void vAES_CBC_DecryptStart( uint8_t *pucKey );
extern bool bAES_CBC_DecryptBlock( uint8_t *pucCiphertext, uint8_t *pucPlaintext );

#endif
