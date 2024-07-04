/**
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#ifndef BOARD_CUSTOM_H
#define BOARD_CUSTOM_H

#ifdef __cplusplus
extern "C" {
#endif

// NINA-B3 board definition
#include "nrf_gpio.h"

// In this file PIN 25 is used as button SWITCH_1, if the GREEN led
// should be used it is possible to defined that one instead.
#define LEDS_NUMBER 		3
#define LED_1 				NRF_GPIO_PIN_MAP( 0, 13 )		// RED
#define LED_2 				NRF_GPIO_PIN_MAP( 1, 00 )		// BLUE
#define LED_3 				NRF_GPIO_PIN_MAP( 0, 25 )		// GREEN	

#define LEDS_ACTIVE_STATE 	0	

#define LEDS_LIST 			{ LED_1, LED_2, LED_3 }
#define LEDS_INV_MASK 		LEDS_MASK	

#define BSP_LED_0 			LED_1
#define BSP_LED_1 			LED_2
#define BSP_LED_2 			LED_3

// #define BUTTONS_NUMBER 	2
// #define BUTTON_1 		25 								// SWITCH_1
// #define BUTTON_2 		2 								// SWITCH_2
#define BUTTONS_NUMBER 		1
#define BUTTON_1 			2								// SWITCH_2

#define BUTTON_PULL 		NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

// #define BUTTONS_LIST 	{ BUTTON_1, BUTTON_2 }
#define BUTTONS_LIST 		{ BUTTON_1 }

#define BSP_BUTTON_0 		BUTTON_1
// #define BSP_BUTTON_1 	BUTTON_2

#define SENSOR_RXD 			NRF_GPIO_PIN_MAP( 0, 29 )
#define SENSOR_TXD 			NRF_GPIO_PIN_MAP( 1, 13 )
#define CTS_PIN_NUMBER 		NRF_GPIO_PIN_MAP( 1, 12 )
#define RTS_PIN_NUMBER 		NRF_GPIO_PIN_MAP( 0, 31 )
#define TRC_RXD				NRF_GPIO_PIN_MAP( 0, 31 )
#define TRC_TXD				NRF_GPIO_PIN_MAP( 1, 12 )	

#define HWFC 			true

#ifdef __cplusplus
}

#endif

#endif // NINA_B1_H