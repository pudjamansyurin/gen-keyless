/*
 * _config.h
 *
 *  Created on: Aug 26, 2019
 *      Author: Puja
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "cmsis_os.h"
#include "main.h"
#include "_swv.h"

// macro to manipulate bit
#define SetBit(x) 								(1 << x)
#define SetBitOf(var, x) 					(var |= 1 << x)
#define ClearBitOf(var, x) 				(var &= ~(1 << x))
#define ToggleBitOf(var, x) 			(var ^= 1 << x)

// event list
#define EVENT_KEYLESS_BROADCAST		0x12345678
#define EVENT_KEYLESS_FINDER			0x00000002

// Function prototype
void BSP_Led_Write(uint8_t state);
void BSP_Led_Toggle(void);
void Set_PA(uint8_t state);

#endif /* CONFIG_H_ */