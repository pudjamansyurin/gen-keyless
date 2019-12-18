/*
 * _config.c
 *
 *  Created on: Aug 26, 2019
 *      Author: Puja
 */

#include "_config.h"

void BSP_Led_Write(uint8_t state) {
	HAL_GPIO_WritePin(BSP_LED_GPIO_Port, BSP_LED_Pin, state);
}

void BSP_Led_Toggle(void) {
	HAL_GPIO_TogglePin(BSP_LED_GPIO_Port, BSP_LED_Pin);
}

void Set_PA(uint8_t state) {
	HAL_GPIO_WritePin(NRF24_PA_GPIO_Port, NRF24_PA_Pin, !state);
}
