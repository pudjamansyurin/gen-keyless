/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "limits.h"
#include "_config.h"
#include "_nrf24l01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

osThreadId KeylessTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_IWDG_Init(void);
void StartKeylessTask(void const *argument);

/* USER CODE BEGIN PFP */
void PreSleepProcessing(void);
void PostSleepProcessing(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_RTC_Init();
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of KeylessTask */
	osThreadDef(KeylessTask, StartKeylessTask, osPriorityNormal, 0, 256);
	KeylessTaskHandle = osThreadCreate(osThread(KeylessTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
			{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
			{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
			{
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
			{
		Error_Handler();
	}
	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void)
{

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
			{
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
			{
		Error_Handler();
	}
	/** Enable the WakeUp
	 */
	if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
			{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
			{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NRF24_PA_GPIO_Port, NRF24_PA_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(BSP_LED_GPIO_Port, BSP_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA0 PA2 PA3 PA8
	 PA11 PA12 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_8
			| GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : NRF24_CSN_Pin NRF24_CE_Pin */
	GPIO_InitStruct.Pin = NRF24_CSN_Pin | NRF24_CE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : NRF24_IRQ_Pin */
	GPIO_InitStruct.Pin = NRF24_IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(NRF24_IRQ_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : NRF24_PA_Pin BSP_LED_Pin */
	GPIO_InitStruct.Pin = NRF24_PA_Pin | BSP_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN_SEAT_Pin BTN_ALARM_Pin */
	GPIO_InitStruct.Pin = BTN_SEAT_Pin | BTN_ALARM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB4 PB6 PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PH3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	extern nrf24l01 nrf;

	if (osKernelRunning()) {
		// handle NRF24 IRQ
		if (GPIO_Pin == NRF24_IRQ_Pin) {
			nrf_irq_handler(&nrf);
		}
		// handle buttons
		if (GPIO_Pin == BTN_ALARM_Pin || GPIO_Pin == BTN_SEAT_Pin) {
			xTaskNotifyFromISR(KeylessTaskHandle, GPIO_Pin, eSetBits, &xHigherPriorityTaskWoken);
		}
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void PreSleepProcessing(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	// Prepare to enter sleep mode
	//	HAL_SPI_MspDeInit(&hspi1);
	__HAL_RCC_SPI1_CLK_DISABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	__HAL_RCC_GPIOA_CLK_DISABLE();

	/* RTC Wakeup Interrupt Generation:
	 RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16

	 Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE))
	 Wakeup Time = Wakeup Time Base * WakeUpCounter
	 WakeUpCounter = Wakeup Time / Wakeup Time Base

	 To configure the wake up timer to:
	 Wakeup Time Base = 16 / (32.768KHz) = 0.48828125 ms
	 WakeUpCounter = X ms / 0.48828125 ms = ?

	 Therefore, maximum wake-up counter = 65535 = 0xFFFF
	 */
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, ((PING_INTERVAL_MS * 1000) / 488), RTC_WAKEUPCLOCK_RTCCLK_DIV16);

	// Suspend the tick
	HAL_SuspendTick();

	// Enter STOP 2 mode
	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
}

void PostSleepProcessing(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	// Resume the tick
	HAL_ResumeTick();

	// Re-Configure the system clock
	SystemClock_Config();

	/* Disable all used wake-up source */
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

	// Re-Init the SPI
	//	HAL_SPI_MspInit(&hspi1);
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartKeylessTask */
/**
 * @brief  Function implementing the KeylessTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartKeylessTask */
void StartKeylessTask(void const *argument)
{
	/* USER CODE BEGIN 5 */
	extern nrf24l01 nrf;
	uint32_t GPIO_Pin;
	GPIO_TypeDef *GPIO_Port;
	BaseType_t xResult;
	nrf24l01_config config;
	NRF_TX_PWR power;

	uint8_t msg;
	uint8_t payload_length = 8;
	uint8_t payload_tx[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	// set configuration
	nrf_set_config(&config, payload_tx, payload_length);
	// initialization
	nrf_init(&nrf, &config);

	/* Infinite loop */
	for (;;) {
		// Fed the dog
		HAL_IWDG_Refresh(&hiwdg);

		// set default power to lowest
		power = NRF_TX_PWR_M18dBm;
		msg = MSG_KEYLESS_BROADCAST;

		// check if event happen
		xResult = xTaskNotifyWait(0x00, ULONG_MAX, &GPIO_Pin, pdMS_TO_TICKS(0));

		// do this if events occurred
		if (xResult == pdTRUE) {
			// handle bounce effect
			osDelay(50);
			xTaskNotifyStateClear(NULL);

			// read again the value
			GPIO_Port = (GPIO_Pin == BTN_ALARM_Pin ? BTN_ALARM_GPIO_Port : BTN_SEAT_GPIO_Port);
			if (HAL_GPIO_ReadPin(GPIO_Port, GPIO_Pin)) {
				// only check when button HIGH
				if ((GPIO_Pin == BTN_ALARM_Pin)) {
					// Finder command
					msg = MSG_KEYLESS_FINDER;
					power = NRF_TX_PWR_0dBm;
				} else {
					// Seat Unlock command
					msg = MSG_KEYLESS_SEAT;
				}
			}
		}

		if (xResult == pdFALSE || msg != MSG_KEYLESS_BROADCAST) {
			// prepare payload
			payload_tx[payload_length - 1] = msg;
			Set_PA((power == NRF_TX_PWR_0dBm));

			// change "Power Down" -> "Standby I"
			nrf_power_up(&nrf, 1);

			// "Standby I" : set power
			nrf_set_tx_power(&nrf, power);

			// "TX Mode" : send packet (blocking)
			nrf_send_packet_noack(&nrf, payload_tx);

			// change "TX Mode" -> "Power Down"
			ce_reset(&nrf);
			nrf_power_up(&nrf, 0);

			//			// indicator
			//			for (int i = 0; i < ((msg + 1) * 2); i++) {
			//				BSP_Led_Toggle();
			//				osDelay(50);
			//			}
		}

		// Enter Stop-2 mode
		PreSleepProcessing();

		// Exit Stop-2 mode
		PostSleepProcessing();
	}
	/* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1)
		;

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
