/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include "printf.h"  // https://embeddedartistry.com/blog/2019/11/06/an-embedded-friendly-printf-implementation/
#include "mcp9808.h" // https://embeddedespresso.com/temperature-measurement-never-so-easy-with-stm32-and-mcp9808/

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	Init = 0,
	None  = 1,
	Hysteresis = 2,
	PI = 3,
}_control_type;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_SEND_PERIOD 500
#define UART_BUS_POINTER (&huart2)
#define PWM_MIN 0
#define PWM_MAX 999
#define KP_PI_GAIN 500 // guessing and checking :)
#define KI_PI_GAIN 500
#define SAMPLE_TIME 0.2 //s

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t msgStr[64]; /* String where to store the OLED line and/or the serial port output */

static volatile _control_type last_control_type = None;
static volatile _control_type control_type = Init;
static volatile float tempVal; // The static volatile modifiers were an attempt to solve the visibility problem in SWV Data Trace Timeline Graph. Didn't work. Area 51.
static volatile float tempLower;
static volatile float tempUpper;
static volatile float tempRef = 40.0;
static volatile float tempHysteresisWidth = 5.0;
static volatile uint32_t timestamp_OFF = 0;
static volatile uint32_t falling_time = 500;

uint32_t pwmDuty;

uint32_t uartSoftTimer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ControlFeedbackLoop(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void turnOFF_PWM(void)
{
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void turnOFF_Hist(void)
{
	HAL_GPIO_DeInit(GPIOB, HYSTERESIS_CONTROL_Pin);

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = HYSTERESIS_CONTROL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void ControlFeedbackLoop(void)
{
	tempVal = Mcp9808GetTemperature();
	switch(control_type)
	{
		case Init: //Both Stop
		{
			turnOFF_PWM();
			turnOFF_Hist();
			control_type = None;
			break;
		}
		case None: //Both Stop
		{
			if(last_control_type != None)
			{
				turnOFF_PWM();
				turnOFF_Hist();
			}
			break;
		}
		case Hysteresis: //PWM Stop
		{
			if(last_control_type != Hysteresis)
			{
				turnOFF_PWM();

				GPIO_InitTypeDef GPIO_InitStruct = {0};
				GPIO_InitStruct.Pin = HYSTERESIS_CONTROL_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
			}

			HAL_GPIO_WritePin(HYSTERESIS_CONTROL_GPIO_Port, HYSTERESIS_CONTROL_Pin,
						hysteresisCtrl(tempRef - tempVal, tempHysteresisWidth));

			break;
		}
		case PI: //Hysteresis stop
		{
			if(last_control_type != PI)
			{
				turnOFF_Hist();

				GPIO_InitTypeDef GPIO_InitStruct = {0};
			    GPIO_InitStruct.Pin = GPIO_PIN_8;
			    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			    GPIO_InitStruct.Pull = GPIO_NOPULL;
			    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
			    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			}

			pwmDuty = piCtrl(tempRef - tempVal, SAMPLE_TIME, KP_PI_GAIN, KI_PI_GAIN,
							PWM_MIN, PWM_MAX); // both controllers are active - you can switch between them on the breadboard
			HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, &pwmDuty, 1);


			break;
		}
		default:
		{
			turnOFF_PWM();
			turnOFF_Hist();
			break;
		}
	}
	last_control_type = control_type;
}

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

	Mcp9808SetResolution(3);
	HAL_TIM_Base_Start_IT(&htim15); // control loop interrupt
	uartSoftTimer = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if (HAL_GetTick() - uartSoftTimer > UART_SEND_PERIOD)
		{
			uartSoftTimer = HAL_GetTick();
			sprintf((char*) msgStr, "MCP9808 reading: %.3f °C\r\n", tempVal);
			HAL_UART_Transmit_DMA(UART_BUS_POINTER, msgStr,
					strlen((char*) msgStr));
		}
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM15) // 5 Hz
	{
		ControlFeedbackLoop();
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
