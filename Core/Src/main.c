/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "touchsensing.h"
#include "ts.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST_TKEY(NB) ((MyTKeys[(NB)].p_Data->StateId == TSL_STATEID_DETECT) || \
		    (MyTKeys[(NB)].p_Data->StateId == TSL_STATEID_DEB_RELEASE_DETECT))
 // PB7 = LED_RED  WiMOD:NC, eMOD:Available
 #define LED_RED_TOGGLE {GPIOB->ODR ^=  (1<<7);}
 #define LED_RED_OFF    {GPIOB->ODR &= ~(1<<7);}
 #define LED_RED_ON     {GPIOB->ODR |=  (1<<7);}
 // PA15 = LED_BLUE
 #define LED_BLUE_TOGGLE  {GPIOA->ODR ^=  (1<<15);}
 #define LED_BLUE_OFF     {GPIOA->ODR &= ~(1<<15);}
 #define LED_BLUE_ON      {GPIOA->ODR |=  (1<<15);}
 // PA8 = LED_GREEN  for TS
#define LED_GREEN_TOGGLE  {GPIOA->ODR ^=  (1<<8);}
#define LED_GREEN_OFF     {GPIOA->ODR &= ~(1<<8);}
#define LED_GREEN_ON      {GPIOA->ODR |=  (1<<8);}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// For debug purpose with STMStudio
TSL_StateId_enum_T DS[TSLPRM_TOTAL_TKEYS]; // To store the States (one per object)
TSL_tDelta_T DD[TSLPRM_TOTAL_TKEYS]; // To store the Deltas (one per channel)
TSL_tMeas_T DM[TSLPRM_TOTAL_TKEYS];// To store the Measures (one per object)
TSL_tMeas_T DR[TSLPRM_TOTAL_TKEYS];// To store the references (one per object)

tsl_user_status_t status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ProcessSensors(void);

void LED_BLINK(Color color, int round) {
	__IO uint32_t led;
	uint32_t pin;
switch (color) {
	case RED:
		led = GPIOB->ODR;
		pin = GPIO_PIN_7;
		break;
	case BLUE:
		led = GPIOA->ODR;
		pin = GPIO_PIN_15;
		break;
	case GREEN:
		led = GPIOA->ODR;
		pin = GPIO_PIN_8;
		break;
	default:
		return;
	}
	led |=  (1<<pin);
	for (int l = 0; l < round; ++l) {
		led ^=  (1<<pin);
		for (int wait = 0; wait < 1000; ++wait) {
			__NOP();
		}
	}
	led &= ~(1<<pin);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ProcessSensors(void)
{
	uint32_t idx = 0;
	uint32_t idx_ds = 0;
   uint32_t idx_dd = 0;
   uint32_t idx_dm = 0;
   uint32_t idx_dr = 0;

   // STMStudio debug
   for (idx = 0; idx < TSLPRM_TOTAL_TKEYS; idx++)
   {
     DS[idx_ds++] = MyTKeys[idx].p_Data->StateId;
     DD[idx_dd++] = MyTKeys[idx].p_ChD->Delta;
     DM[idx_dm++] = MyTKeys[idx].p_ChD->Meas;
     DR[idx_dr++] = MyTKeys[idx].p_ChD->Ref;
   }

	// TKEY 0
	if (TEST_TKEY(0)) {
		LED_BLINK(GREEN, 4);
	} else {
		LED_GREEN_OFF
		;
	}

	// TKEY 1
	if (TEST_TKEY(1)) {
		LED_BLINK(GREEN, 4);
	} else {
		LED_GREEN_OFF
		;
	}

	if (TEST_TKEY(2)) {
		LED_BLINK(GREEN, 4);
	} else {
		LED_GREEN_OFF
		;
	}

	if (TEST_TKEY(3)) {
		LED_BLINK(GREEN, 4);
	} else {
		LED_GREEN_OFF
		;
	}

	if (TEST_TKEY(4)) {
		LED_BLINK(GREEN, 4);
	} else {
		LED_GREEN_OFF
		;
	}
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
  MX_TS_Init();
  MX_TOUCHSENSING_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 status =  tsl_user_Exec();
	 switch (status) {
		case TSL_USER_STATUS_OK_ECS_ON:
			LED_BLUE_TOGGLE;
			break;
		case TSL_USER_STATUS_OK_ECS_OFF:
		case TSL_USER_STATUS_OK_NO_ECS:
		case TSL_USER_STATUS_BUSY:
		default:
			LED_BLUE_OFF;
			break;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 ProcessSensors();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
}

/* USER CODE BEGIN 4 */

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
