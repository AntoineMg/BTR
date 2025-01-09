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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

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

/* USER CODE BEGIN PV */


//Capteurs US

//Distance capteur US1
int g_int_distCapteurUs1=0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  GenerePWM(50);
  while (1)
  {


	  //Mesure distance
	  //TrigCapteurUs1();
	  //HAL_Delay(500);

	  //printf("%i \n", g_int_distCapteurUs1);
	  //printf("marche stp \n");

	  //HAL_Delay(1000);


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//FONCTIONS

int __io_putchar(int ch){
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
	return ch;
}

void TrigCapteurUs1(void){
	//Sortie ETat Haut
	HAL_GPIO_WritePin(CapteurUs1Trig_GPIO_Port, CapteurUs1Trig_Pin, GPIO_PIN_SET);
	//10us
	HAL_Delay(0.01);
	//Sortie Etat Bas
	HAL_GPIO_WritePin(CapteurUs1Trig_GPIO_Port, CapteurUs1Trig_Pin, GPIO_PIN_RESET);
}

//Callback lors d'interruptions sur EXTI 4:15
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Vérifie quel pin a déclenché l'interruption
    if (GPIO_Pin == CapteurUs1Echo_Pin) {

    	//Verif rising
        if (HAL_GPIO_ReadPin(CapteurUs1Trig_GPIO_Port, CapteurUs1Echo_Pin) == GPIO_PIN_SET){
        	//Lancer le timer
        	HAL_TIM_Base_Start(&htim2);
        }

        //Verif falling
        else if (HAL_GPIO_ReadPin(CapteurUs1Trig_GPIO_Port, CapteurUs1Echo_Pin) == GPIO_PIN_RESET){
        	//Arrete le timer et transmets la valeur
        	HAL_TIM_Base_Stop(&htim2);
        	g_int_distCapteurUs1 = TIM2->CNT;
        	TIM2->CNT = 0;

        }
    }
}

//Genere le signal PWM a 25kHz pour le moteur via TIM3
void GenerePWM(int x_int_alpha){
	//Conversion de alpha en temps d'etat haut
	int l_int_tempsHaut = (x_int_alpha/100)*40;	//le 40 est en us car periode du timer (tick) est ici de 1us
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, l_int_tempsHaut);
}

void Motors_Move(int x_int_angle, int x_int_speed){
	// Limiter l'angle entre -90 et 90 degrés
	    if (x_int_angle > 90){
	    	x_int_angle = 90;
	    }

	    if (x_int_angle < -90){
	    	x_int_angle = -90;
	    }

	    // Calcul des vitesses
	    l_int_Lspeed = MAX_SPEED * (1 - (float)x_int_angle / 90.0f);
	    l_int_Rspeed = MAX_SPEED * (1 + (float)x_int_angle / 90.0f);
}

void Motors_SetDirection(TDirection x_direction){
	//Gerer les ports CTRL1 et CTRL2 des moteurs
	//Cest par ici qu'on met la securite du delai de changement de dircetion avec
	if(x_direcion == FORWARD){

	}
	else if(x_direction == BACKWARD){

	}
	else{
		//Mode parking => tout a 0
	}
}

void Motors_Stop(void){
	//Arrete tout les moteurs
	Motors_SetSpeed(MOT1,0);
	Motors_SetSpeed(MOT1,0);
}

void Motors_SetSpeed(TNumMotor x_numMotor, uint8_t x_int_speed){
	//Controle chaque moteur individuellement
	if(x_numMotor==L_MOTOR){
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x_speed);
		}
	else if(x_numMotor==R_MOTOR){
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, x_speed);
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
