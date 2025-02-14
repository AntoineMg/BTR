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
#include "i2c.h"
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

//CONDITIONNELS
//#define TESTMOTORS
//#define TESTUS
#define MMA8451Q


#define MAX_SPEED 1


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//Capteurs US

//Distance capteur US1
uint8_t g_int_distCapteurUs1=0;
uint8_t g_int_distRetenueUs1=0;

int32_t g_int_mot1PositionActuelle;
int32_t g_int_mot1PositionPrecedente;

uint8_t g_int_rpm;

char rx_buffer[10];  // Tampon pour stocker les caractères reçus
uint8_t rx_index = 0;

uint8_t g_int_LSpeed=0;
uint8_t g_int_RSpeed=0;

int l_bool_commande = 0;

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
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);


  Motors_SetDirection(NEUTRAL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
#ifdef TESTMOTORS


	  HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_buffer, 1);

	  if(l_bool_commande == 1){
		  l_bool_commande = 0;

		  Motors_SetDirection(BACKWARD);

		  //Si le régime demndé est trop faible, le moteur démarre avant a haute vitesse
		  if(g_int_LSpeed>40 && g_int_LSpeed<150 && g_int_rpm==0){
			  Motors_SetSpeed(L_MOTOR, 220);
			  HAL_Delay(500);
		  }
		  else{
			  HAL_Delay(500);
		  }

		  //Actualise la vitesse du moteur
		  Motors_SetSpeed(L_MOTOR, g_int_LSpeed);

		  //Attente moteur
		  HAL_Delay(1500);
		  Print_Speed();
		  Motors_Stop();
		  //HAL_Delay(1000);

	  }



#endif

#ifdef TESTUS
	  TrigCapteurUs1();

	  HAL_Delay(500);

	  //filtrage
	  if((g_int_distCapteurUs1 <= 220) && (g_int_distCapteurUs1 >= 2)){
		  g_int_distRetenueUs1 = g_int_distCapteurUs1;
	  }

	  //uint8_t buffer[15];
	  //snprintf((char *)buffer, sizeof(buffer), "%d", g_int_distCapteurUs1);

	  //uint8_t buffer[15] = g_int_distCapteurUs1;

	  HAL_UART_Transmit(&huart2, &g_int_distRetenueUs1, 1, HAL_MAX_DELAY);
	  //printf("\n Erreur de lecture %i \n", g_int_distCapteurUs1);


	  //printf("marche stp \n");

	  //HAL_Delay(1000);
#endif
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
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

//a refaire completement bancal
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
        if (HAL_GPIO_ReadPin(CapteurUs1Echo_GPIO_Port, CapteurUs1Echo_Pin) == GPIO_PIN_SET){
        	//Lancer le timer
        	HAL_TIM_Base_Start(&htim6);
        }

        //Verif falling
        else if (HAL_GPIO_ReadPin(CapteurUs1Echo_GPIO_Port, CapteurUs1Echo_Pin) == GPIO_PIN_RESET){
        	//Arrete le timer et transmets la valeur
        	//ici la valeur est directement en cm grace au prescaler de tim2 qui est a 941
        	HAL_TIM_Base_Stop(&htim6);
        	g_int_distCapteurUs1 = TIM6->CNT;
        	TIM6->CNT = 0;

        }
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  // Vérifie quel uart a déclenché l'interruption
      if (huart == &huart2) {

		  char received_char = rx_buffer[0];  // Récupérer le caractère reçu
		  //traitement caractere
		  g_int_LSpeed = received_char;
		  l_bool_commande = 1;


		  if (rx_index < 10)
		  {
			  rx_buffer[rx_index++] = received_char;  // Ajouter à la position actuelle dans le tampon
		  }

		  // Relancer la réception pour récupérer un autre caractère
		  //HAL_UART_Receive_IT(&huart1, (uint8_t *)rx_buffer, 1);  // Recevoir un autre caractère
      }

}

//mesures

void Sensor_Read(TNumSensor x_numSensor){
	//Envoi pwm trig
	TrigCapteurUs1();

	//filtrage
	if((g_int_distCapteurUs1 >= 220) || (g_int_distCapteurUs1 <= 2)){
		//rien
	}
	else {
		g_int_distRetenueUs1 = g_int_distCapteurUs1;
	}


}


void NavigationUpdate(void){

}

void Motors_Move(int x_int_angle, int x_int_speed){
	// Limiter l'angle entre -90 et 90 degrés
		int l_int_Lspeed;
		int l_int_Rspeed;

	    if (x_int_angle > 90){
	    	x_int_angle = 90;
	    }

	    else if (x_int_angle < -90){
	    	x_int_angle = -90;
	    }

	    // Calcul des vitesses
	    l_int_Lspeed = x_int_speed * (1 - (float)x_int_angle / 90.0f);
	    l_int_Rspeed = x_int_speed * (1 + (float)x_int_angle / 90.0f);

	    //Affectation des vitesses aux moteurs
	    Motors_SetSpeed(L_MOTOR, l_int_Lspeed);
	    Motors_SetSpeed(R_MOTOR, l_int_Rspeed);

}

void Motors_SetDirection(TDirection x_direction){
	//Gerer les ports CTRL1 et CTRL2 des moteurs
	//Cest par ici qu'on met la securite du delai de changement de dircetion avec
	if(x_direction == NEUTRAL){
		HAL_GPIO_WritePin(GPIOC, Mot1_Ctrl1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Mot1_Ctrl2_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOC, Mot2_Ctrl1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Mot2_Ctrl2_Pin, GPIO_PIN_RESET);
	}

	else if(x_direction == FORWARD){
		HAL_GPIO_WritePin(GPIOC, Mot1_Ctrl1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Mot1_Ctrl2_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOC, Mot2_Ctrl1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, Mot2_Ctrl2_Pin, GPIO_PIN_RESET);

	}
	else if(x_direction == BACKWARD){
		HAL_GPIO_WritePin(GPIOC, Mot1_Ctrl1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, Mot1_Ctrl2_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOC, Mot2_Ctrl1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Mot2_Ctrl2_Pin, GPIO_PIN_SET);

	}
	else{
		//Mode parking => tout a 0 comme le neutre, mis 2 fois pour avoir une securite de 100%
		HAL_GPIO_WritePin(GPIOC, Mot1_Ctrl1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Mot1_Ctrl2_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOC, Mot2_Ctrl1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Mot2_Ctrl2_Pin, GPIO_PIN_RESET);
	}
}

void Motors_Stop(void){
	//Arrete tout les moteurs
	Motors_SetSpeed(L_MOTOR,0);
	Motors_SetSpeed(R_MOTOR,0);
}

void Motors_SetSpeed(TNumMotor x_numMotor, uint8_t x_int_speed){
	//recoit une vitesse entre 0 et 255, puis la transforme en pourcent par rapport au max
	//x_int_speed = (x_int_speed * TIM3->ARR)/ 255;
	x_int_speed = (x_int_speed+138)/ 3;



	//Controle chaque moteur individuellement
	if(x_numMotor==L_MOTOR){
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, x_int_speed);
		}
	else if(x_numMotor==R_MOTOR){
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, x_int_speed);
		}
}

void Encoders_GetData(void){
	int32_t l_int_distanceParcourue = 0;

	// Lire la position actuelle
	int32_t positionActuelle = __HAL_TIM_GET_COUNTER(&htim2);


	// Calculer la distance parcourue depuis la dernière lecture
	l_int_distanceParcourue = positionActuelle;


	g_int_rpm = 4*(l_int_distanceParcourue*30)/224;

	// Réinitialiser le compteur
	__HAL_TIM_SET_COUNTER(&htim2, 0);

}

void Print_Speed(void){
	// Afficher la distance parcourue (ou vitesse)
	HAL_UART_Transmit(&huart2, &g_int_rpm, 1, HAL_MAX_DELAY);
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
