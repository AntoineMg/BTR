/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum{L_SENSOR, R_SENSOR}TNumSensor;
typedef enum{L_MOTOR, R_MOTOR}TNumMotor;
typedef enum{NEUTRAL, FORWARD, BACKWARD}TDirection;



/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void TrigCapteurUs1(void);
void Sensor_Read(TNumSensor);
void Motors_Move(int, int);
void Motors_SetDirection(TDirection);
void Motors_Stop(void);
void Motors_SetSpeed(TNumMotor, uint8_t);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI4_15_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define CapteurUs1Echo_Pin GPIO_PIN_5
#define CapteurUs1Echo_GPIO_Port GPIOA
#define CapteurUs1Echo_EXTI_IRQn EXTI4_15_IRQn
#define CapteurUs1Trig_Pin GPIO_PIN_6
#define CapteurUs1Trig_GPIO_Port GPIOA
#define Mot1_Enable_Pin GPIO_PIN_0
#define Mot1_Enable_GPIO_Port GPIOB
#define Mot2_Enable_Pin GPIO_PIN_1
#define Mot2_Enable_GPIO_Port GPIOB
#define CapteurUs2Trig_Pin GPIO_PIN_12
#define CapteurUs2Trig_GPIO_Port GPIOB
#define CapteurUs2Echo_Pin GPIO_PIN_13
#define CapteurUs2Echo_GPIO_Port GPIOB
#define Mot1_Ctrl1_Pin GPIO_PIN_6
#define Mot1_Ctrl1_GPIO_Port GPIOC
#define Mot1_Ctrl2_Pin GPIO_PIN_7
#define Mot1_Ctrl2_GPIO_Port GPIOC
#define Mot2_Ctrl1_Pin GPIO_PIN_8
#define Mot2_Ctrl1_GPIO_Port GPIOC
#define Mot2_Ctrl2_Pin GPIO_PIN_9
#define Mot2_Ctrl2_GPIO_Port GPIOC
#define cd2_Pin GPIO_PIN_11
#define cd2_GPIO_Port GPIOA
#define cd2_EXTI_IRQn EXTI4_15_IRQn
#define cd3_Pin GPIO_PIN_12
#define cd3_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
