/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Serial_Recived(uint8_t* Buf, uint32_t *Len);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define CSN_Pin GPIO_PIN_14
#define CSN_GPIO_Port GPIOC
#define CE_Pin GPIO_PIN_15
#define CE_GPIO_Port GPIOC
#define HORISONTAL_Pin GPIO_PIN_0
#define HORISONTAL_GPIO_Port GPIOA
#define VERTICAL_Pin GPIO_PIN_1
#define VERTICAL_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_2
#define BT_TX_GPIO_Port GPIOA
#define BT_RX_Pin GPIO_PIN_3
#define BT_RX_GPIO_Port GPIOA
#define BAT_Pin GPIO_PIN_4
#define BAT_GPIO_Port GPIOA
#define BUZZ_Pin GPIO_PIN_0
#define BUZZ_GPIO_Port GPIOB
#define BT_LD_Pin GPIO_PIN_1
#define BT_LD_GPIO_Port GPIOB
#define BT_LA_Pin GPIO_PIN_2
#define BT_LA_GPIO_Port GPIOB
#define SCL_IMU_Pin GPIO_PIN_10
#define SCL_IMU_GPIO_Port GPIOB
#define SDA_IMU_Pin GPIO_PIN_11
#define SDA_IMU_GPIO_Port GPIOB
#define BT_RS_Pin GPIO_PIN_12
#define BT_RS_GPIO_Port GPIOB
#define BT_RA_Pin GPIO_PIN_13
#define BT_RA_GPIO_Port GPIOB
#define BT_RB_Pin GPIO_PIN_14
#define BT_RB_GPIO_Port GPIOB
#define BT_RC_Pin GPIO_PIN_15
#define BT_RC_GPIO_Port GPIOB
#define BT_RD_Pin GPIO_PIN_8
#define BT_RD_GPIO_Port GPIOA
#define BT_LC_Pin GPIO_PIN_15
#define BT_LC_GPIO_Port GPIOA
#define BT_LB_Pin GPIO_PIN_3
#define BT_LB_GPIO_Port GPIOB
#define VIN_Pin GPIO_PIN_4
#define VIN_GPIO_Port GPIOB
#define ON_Pin GPIO_PIN_5
#define ON_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define BT_POWER_Pin GPIO_PIN_8
#define BT_POWER_GPIO_Port GPIOB
#define MOS_Pin GPIO_PIN_9
#define MOS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
