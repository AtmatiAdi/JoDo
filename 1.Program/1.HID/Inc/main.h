/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

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
#define LEFT_BT_Pin GPIO_PIN_1
#define LEFT_BT_GPIO_Port GPIOB
#define RIGHT_BT_Pin GPIO_PIN_2
#define RIGHT_BT_GPIO_Port GPIOB
#define SCL_IMU_Pin GPIO_PIN_10
#define SCL_IMU_GPIO_Port GPIOB
#define SDA_IMU_Pin GPIO_PIN_11
#define SDA_IMU_GPIO_Port GPIOB
#define SELECT_Pin GPIO_PIN_12
#define SELECT_GPIO_Port GPIOB
#define UP_Pin GPIO_PIN_13
#define UP_GPIO_Port GPIOB
#define RIGHT_Pin GPIO_PIN_14
#define RIGHT_GPIO_Port GPIOB
#define DOWN_Pin GPIO_PIN_15
#define DOWN_GPIO_Port GPIOB
#define LEFT_Pin GPIO_PIN_8
#define LEFT_GPIO_Port GPIOA
#define BUTTON_E_Pin GPIO_PIN_15
#define BUTTON_E_GPIO_Port GPIOA
#define BUTTON_F_Pin GPIO_PIN_3
#define BUTTON_F_GPIO_Port GPIOB
#define VIN_Pin GPIO_PIN_4
#define VIN_GPIO_Port GPIOB
#define ON_Pin GPIO_PIN_5
#define ON_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define POW_BT_Pin GPIO_PIN_8
#define POW_BT_GPIO_Port GPIOB
#define MOS_Pin GPIO_PIN_9
#define MOS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//ser
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
