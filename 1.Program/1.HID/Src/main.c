/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_custom_hid_if.h"
#include "usbd_customhid.h"

#include "string.h"


#include "ssd1306.h"
#include "i2c-lcd.h"
//#include "stdio.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t V_Value;
uint8_t H_Value;
uint8_t Buttons_1 = 0;
uint8_t Buttons_2 = 0;

//extern USBD_HandleTypeDef  *hUsbDeviceFS;
uint8_t dataToSend[20] = {0x00, 0x14, 0x01, 0xEE, 0x0F, 0x00, 0x0F, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//                        butns|size|     |     | lt  | rt  | lx        | ly        | rx        | ry        | unused
//                                    b10 & b9  & b7  & b8  & dr  & dl  & dd  & du
//                                          b4  & b3  & b2  & b1  & b0  & nusd & xbox & b6  & b5
uint8_t dataReceived[8]; // not used yet
uint16_t adcFrom [3];
int16_t adcFromBuffer [3];

uint32_t ticksNextBlink = 0;
uint32_t ticksForBlink = 1000/60;

uint8_t buttonPressed = 0;

uint8_t EncoderState = 0;
uint8_t EncoderLastState = 0;
uint8_t Encoder_A = 0;
uint8_t Encoder_B = 0;

uint16_t WheelValue = 2048;

/*
uint8_t readA(){ return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3); }
uint8_t readB(){ return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4); }
uint8_t readX(){ return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5); }
uint8_t readY(){ return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6); }
uint8_t readMode(){ return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1); }
uint8_t readLB(){ return 0; }
uint8_t readRB(){ return 0; }
uint8_t readDU(){ return 0; }
uint8_t readDD(){ return 0; }
uint8_t readDL(){ return 0; }
uint8_t readDR(){ return 0; }
uint8_t readStart(){ return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0); }
uint8_t readBack(){ return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7); }
uint8_t readLS(){ return 0; }
uint8_t readRS(){ return 0; }
*/

void UpdateWheel()
{
	Encoder_A = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	Encoder_B = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) << 1;
	EncoderState = Encoder_A | Encoder_B;


		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
		if (EncoderLastState != EncoderState){
		    switch (EncoderState) {
		      case 0:
		        if (EncoderLastState == 2){
		        	WheelValue += 24;
		          //cw = 1;
		        }
		        else if(EncoderLastState == 1){
		        	WheelValue -= 24;
		          //cw = -1;
		        }
		        break;
		      case 1:
		        if (EncoderLastState == 0){
		        	WheelValue += 24;
		          //cw = 1;
		        }
		        else if(EncoderLastState == 3){
		        	WheelValue -= 24;
		          //cw = -1;
		        }
		        break;
		      case 2:
		        if (EncoderLastState == 3){
		        	WheelValue += 24;
		          //cw = 1;
		        }
		        else if(EncoderLastState == 0){
		        	WheelValue -= 24;
		          //cw = -1;
		        }
		        break;
		      case 3:
		        if (EncoderLastState == 1){
		        	WheelValue += 24;
		          //cw = 1;
		        }
		        else if(EncoderLastState == 2){
		        	WheelValue -= 24;
		          //cw = -1;
		        }
		        break;
		    }
		  }

		EncoderLastState = EncoderState;
}

void updateButtons()
{
	// btns |rs|, |ls|, |select|, |start|, |dr|, |dl|, |dd|, |du|
	dataToSend[2] = 0;
	//dataToSend[2] = Buttons_1;
	/*		// Chwilowo
	dataToSend[2] |= (readDU() & 1) << 0;
	dataToSend[2] |= (readDD() & 1) << 1;
	dataToSend[2] |= (readDL() & 1) << 2;
	dataToSend[2] |= (readDR() & 1) << 3;
	dataToSend[2] |= (readStart() & 1) << 4;
	dataToSend[2] |= (readBack()  & 1) << 5;
	dataToSend[2] |= (readLS() & 1) << 6;
	dataToSend[2] |= (readRS() & 1) << 7;
	*/
	// btns |y|, |x|, |b|, |a|, _, _, |rb|, |lb|

	dataToSend[3] = 0;
	//dataToSend[3] = Buttons_2;
	/*		// Chwilowo
	dataToSend[3] |= (readLB() & 1) << 0;
	dataToSend[3] |= (readRB() & 1) << 1;
	dataToSend[3] |= (0 & 1) << 2;
	dataToSend[3] |= (0 & 1) << 3;
	dataToSend[3] |= (readA()  & 1) << 4;
	dataToSend[3] |= (readB()  & 1) << 5;
	dataToSend[3] |= (readX()  & 1) << 6;
	dataToSend[3] |= (readY()  & 1) << 7;
	*/
	// z = -left-trigger zone _ 0 _ right-trigger zone
	/*
	adcFromBuffer[2] = (int16_t)adcFrom[2];
	adcFromBuffer[2] /= 8;
	adcFromBuffer[2] = adcFromBuffer[2] - 255;
	*/
	// left & right triggers
	dataToSend[4] = 0; //left
	dataToSend[5] = 0; //right
	/*
	if (adcFromBuffer[2] < 0)
		dataToSend[4] = (uint8_t)(256 - (adcFromBuffer[2] & 0xFF));
	else
		dataToSend[5] = adcFromBuffer[2] & 0xFF;
		*/
///////////////////////////////////////////////////////////////////////////// lx
	adcFromBuffer[1] = WheelValue;//(int16_t)adcFrom[0];
	adcFromBuffer[1] -= 2048;
	adcFromBuffer[1] *= 16;
	dataToSend[6] = adcFromBuffer[1] & 0xFF;
	dataToSend[7] = (adcFromBuffer[1] >> 8) & 0xFF;
	/*
	adcFromBuffer[1] = (int16_t)V_Value;
	adcFromBuffer[1] -= 110;
	//adcFromBuffer[1] -= 1;		// Aby nie przekroczyc zakresu
	adcFromBuffer[1] *= -256;
	dataToSend[6] = adcFromBuffer[1] & 0xFF;
	dataToSend[7] = (adcFromBuffer[1] >> 8) & 0xFF;
///////////////////////////////////////////////////////////////////////////// ly
	adcFromBuffer[0] = (int16_t)H_Value;
	adcFromBuffer[0] -= 110;
	//adcFromBuffer[1] -= 1;		// Aby nie przekroczyc zakresu
	adcFromBuffer[0] *= -256;
	dataToSend[8] = adcFromBuffer[0] & 0xFF;
	dataToSend[9] = (adcFromBuffer[0] >> 8) & 0xFF;
	*/
	//adcFromBuffer[0] = (int16_t)adcFrom[0];
	//adcFromBuffer[0] -= 2048;
	//adcFromBuffer[0] *= 16;
	adcFromBuffer[0] = 0;
	dataToSend[8] = adcFromBuffer[0] & 0xFF;
	dataToSend[9] = (adcFromBuffer[0] >> 8) & 0xFF;

/////////////////////////////////////////////////////////////////////////////
	// rx
	dataToSend[10] = 0x7F;
	dataToSend[11] = 0xFF;
	// ry
	dataToSend[12] = 0x7F;
	dataToSend[13] = 0xFF;
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

	//Encoder_A = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	//Encoder_B = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) << 1;
	//EncoderLastState = Encoder_A | Encoder_B;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcFrom, 3); // for analog reading start periph-to-mem dma for 3 pins. settings in CUBE
  //adcFrom[0] = 0;
  //adcFrom[1] = 0;
  // INICJALIZACJA EKRANU
  /*
	lcd_init ();
	uint8_t check = SSD1306_Init ();
	SSD1306_Fill (0);
	SSD1306_UpdateScreen(); //display
	SSD1306_GotoXY (10,10);
	SSD1306_Puts ("HELLO", &Font_11x18, 1);
	SSD1306_GotoXY (10, 30);
	SSD1306_Puts ("WORLD !!", &Font_11x18, 1);
	SSD1306_UpdateScreen(); //display
	//while((HAL_I2C_IsDeviceReady (&hi2c1, 0x4E, 10, 100)));
	//lcd_send_string ("HELLO WORLD !!");
	SSD1306_Fill (0);
	SSD1306_UpdateScreen(); //display

	HAL_Delay(100);
	*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //printf("Elo\n");
  //char buff[2];


  //HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, 1);

  while (1)
    {
	  /*
  	  char String[16];

	  int n;
	  uint8_t Start_Bit;

	  HAL_UART_Receive(&huart3, &Start_Bit, 2 ,1000);		// Blokujace bo nie ma pzrerwa�
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
	  if (Start_Bit == 1)
	  {
		  U3S('1');

		  HAL_UART_Receive(&huart3, &H_Value, 1 ,1000);		// Blokujace bo nie ma pzrerwa�
		  n=sprintf (String, "Y: %d    ", H_Value);
		  SSD1306_GotoXY (0,10);
		  SSD1306_Puts (String, &Font_7x10, 1);

		  HAL_UART_Receive(&huart3, &V_Value, 1 ,1000);		// Blokujace bo nie ma pzrerwa�
		  n=sprintf (String, "X: %d    ", V_Value);
		  SSD1306_GotoXY (0,20);
		  SSD1306_Puts (String, &Font_7x10, 1);

		  HAL_UART_Receive(&huart3, &Buttons_1, 1 ,1000);		// Blokujace bo nie ma pzrerwa�
		  n=sprintf (String, "B_1: %d    ", Buttons_1);
		  SSD1306_GotoXY (0,30);
		  SSD1306_Puts (String, &Font_7x10, 1);

		  HAL_UART_Receive(&huart3, &Buttons_2, 1 ,1000);		// Blokujace bo nie ma pzrerwa�
		  n=sprintf (String, "B_2: %d    ", Buttons_2);
		  SSD1306_GotoXY (0,40);
		  SSD1306_Puts (String, &Font_7x10, 1);
	  }
	  else
	  {
		  // Odebralismy przypadkiem syf jakis
		 SSD1306_GotoXY (0,10);
		 SSD1306_Puts ("Error_01", &Font_7x10, 1);
		 SSD1306_GotoXY (0,20);
		 n=sprintf (String, "Recived: %d    ", Start_Bit);
		 SSD1306_Puts (String, &Font_7x10, 1);
	  }
	  SSD1306_UpdateScreen(); //display
*/
	  UpdateWheel();

	  //if (HAL_GetTick() > ticksNextBlink)
	  {
		  ticksNextBlink = HAL_GetTick() + ticksForBlink;
		  updateButtons();

		  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, dataToSend, 20);

	  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|CSN_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ON_Pin|MOS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin CSN_Pin CE_Pin */
  GPIO_InitStruct.Pin = LED_Pin|CSN_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_BT_Pin RIGHT_BT_Pin SELECT_Pin UP_Pin 
                           RIGHT_Pin DOWN_Pin BUTTON_F_Pin VIN_Pin 
                           POW_BT_Pin */
  GPIO_InitStruct.Pin = LEFT_BT_Pin|RIGHT_BT_Pin|SELECT_Pin|UP_Pin 
                          |RIGHT_Pin|DOWN_Pin|BUTTON_F_Pin|VIN_Pin 
                          |POW_BT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_Pin BUTTON_E_Pin */
  GPIO_InitStruct.Pin = LEFT_Pin|BUTTON_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ON_Pin MOS_Pin */
  GPIO_InitStruct.Pin = ON_Pin|MOS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
