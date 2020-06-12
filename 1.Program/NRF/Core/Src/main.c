/* USER CODE BEGIN Header */
/*

░░░░░██╗░█████╗░██╗░░░██╗░██████╗████████╗██╗░█████╗░██╗░░██╗
░░░░░██║██╔══██╗╚██╗░██╔╝██╔════╝╚══██╔══╝██║██╔══██╗██║░██╔╝
░░░░░██║██║░░██║░╚████╔╝░╚█████╗░░░░██║░░░██║██║░░╚═╝█████═╝░
██╗░░██║██║░░██║░░╚██╔╝░░░╚═══██╗░░░██║░░░██║██║░░██╗██╔═██╗░
╚█████╔╝╚█████╔╝░░░██║░░░██████╔╝░░░██║░░░██║╚█████╔╝██║░╚██╗
░╚════╝░░╚════╝░░░░╚═╝░░░╚═════╝░░░░╚═╝░░░╚═╝░╚════╝░╚═╝░░╚═╝

░░░░░██╗░░░░█████╗░░░░██████╗░░░░░█████╗░  ██╗░░░██╗░░███╗░░
░░░░░██║░░░██╔══██╗░░░██╔══██╗░░░██╔══██╗  ██║░░░██║░████║░░
░░░░░██║░░░██║░░██║░░░██║░░██║░░░██║░░██║  ╚██╗░██╔╝██╔██║░░
██╗░░██║░░░██║░░██║░░░██║░░██║░░░██║░░██║  ░╚████╔╝░╚═╝██║░░
╚█████╔╝██╗╚█████╔╝██╗██████╔╝██╗╚█████╔╝  ░░╚██╔╝░░███████╗
░╚════╝░╚═╝░╚════╝░╚═╝╚═════╝░╚═╝░╚════╝░  ░░░╚═╝░░░╚══════╝
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "my_usbd_customhid.h"
#include "my_usbd_custom_hid_if.h"
#include "my_icons.h"
#include "my_bitmaps.h"
#include "ssd1306.h"
#include "i2c-lcd.h"
#include "string.h"
#include "MY_NRF24.h"
#include "TJ_MPU6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FUNC_ACCEL_GYRO_DATA	128
#define FUNC_JOYSTICK_DATA		129
#define FUNC_ACCEL_GYRO_COMBO	130

#define ADS1115_ADDRESS 0x48

char Program_name[] = {
	'G','a','m','e','p','a','d',
	'S','M','i','c','r','o','m','o','u','s','e',
	'T','e','s','t','1',
	'T','e','s','t','2',
	'T','e','s','t','3',
	'T','e','s','t','4'};
int Program_length[] = {
	7,
	11,
	5,
	5,
	5,
	5,};
int Program_Count = 6;
int Program_Running;

char ADSwrite[6];

char Msg[16];
char Flag[8] = {0,0,0,0,0,0,0,0};
int IsHID = 0;
RawData_Def AccelData, GyroData;

uint64_t PipeAddres = 0x11223344AA;
char RF_RxData[32], RF_TxData[32], BadFunc;

uint8_t dataToSend[20] = {0x00, 0x14, 0x01, 0xEE, 0x0F, 0x00, 0x0F, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//                        butns|size|     |     | lt  | rt  | lx        | ly        | rx        | ry        | unused
//                                    b10 & b9  & b7  & b8  & dr  & dl  & dd  & du
//

int Serial_Send(uint8_t* Buf, uint32_t *Len){
	CDC_Transmit_FS(Buf, Len);
	return &Len;
}

#define I2C1_SCL_Pin SCL_Pin
#define I2C1_SCL_GPIO_Port SCL_GPIO_Port
#define I2C1_SDA_Pin SDA_Pin
#define I2C1_SDA_GPIO_Port SDA_GPIO_Port

static bool wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint32_t timeout)
 {
    uint32_t Tickstart = HAL_GetTick();
    bool ret = true;
    /* Wait until flag is set */
    for(;(state != HAL_GPIO_ReadPin(port, pin)) && (true == ret);)
    {
        /* Check for the timeout */
        if (timeout != HAL_MAX_DELAY)
        {
            if ((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout))
            {
                ret = false;
            }
            else
            {
            }
        }
        asm("nop");
    }
    return ret;
}


static void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef* handle, uint32_t timeout)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. Clear PE bit.
    CLEAR_BIT(handle->Instance->CR1, I2C_CR1_PE);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_I2C_DeInit(handle);

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin = I2C1_SCL_Pin;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = I2C1_SDA_Pin;
    HAL_GPIO_Init(I2C1_SDA_GPIO_Port, &GPIO_InitStructure);

    // 3. Check SCL and SDA High level in GPIOx_IDR.
    HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET);

    wait_for_gpio_state_timeout(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET, timeout);
    wait_for_gpio_state_timeout(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET, timeout);

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_RESET);

    // 5. Check SDA Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_RESET, timeout);

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_RESET);

    // 7. Check SCL Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_RESET, timeout);

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET, timeout);

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET, timeout);

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    //MAT//GPIO_InitStructure.Alternate = GPIO_AF1_I2C1;

    GPIO_InitStructure.Pin = I2C1_SCL_Pin;
    HAL_GPIO_Init(I2C1_SCL_GPIO_Port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = I2C1_SDA_Pin;
    HAL_GPIO_Init(I2C1_SDA_GPIO_Port, &GPIO_InitStructure);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    SET_BIT(handle->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 14. Clear SWRST bit in I2Cx_CR1 register. */
    CLEAR_BIT(handle->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
    SET_BIT(handle->Instance->CR1, I2C_CR1_PE);
    asm("nop");

    // Call initialization function.
    HAL_I2C_Init(handle);
}
/*
static void I2C_ClearBusyFlagErratum2(I2C_HandleTypeDef* handle, uint32_t timeout)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. Clear PE bit.
    CLEAR_BIT(handle->Instance->CR1, I2C_CR1_PE);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_I2C_DeInit(handle);

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin = I2C2_SCL_Pin;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = I2C2_SDA_Pin;
    HAL_GPIO_Init(I2C2_SDA_GPIO_Port, &GPIO_InitStructure);

    // 3. Check SCL and SDA High level in GPIOx_IDR.
    HAL_GPIO_WritePin(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_SET);

    wait_for_gpio_state_timeout(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_SET, timeout);
    wait_for_gpio_state_timeout(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_SET, timeout);

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_RESET);

    // 5. Check SDA Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_RESET, timeout);

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_RESET);

    // 7. Check SCL Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_RESET, timeout);

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_SET, timeout);

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_SET, timeout);

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    //MAT//GPIO_InitStructure.Alternate = GPIO_AF1_I2C1;

    GPIO_InitStructure.Pin = I2C2_SCL_Pin;
    HAL_GPIO_Init(I2C2_SCL_GPIO_Port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = I2C1_SDA_Pin;
    HAL_GPIO_Init(I2C2_SDA_GPIO_Port, &GPIO_InitStructure);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    SET_BIT(handle->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    // 14. Clear SWRST bit in I2Cx_CR1 register.
    CLEAR_BIT(handle->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
    SET_BIT(handle->Instance->CR1, I2C_CR1_PE);
    asm("nop");

    // Call initialization function.
    HAL_I2C_Init(handle);
}
*/
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double MapValue(double Val, double FromLow,double FromHigh,double ToLow,double ToHigh){
	double out = ToLow + ((ToHigh - ToLow) / (FromHigh - FromLow)) * (Val - FromLow);
	if (out > ToHigh) out = ToHigh;
	if (out < ToLow) out - ToLow;
	return out;
}

void AdsRead(int16_t *buf){
	for (int a = 0; a < 4; a++){
		  ADSwrite[0] = 0x01;
		  switch(a){
		  case 0: {
			  ADSwrite[1] = 0xC1; // 11000011
			  break;
		  }
		  case 1: {
			  ADSwrite[1] = 0xD1; // 11010011
			  break;
		  }
		  case 2: {
			  ADSwrite[1] = 0xE1; // 11100011
			  break;
		  }
		  case 3: {
			  ADSwrite[1] = 0xF1; // 11110011
			  break;
		  }
		  }

		  ADSwrite[2] = 0xE3; // 10000011 // 10100011 // 11000011// 11100011
		  HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDRESS<<1, ADSwrite, 3, 100);
		  ADSwrite[0] = 0x00;
		  HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDRESS<<1, ADSwrite, 1, 100);
		  NRF24_DelayMicroSeconds(100);
		  HAL_I2C_Master_Receive(&hi2c2, ADS1115_ADDRESS<<1, ADSwrite, 2, 100);
		  buf[a] = (((int16_t)ADSwrite[0]) << 8 | ADSwrite[1]);
	  }
}

void AdcRead(int16_t *buf){
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
		buf[0] = HAL_ADC_GetValue(&hadc1);
	}
	if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
		buf[1] = HAL_ADC_GetValue(&hadc1);
	}
	if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
		buf[2] = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);
}

void updateButtons()
{
	  int16_t Val[4];
	  AdsRead(Val);
	  Val[1] = (int16_t)MapValue(Val[1], 0, Val[0], 0, 10);
	  Val[2] = (int16_t)MapValue(Val[2], 0, Val[0], -32768, 32767);
	  Val[3] = (int16_t)MapValue(Val[3], 0, Val[0], -32768, 32767);
	  if ((Val[2] <= 3) && (Val[2] >= -3)) Val[2] = 0;
	  if ((Val[3] <= 3) && (Val[3] >= -3)) Val[3] = 0;
	  if (Val[1] > 5) Val[1] = 0; else Val[1] = 1;

	 int16_t Adc[3];
	 AdcRead(Adc);
	 Adc[0] = (int16_t)MapValue(Adc[0], 0, 4095, -32768, 32767);
	 Adc[1] = (int16_t)MapValue(Adc[1], 0, 4095, -32768, 32767);

	  ////////////////////////////////////////////////////////////////////////////////////////////////////
	// btns |rs|, |ls|, |select|, |start|, |dr|, |dl|, |dd|, |du|
	dataToSend[2] = 0;
	dataToSend[2] |= Val[1] << 0;
	dataToSend[2] |= !HAL_GPIO_ReadPin(BT_LC_GPIO_Port, BT_LC_Pin) << 1;
	dataToSend[2] |= !HAL_GPIO_ReadPin(BT_LB_GPIO_Port, BT_LB_Pin) << 2;
	dataToSend[2] |= !HAL_GPIO_ReadPin(BT_LD_GPIO_Port, BT_LD_Pin) << 3;
	//dataToSend[2] |= (readStart() & 1) << 4;
	//dataToSend[2] |= (readBack()  & 1) << 5;
	//dataToSend[2] |= (Val[1]  & 1) << 6;
	//dataToSend[2] |= (readRS() & 1) << 7;
	// btns |y|, |x|, |b|, |a|, _, _, |rb|, |lb|
	dataToSend[3] = 0;
	dataToSend[3] |= !HAL_GPIO_ReadPin(BT_LA_GPIO_Port, BT_LA_Pin)  & 1 << 0;
	dataToSend[3] |= !HAL_GPIO_ReadPin(BT_RA_GPIO_Port, BT_RA_Pin) << 1;
	//dataToSend[3] |= (Val[1] & 1) << 2;
	//dataToSend[3] |= (Val[1] & 1) << 3;
	dataToSend[3] |= !HAL_GPIO_ReadPin(BT_RC_GPIO_Port, BT_RC_Pin) << 4;
	dataToSend[3] |= !HAL_GPIO_ReadPin(BT_RB_GPIO_Port, BT_RB_Pin) << 5;
	dataToSend[3] |= !HAL_GPIO_ReadPin(BT_RD_GPIO_Port, BT_RD_Pin) << 6;
	dataToSend[3] |= !HAL_GPIO_ReadPin(BT_RS_GPIO_Port, BT_RS_Pin) << 7;
	// left & right triggers
	dataToSend[4] = 0; //left
	dataToSend[5] = 0; //right
	//lx
	dataToSend[6] = Val[3] & 0xFF;
	dataToSend[7] = (Val[3] >> 8) & 0xFF;
	// ly
	dataToSend[8] = Val[2] & 0xFF;
	dataToSend[9] = (Val[2] >> 8) & 0xFF;
	// rx
	dataToSend[10] = Adc[1] & 0xFF;
	dataToSend[11] = (Adc[1] >> 8) & 0xFF;
	// ry
	dataToSend[12] = Adc[0] & 0xFF;
	dataToSend[13] = (Adc[0] >> 8) & 0xFF;
}

void UlToStr(char *s, unsigned long bin, unsigned char n)
{
    s += n;
    *s = '\0';

    while (n--)
    {
        *--s = (bin % 10) + '0';
        bin /= 10;
    }
}

void Serial_Recived(uint8_t* Buf, uint32_t *Len){
	switch(Buf[0]){
	case '0': {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
		Msg[0] = 'O';
		Msg[1] = 'N';
		Serial_Send(Msg, 2);
		break;
	}
	case '1': {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		Msg[0] = 'O';
		Msg[1] = 'F';
		Serial_Send(Msg, 2);
		break;
	}
	case '2': {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		Msg[0] = FUNC_ACCEL_GYRO_DATA;
		Serial_Send(Msg, 1);
		break;
	}
	case '3': {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		int16_t Ax = 1024;
		int16_t Ay = -1024;
		int16_t Az = 32767;
		int16_t Gx = -32767;
		int16_t Gy = 255;
		int16_t Gz = -255;
		Msg[0] = FUNC_ACCEL_GYRO_DATA;
		Msg[1] = (char)Ax;
		Msg[2] = (char)(Ax >> 8);
		Msg[3] = (char)Ay;
		Msg[4] = (char)(Ay >> 8);
		Msg[5] = (char)Az;
		Msg[6] = (char)(Az >> 8);
		Msg[7] = (char)Gx;
		Msg[8] = (char)(Gx >> 8);
		Msg[9] = (char)Gy;
		Msg[10] = (char)(Gy >> 8);
		Msg[11] = (char)Gz;
		Msg[12] = (char)(Gz >> 8);

		Serial_Send(Msg, 13);
	}
	case 'a': {
		Flag[0] = 1;
		break;
	}
	case 'b': {
		Flag[1] = 1;
		break;
	}
	case 'B': {
		Flag[1] = 0;
		break;
	}
	case 'c': {
		Flag[2] = 1;
		break;
	}
	case 'd': {
		Flag[3] = 1;
		break;
	}
	case 'D': {
		Flag[3] = 0;
		break;
	}
	case FUNC_JOYSTICK_DATA: {
		//Flag[4] = 1;
		//BadFunc = 0;
		// Funkcja FUNC_JOYSTICK_DATA
		RF_TxData[0] = FUNC_JOYSTICK_DATA;
		RF_TxData[1] = Buf[1];
		RF_TxData[2] = Buf[2];
		RF_TxData[3] = Buf[3];
		RF_TxData[4] = Buf[4];
		break;
	}
	default: {
		if (Buf[0] >= 128){
			RF_TxData[0] = Buf[0];
			for (int a = 1; a < *Len; a ++){
				RF_TxData[a] = Buf[a];
			}
		}else {
			Flag[4] = 1;
			BadFunc = Buf[0];
		}
	}
	}
}

void Update(){
	if (HAL_GPIO_ReadPin(VIN_GPIO_Port, VIN_Pin)){
		// Zasilanie na USB
		HAL_GPIO_WritePin(ON_GPIO_Port, ON_Pin, 0);
		if (HAL_GPIO_ReadPin(BT_POWER_GPIO_Port, BT_POWER_Pin)){
			// Mosfet i tak przeskoczy, pzrerwa w zasilaniu zresetuje procka
		}
		// Ikona zasilania z usb
		SSD1306_DrawIcon16x16(0,48, plug_icon16x16);
		SSD1306_UpdateScreen();
	} else {
		// Zasilanie na Baterii
		if (HAL_GPIO_ReadPin(BT_POWER_GPIO_Port, BT_POWER_Pin)){
			// Wyłączyć
			SSD1306_Fill(0);
			SSD1306_GotoXY(20, 20);
			SSD1306_Puts("Byo :D", &Font_16x26, 1);
			SSD1306_UpdateScreen();
			HAL_Delay(1000);
			HAL_GPIO_WritePin(ON_GPIO_Port, ON_Pin, 0);
			HAL_Delay(1000);
		} else {
			// Ikona naładowania
			HAL_GPIO_WritePin(ON_GPIO_Port, ON_Pin, 1);
			int16_t Val[3];
			AdcRead(Val);
			int bat = MapValue(Val[2], 0,4095 , 0, 1000);
			if (bat > 390) {
				SSD1306_DrawIcon16x16(0,48, bat3_icon16x16);
			} else if (bat > 360) {
				SSD1306_DrawIcon16x16(0,48, bat2_icon16x16);
			} else if (bat > 330) {
				SSD1306_DrawIcon16x16(0,48, bat1_icon16x16);
			} else if (bat > 300) {
				SSD1306_DrawIcon16x16(0,48, bat0_icon16x16);
			} else {
				// Bateria rozładowana, wyłączenie
				SSD1306_Fill(0);
				SSD1306_GotoXY(20, 20);

				SSD1306_Puts("Battery", &Font_16x26, 1);
				SSD1306_GotoXY(20, 40);
				SSD1306_Puts("is flat", &Font_16x26, 1);
				SSD1306_UpdateScreen();
				HAL_Delay(1000);
				//SSD1306_DrawIcon16x16(0,48, cancel_icon16x16);
				//HAL_GPIO_WritePin(ON_GPIO_Port, ON_Pin, 0);
				//HAL_Delay(3000);
			}
			//SSD1306_UpdateScreen();
		}
	}
}

void Begin(){
	SSD1306_Fill(0);
	SSD1306_DrawBitmap(0, 0, logo_128x64, 128, 64);
	SSD1306_UpdateScreen();
	HAL_Delay(1000);
}

int SelectProgram(){
	int s_row = 0;
	int sel = 0;
	int B = 0;
	int D = 0;
	while(1){
		SSD1306_Fill(0);
		SSD1306_GotoXY(0, 0);
		SSD1306_Puts("Program Sel", &Font_11x18, 1);
		int start = 0;
		for (int a = 0; a < s_row; a++){
			start+=Program_length[a];
		}
		for (int num = 0; num < 3; num++){
			if (num + s_row > Program_Count-1) break;
			SSD1306_GotoXY(20, 20 + num*16);
			for (int c = 0; c < Program_length[s_row+num]; c++){
				SSD1306_Putc(Program_name[c + start], &Font_7x10, 1);
			}
			start+=Program_length[num+s_row];
		}
		SSD1306_DrawIcon16x16(0, 20-4 + (sel-s_row)*16, arrow_right_icon16x16);
		SSD1306_UpdateScreen();
		HAL_Delay(100);
		while(1){
			if (!HAL_GPIO_ReadPin(BT_RD_GPIO_Port, BT_RD_Pin)){
				if (B == 0){
					sel--;
					B = 1;
					break;
				}
			} else B = 0;
			if (!HAL_GPIO_ReadPin(BT_RB_GPIO_Port, BT_RB_Pin)){
				if (D == 0){
					sel++;
					D = 1;
					break;
				}
			} else D = 0;
			if (!HAL_GPIO_ReadPin(BT_RC_GPIO_Port, BT_RC_Pin)) return sel;
		}
		if (sel - 2> s_row) {	// Scroll w dół
			s_row++;
		}
		if (sel < s_row) {		// scroll w górę
			s_row--;
		}
		if (s_row + 3 > Program_Count) {	// Przepełnienie w dół
			s_row = sel = 0;
		}
		if (s_row < 0) {		// Pzrepelnienie w górę
			s_row = Program_Count-3;
			sel = Program_Count-1;
		}
	}
}

void InitDevice_NRF(){
	NRF24_begin(CSN_GPIO_Port, CSN_Pin, CE_Pin, hspi1);
	NRF24_setAutoAck(true);
	NRF24_setChannel(52);
	NRF24_setPayloadSize(13);
	NRF24_openReadingPipe(1, PipeAddres);
	NRF24_enableDynamicPayloads();
	NRF24_enableAckPayload();
	NRF24_startListening();
}

void InitDevice_MPU(){
	MPU_ConfigTypeDef MpuConfig;
	MPU6050_Init(&hi2c1);
	MpuConfig.Accel_Full_Scale = AFS_SEL_4g;
	MpuConfig.ClockSource = Internal_8MHz;
	MpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
	MpuConfig.Gyro_Full_Scale = FS_SEL_500;
	MpuConfig.Sleep_Mode_Bit = 0;
	MPU6050_Config(&MpuConfig);
}

void Init_Test(){

}

void Loop_Test(){
	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("Program", &Font_11x18, 1);
	SSD1306_GotoXY(0, 20);
	SSD1306_Puts("testowy", &Font_11x18, 1);
}

void Init_Gamepad(){
	my_MX_USB_DEVICE_Init();
	InitDevice_MPU();
}

void Loop_Gamepad(){
	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("GAMEPAD", &Font_11x18, 1);
	updateButtons();
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, dataToSend, 20);
	HAL_Delay(10);
}

void Init_SMicromouse(){
	MX_USB_DEVICE_Init();
	InitDevice_MPU();
	InitDevice_NRF();
}

void Loop_SMicromouse(){
	SSD1306_Fill(0);
	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("Micromouse", &Font_11x18, 1);
	if (NRF24_available()){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		NRF24_read(RF_RxData, 13);
		Serial_Send(RF_RxData, 13);
		NRF24_writeAckPayload(1, RF_TxData, 16);
		// BARDZO ISTOTNE, I2C i NRF ZAKLUCAJA SIE JAKOS, DLA TEGO DANE ZBIERAMY PO TRANSMISJI NRF
		// ZAKLUCENIA NADAL WYSTEPUJA ALE ZADZIEJ, JE WYELIMINOWAC TRZEBA INACZEJ ??
		// Ustawienie funkcji
		RF_TxData[0] = 0;
		int16_t Val[4];
		if (Flag[3] == 1) {
		  // Ustawienie funkcji
		  RF_TxData[0] = FUNC_JOYSTICK_DATA;
		  // Odczyt z ADS1115
		  for (int a = 0; a < 4; a++){
			  ADSwrite[0] = 0x01;
			  switch(a){
			  case 0: {
				  ADSwrite[1] = 0xC1; // 11000011
				  break;
			  }
			  case 1: {
				  ADSwrite[1] = 0xD1; // 11010011
				  break;
			  }
			  case 2: {
				  ADSwrite[1] = 0xE1; // 11100011
				  break;
			  }
			  case 3: {
				  ADSwrite[1] = 0xF1; // 11110011
				  break;
			  }
			  }
			  /*
			  __HAL_RCC_I2C2_FORCE_RESET();
			  __HAL_RCC_I2C2_RELEASE_RESET();
			  MX_I2C2_Init();
			  __HAL_RCC_I2C2_FORCE_RESET();
			  __HAL_RCC_I2C2_RELEASE_RESET();
			  MX_I2C2_Init();*/


			  ADSwrite[2] = 0xE3; // 10000011 // 10100011 // 11000011// 11100011
			  HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDRESS<<1, ADSwrite, 3, 100);
			  ADSwrite[0] = 0x00;
			  HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDRESS<<1, ADSwrite, 1, 100);
			  //HAL_Delay(1);
			  NRF24_DelayMicroSeconds(100);
			  HAL_I2C_Master_Receive(&hi2c2, ADS1115_ADDRESS<<1, ADSwrite, 2, 100);

			  //RF_TxData[1 + a*2] = ADSwrite[1];
			  //RF_TxData[1 + a*2 + 1] = ADSwrite[0];
			  Val[a] = (((int16_t)ADSwrite[0]) << 8 | ADSwrite[1]);
			}
		  //Val[1] = (int16_t)MapValue(Val[1], 0, Val[0], -1023, 1023);
		  Val[2] = (int16_t)MapValue(Val[2], 0, Val[0], -1023, 1023) - 22;
		  Val[3] = (int16_t)MapValue(Val[3], 0, Val[0], -1023, 1023) - 22;
		  //Val[0] = (int16_t)MapValue(Val[0], 0, Val[0], -1023, 1023);

		  if ((Val[2] <= 6) && (Val[2] >= -6)) Val[2] = 0;
		  if ((Val[3] <= 6) && (Val[3] >= -6)) Val[3] = 0;

		  RF_TxData[1] = Val[2];
		  RF_TxData[2] = Val[2] >> 8;
		  RF_TxData[3] = Val[3];
		  RF_TxData[4] = Val[3] >> 8;

		  // Nastepne odczyty...

		}
		}
		if (Flag[4] == 1){
		Flag[4] = 0;
		char Bad[6] = {'B', 'A', 'D', '(', BadFunc, ')'};
		Serial_Send(Bad, 6);
		HAL_Delay(1000);
		Bad[0] = BadFunc;
		Serial_Send(Bad, 6);
		}
		if ((Flag[0] == 1) || (Flag[1] == 1)) {
		Flag[0] = 0;

		I2C_ClearBusyFlagErratum(&hi2c1, 1000);

		__HAL_RCC_I2C1_FORCE_RESET();
		__HAL_RCC_I2C1_RELEASE_RESET();
		MX_I2C1_Init();
		__HAL_RCC_I2C1_FORCE_RESET();
		__HAL_RCC_I2C1_RELEASE_RESET();
		MX_I2C1_Init();

		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);

		MPU6050_Get_Accel_RawData(&AccelData);	// Najpier trzeba akcelerometr
		MPU6050_Get_Gyro_RawData(&GyroData);		// Potem zyroskop

		Msg[0] = FUNC_ACCEL_GYRO_DATA;
		Msg[1] = AccelData.x;
		Msg[2] = AccelData.x >> 8;
		Msg[3] = AccelData.y;
		Msg[4] = AccelData.y >> 8;
		Msg[5] = AccelData.z;
		Msg[6] = AccelData.z >> 8;

		Msg[7] = GyroData.x;
		Msg[8] = GyroData.x >> 8;
		Msg[9] = GyroData.y;
		Msg[10] = GyroData.y >> 8;
		Msg[11] = GyroData.z;
		Msg[12] = GyroData.z >> 8;

		Serial_Send(Msg, 13);
		HAL_Delay(10);
		}
		if (Flag[2] == 1){
		Flag[2] = 0;
		// Odczyt z ADS1115
		int16_t Val[4];
		for (int a = 0; a < 4; a++){
		  ADSwrite[0] = 0x01;
		  switch(a){
		  case 0: {
			  ADSwrite[1] = 0xC1; // 11000011
			  break;
		  }
		  case 1: {
			  ADSwrite[1] = 0xD1; // 11010011
			  break;
		  }
		  case 2: {
			  ADSwrite[1] = 0xE1; // 11100011
			  break;
		  }
		  case 3: {
			  ADSwrite[1] = 0xF1; // 11110011
			  break;
		  }
		  }

		  /*
		  __HAL_RCC_I2C2_FORCE_RESET();
		  __HAL_RCC_I2C2_RELEASE_RESET();
		  MX_I2C2_Init();
		  __HAL_RCC_I2C2_FORCE_RESET();
		  __HAL_RCC_I2C2_RELEASE_RESET();
		  MX_I2C2_Init();*/

		  ADSwrite[2] = 0xE3; // 10000011 // 10100011 // 11000011// 11100011
		  HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDRESS<<1, ADSwrite, 3, 100);
		  ADSwrite[0] = 0x00;
		  HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDRESS<<1, ADSwrite, 1, 100);
		  //HAL_Delay(1);
		  NRF24_DelayMicroSeconds(100);
		  HAL_I2C_Master_Receive(&hi2c2, ADS1115_ADDRESS<<1, ADSwrite, 2, 100);

		  //Msg[1 + a*2] = ADSwrite[1];
		  //Msg[1 + a*2 + 1] = ADSwrite[0];
		  Val[a] = (((int16_t)ADSwrite[0]) << 8 | ADSwrite[1]);
		  //Serial_Send("ELO", 3);

		}

		Val[1] = (int16_t)MapValue(Val[1], 0, Val[0], -1023, 1023);
		Val[2] = (int16_t)MapValue(Val[2], 0, Val[0], -1023, 1023) - 22;
		Val[3] = (int16_t)MapValue(Val[3], 0, Val[0], -1023, 1023) -22;
		Val[0] = (int16_t)MapValue(Val[0], 0, Val[0], -1023, 1023);

		if ((Val[2] <= 3) && (Val[2] >= -3)) Val[2] = 0;
		if ((Val[3] <= 3) && (Val[3] >= -3)) Val[3] = 0;

		Msg[0] = FUNC_JOYSTICK_DATA;
		Msg[1] = Val[2];
		Msg[2] = Val[2] >> 8;
		Msg[3] = Val[3];
		Msg[4] = Val[3] >> 8;
		if (Val[1] > 0) Msg[5] = 0; else Msg[5] = 128;
		if (HAL_GPIO_ReadPin(BT_LA_GPIO_Port, BT_LA_Pin) == GPIO_PIN_RESET) Msg[5] += 64;
		if (HAL_GPIO_ReadPin(BT_LB_GPIO_Port, BT_LB_Pin) == GPIO_PIN_RESET) Msg[5] += 32;
		if (HAL_GPIO_ReadPin(BT_LC_GPIO_Port, BT_LC_Pin) == GPIO_PIN_RESET) Msg[5] += 16;
		if (HAL_GPIO_ReadPin(BT_LD_GPIO_Port, BT_LD_Pin) == GPIO_PIN_RESET) Msg[5] += 8;

		if (HAL_GPIO_ReadPin(BT_POWER_GPIO_Port, BT_POWER_Pin) == GPIO_PIN_SET) Msg[5] += 1;

		Msg[6] = 0;
		Msg[7] = 0;
		Msg[8] = 0;
		Msg[9] = 0;

		Msg[10] = 0;
		if (HAL_GPIO_ReadPin(BT_RS_GPIO_Port, BT_RS_Pin) == GPIO_PIN_RESET) Msg[10] += 128;
		if (HAL_GPIO_ReadPin(BT_RA_GPIO_Port, BT_RA_Pin) == GPIO_PIN_RESET) Msg[10] += 64;
		if (HAL_GPIO_ReadPin(BT_RB_GPIO_Port, BT_RB_Pin) == GPIO_PIN_RESET) Msg[10] += 32;
		if (HAL_GPIO_ReadPin(BT_RC_GPIO_Port, BT_RC_Pin) == GPIO_PIN_RESET) Msg[10] += 16;
		if (HAL_GPIO_ReadPin(BT_RD_GPIO_Port, BT_RD_Pin) == GPIO_PIN_RESET) Msg[10] += 8;

		// Nastepne odczyty

		//RF_TXData[5] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)

		Serial_Send(Msg, 11);

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  //MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

// Zasilanie
  if (!HAL_GPIO_ReadPin(VIN_GPIO_Port, VIN_Pin)){
	  // Zasilanie baterynje
    HAL_GPIO_WritePin(ON_GPIO_Port, ON_Pin, 1);
  }
// Reanimacja I2C
  I2C_ClearBusyFlagErratum(&hi2c1, 1000);
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();
  MX_I2C1_Init();
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();
  MX_I2C1_Init();
  //I2C_ClearBusyFlagErratum2(&hi2c2, 1000);
  __HAL_RCC_I2C2_FORCE_RESET();
  __HAL_RCC_I2C2_RELEASE_RESET();
  MX_I2C2_Init();
  __HAL_RCC_I2C2_FORCE_RESET();
  __HAL_RCC_I2C2_RELEASE_RESET();
  MX_I2C2_Init();
// Ekran
  SSD1306_Init ();
//
  	Begin();
	Program_Running = SelectProgram();
	switch (Program_Running){
		case 0: {
			Init_Gamepad();
			break;
		}
		case 1: {
			Init_SMicromouse();
			break;
		}
		default:  {
			Init_Test();
			break;
		}
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	SSD1306_Fill(0);
	switch (Program_Running){
		case 0: {
			Loop_Gamepad();
			break;
		}
		case 1: {
			Loop_SMicromouse();
			break;
		}
		default:  {
			Loop_Test();
			break;
		}
	}

	Update();
	SSD1306_UpdateScreen();
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	Error_Handler();
  }
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
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
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BT_LD_Pin BT_LA_Pin BT_RS_Pin BT_RA_Pin 
                           BT_RB_Pin BT_RC_Pin BT_LB_Pin VIN_Pin 
                           BT_POWER_Pin */
  GPIO_InitStruct.Pin = BT_LD_Pin|BT_LA_Pin|BT_RS_Pin|BT_RA_Pin 
                          |BT_RB_Pin|BT_RC_Pin|BT_LB_Pin|VIN_Pin 
                          |BT_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BT_RD_Pin BT_LC_Pin */
  GPIO_InitStruct.Pin = BT_RD_Pin|BT_LC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ON_Pin MOS_Pin */
  GPIO_InitStruct.Pin = ON_Pin|MOS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
