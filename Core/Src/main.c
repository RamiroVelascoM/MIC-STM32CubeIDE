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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "button.h"
#include "ESP01.h"
#include "MPU6050.h"
#include "UNERBUS.h"
#include "display.h"
#include "fonts.h"
#include <stdbool.h>
// #include "stm32f1xx_hal_flash.h"
// #include "stm32f1xx_hal_flash_ex.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * ENUMERACIONES
 */
typedef enum{
    ACKNOWLEDGE = 0x0D,
	GETLOCALIP = 0xE0,
    ALIVE = 0xF0,
    FIRMWARE = 0xF1,
	ANALOG_IR = 0xF2,
	MPU_6050 = 0xF3,
	DISPLAY_SSD1306 = 0xF4,
    UNKNOWNCOMMAND = 0xFF
}_eID;

/**
 * UNIONES
 */
typedef union{
	struct{
		uint8_t	b0:	1;
		uint8_t	b1:	1;
		uint8_t	b2:	1;
		uint8_t	b3:	1;
		uint8_t	b4:	1;
		uint8_t	b5:	1;
		uint8_t	b6:	1;
		uint8_t	b7:	1;
	} bit;
	uint8_t byte;
} _uFlag;

typedef union{
	uint8_t		u8[4];
	int8_t		i8[4];
	uint16_t	u16[2];
	int16_t		i16[2];
	uint32_t	u32;
	int32_t		i32;
} _uWork;

/**
 * ESTRUCTURAS
 */
typedef struct{
	uint8_t index[8];
	uint16_t data[8];
	uint16_t value[8];
	uint16_t buf[8][64];
	uint32_t sum[8];
}_sADC;

typedef struct{
	uint8_t distanceValues[20];
	uint8_t distanceInMm;
	uint16_t distanceMeasured;
}_sTCRT5000;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define modeIDLE				0
#define modeONE					1
#define modeTWO					2
#define maxMODES				3

#define OFF						0
#define ON						1
#define PRESSED					0
#define NUMCHANNELSADC			8
#define TIMEOUT_BUTTON			4
#define TIMEOUT_ALIVE			50

#define SIZEBUFADC				64
#define SIZEBUFRXPC				128
#define SIZEBUFTXPC				256
#define SIZEBUFRXESP01			128
#define SIZEBUFTXESP01			128

#define HEARTBEAT_MASK			0x80000000
#define	HEARTBEAT_IDLE			0xF0A0F000
#define	HEARTBEAT_WIFI_READY	0xF0A0A0A0
#define	HEARTBEAT_UDP_READY		0xF0AAF0AA

#define WIFI_SSID				"WiFi Velasco Fibertel"
#define WIFI_PASSWORD			"ncgrmvelasco"
#define WIFI_UDP_REMOTE_IP		"192.168.1.8"		//La IP de la PC
#define WIFI_UDP_LOCAL_PORT		30000
#define WIFI_UDP_REMOTE_PORT	30000				//El puerto UDP en la PC

#define ON10MS					flag1.bit.b0
#define I2CENABLED				flag1.bit.b1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/**
 * DEFINICION DE UNIONES
 */
_uFlag flag1;
_uWork w;

/**
 * DEFINICION DE ESTRUCTURAS
 */
_sESP01Handle esp01;
_sUNERBUSHandle unerbusPC;
_sUNERBUSHandle unerbusESP01;
_sADC myADC;
_sTCRT5000 myTCRT5000[NUMCHANNELSADC];
_sButton myButton;
_sMPUData mpuValues;

/**
 * DEFINICION DE DATOS DE COMUNICACION
 */
uint8_t bufRXPC[SIZEBUFRXPC], bufTXPC[SIZEBUFTXPC];
uint8_t bufRXESP01[SIZEBUFRXESP01], bufTXESP01[SIZEBUFTXESP01], dataRXESP01;
uint8_t rxUSBData, newData;

/**
 * DEFINICION DE LAS VARIABLES UTILIZADAS PARA EL ADC
 */
uint8_t indexADC[NUMCHANNELSADC];
uint16_t dataADC[NUMCHANNELSADC];
uint16_t valueADC[NUMCHANNELSADC];
uint16_t bufADC[NUMCHANNELSADC][SIZEBUFADC];
uint32_t sumADC[NUMCHANNELSADC];

/**
 * DEFINICION DE DATOS DE MANEJO DE PROGRAMA
 */
uint8_t mode			= 0;
uint8_t time10ms 		= 40;
uint8_t time100ms		= 10;
uint8_t time1000ms		= 100;
uint8_t	timeOutButton	= TIMEOUT_BUTTON;
uint8_t timeOutAlive 	= TIMEOUT_ALIVE;
uint32_t myHB			= HEARTBEAT_IDLE;
uint32_t maskHB			= HEARTBEAT_MASK;

//const uint32_t token __attribute__ ((section (".eeprom"), used));
char strAux[32];

/**
 * BITMAP BACKGROUND
 */
const uint8_t myDesignBKG[] __attribute__ ((section (".eeprom"), used)) = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x7c, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78,
0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c,
0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e,
0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f,
0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07,
0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83,
0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1,
0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0,
0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0,
0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78,
0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c,
0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e,
0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f,
0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07,
0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83,
0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1,
0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0,
0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0,
0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78,
0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c,
0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e,
0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f,
0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07,
0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83,
0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1,
0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0,
0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0,
0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78,
0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c,
0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e,
0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f,
0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07,
0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83,
0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1,
0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0,
0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0,
0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78,
0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c,
0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0x0f, 0x07, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void ESP01DoCHPD(uint8_t value);

int ESP01WriteUSARTByte(uint8_t value);

void ESP01WriteByteToBufRX(uint8_t value);

void ESP01ChangeState(_eESP01STATUS esp01State);

void DecodeCMD(struct UNERBUSHandle *aBus, uint8_t iStartData);

void Do10ms();

void USBReceive(uint8_t *buf, uint16_t len);

void buttonTask(_sButton *button);

void communicationTask();

void inicializarIRs();

void aliveTask();

void I2CTasks();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//CALLBACKS
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1){
		time10ms--;
		if(!time10ms){
			ON10MS = 1;
			time10ms = 40;
		}
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&myADC.data, NUMCHANNELSADC);
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	for (uint8_t c=0; c<NUMCHANNELSADC; c++)
	{
		myADC.sum[c] -= myADC.buf[c][myADC.index[c]];
		myADC.sum[c] += myADC.data[c];
		myADC.buf[c][myADC.index[c]] = myADC.data[c];
		myADC.value[c] = myADC.sum[c]/SIZEBUFADC;
		myADC.index[c]++;
		myADC.index[c] &= (SIZEBUFADC-1);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		ESP01_WriteRX(dataRXESP01);
		HAL_UART_Receive_IT(&huart1, &dataRXESP01, 1);
	}
}

//.

void ESP01DoCHPD(uint8_t value){
	HAL_GPIO_WritePin(CH_EN_GPIO_Port, CH_EN_Pin, value);
}

int ESP01WriteUSARTByte(uint8_t value){
	if(__HAL_UART_GET_FLAG(&huart1, USART_SR_TXE)){
		USART1->DR = value;
		return 1;
	}
	return 0;
}

void ESP01WriteByteToBufRX(uint8_t value){
	UNERBUS_ReceiveByte(&unerbusESP01, value);
}

void ESP01ChangeState(_eESP01STATUS esp01State){
	switch((uint32_t)esp01State){
	case ESP01_WIFI_CONNECTED:
		myHB = HEARTBEAT_WIFI_READY;
		break;
	case ESP01_UDPTCP_CONNECTED:
		myHB = HEARTBEAT_UDP_READY;
		break;
	case ESP01_UDPTCP_DISCONNECTED:
		myHB = HEARTBEAT_WIFI_READY;
		break;
	case ESP01_WIFI_DISCONNECTED:
		myHB = HEARTBEAT_IDLE;
		break;
	}
}


void DecodeCMD(struct UNERBUSHandle *aBus, uint8_t iStartData){
	uint8_t id;
	uint8_t length = 0;

	id = UNERBUS_GetUInt8(aBus);
	switch(id){
		case GETLOCALIP:
			UNERBUS_Write(aBus, (uint8_t *)ESP01_GetLocalIP(), 16);
			length = 17;
			break;
		case ALIVE:
			UNERBUS_WriteByte(aBus, 0x0D);
			length = 2;
			break;
		case ANALOG_IR:
			for (uint8_t c=0; c<NUMCHANNELSADC; c++)
			{
				w.u32 = myADC.value[c];
				UNERBUS_WriteByte(aBus, w.u8[0]);
				UNERBUS_WriteByte(aBus, w.u8[1]);
			}
			length = 17;
			break;
		case MPU_6050:
			UNERBUS_Write(aBus, mpuValues.buffer, 12);
			length = 13;
			break;
		default:
			break;
	}

	if(length){
		UNERBUS_Send(aBus, id, length);
	}
}

void Do10ms(){
	ON10MS = 0;

	if (time100ms)
		time100ms--;

	if (time1000ms)
		time1000ms--;

	ESP01_Timeout10ms();
	UNERBUS_Timeout(&unerbusESP01);
	UNERBUS_Timeout(&unerbusPC);
}

void Do100ms(){
	time100ms = 10;

	if (maskHB & myHB)
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0); // ON
	else
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1); // OFF

	maskHB >>= 1;
	if (!maskHB)
		maskHB = HEARTBEAT_MASK;

	if (timeOutAlive)
		timeOutAlive--;

	if (timeOutButton)
		timeOutButton--;

	I2CENABLED = ON;
}


void USBReceive(uint8_t *buf, uint16_t len){
	UNERBUS_ReceiveBuf(&unerbusPC, buf, len);
}

void buttonTask(_sButton *button){
	timeOutButton = TIMEOUT_BUTTON;

	myButton.value = HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin);
	checkMEF(&myButton);

	switch (button->estado){
		case DOWN:
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);	// ON
			break;
		case RISING:
			mode++;
			if (mode == maxMODES)
				mode = 0;						// Increase mode (circular: 0-MAX)
			break;
		default:
			break;
	}
}

void communicationTask(){
	if(unerbusESP01.tx.iRead != unerbusESP01.tx.iWrite){
		w.u8[0] = unerbusESP01.tx.iWrite - unerbusESP01.tx.iRead;
		w.u8[0] &= unerbusESP01.tx.maxIndexRingBuf;
		if(ESP01_Send(unerbusESP01.tx.buf, unerbusESP01.tx.iRead, w.u8[0], unerbusESP01.tx.maxIndexRingBuf+1) == ESP01_SEND_READY)
			unerbusESP01.tx.iRead = unerbusESP01.tx.iWrite;
	}

	if(unerbusPC.tx.iRead != unerbusPC.tx.iWrite){
		if(unerbusPC.tx.iRead < unerbusPC.tx.iWrite)
			w.u8[0] = unerbusPC.tx.iWrite - unerbusPC.tx.iRead;
		else
			w.u8[0] = unerbusPC.tx.maxIndexRingBuf+1 - unerbusPC.tx.iRead;

		if(CDC_Transmit_FS(&unerbusPC.tx.buf[unerbusPC.tx.iRead], w.u8[0]) == USBD_OK){
			unerbusPC.tx.iRead += w.u8[0];
			unerbusPC.tx.iRead &= unerbusPC.tx.maxIndexRingBuf;
		}
	}
}

void inicializarIRs(){
	uint8_t initialValues[20] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
	for (uint8_t i=0;i<20;i++)
	{
		myTCRT5000[0].distanceValues[i] = initialValues[i];
	}
}

void aliveTask(){
	if (!timeOutAlive){
		UNERBUS_WriteByte(&unerbusESP01, ACKNOWLEDGE);
		UNERBUS_Send(&unerbusESP01, ALIVE, 2);
		UNERBUS_WriteByte(&unerbusPC, ACKNOWLEDGE);
		UNERBUS_Send(&unerbusPC, ALIVE, 2);
		timeOutAlive = TIMEOUT_ALIVE;
	}
}

void I2CTasks(){
	if (!time1000ms){
		sprintf(strAux, "%.3d", mode++);
		Display_SetCursor(90, 8);
		Display_WriteString(strAux, Font_7x10, Black);
		I2CENABLED = ON;
		mode &= 255;
		time1000ms = 100;
	}

	if ((HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) && I2CENABLED){
		I2CENABLED = Display_UpdateScreen(&hi2c2);
	}

	if ((HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) && I2CENABLED){
		MPU6050_Read_Data_DMA(&hi2c2);
		I2CENABLED = OFF;
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
  HAL_Delay(MPU_TIMEOUT);

  /**
   * INITIALIZE ESP01 HANDLE DATA
   */
  esp01.DoCHPD = ESP01DoCHPD;
  esp01.WriteByteToBufRX = ESP01WriteByteToBufRX;
  esp01.WriteUSARTByte = ESP01WriteUSARTByte;
  ESP01_Init(&esp01);
  ESP01_AttachChangeState(ESP01ChangeState);
  ESP01_SetWIFI(WIFI_SSID, WIFI_PASSWORD);
  ESP01_StartUDP(WIFI_UDP_REMOTE_IP, WIFI_UDP_REMOTE_PORT, WIFI_UDP_LOCAL_PORT);

  /**
   * INITIALIZE UNERBUS ESP01
   */
  unerbusESP01.MyDataReady = DecodeCMD;
  unerbusESP01.WriteUSARTByte = NULL;
  unerbusESP01.rx.buf = bufRXESP01;
  unerbusESP01.rx.maxIndexRingBuf = (SIZEBUFRXESP01 - 1);
  unerbusESP01.tx.buf = bufTXESP01;
  unerbusESP01.tx.maxIndexRingBuf = (SIZEBUFTXESP01 - 1);
  UNERBUS_Init(&unerbusESP01);

  /**
   * INITIALIZE UNERBUS PC
   */
  unerbusPC.MyDataReady = DecodeCMD;
  unerbusPC.WriteUSARTByte = NULL;
  unerbusPC.rx.buf = bufRXPC;
  unerbusPC.rx.maxIndexRingBuf = (SIZEBUFRXPC - 1);
  unerbusPC.tx.buf = bufTXPC;
  unerbusPC.tx.maxIndexRingBuf = (SIZEBUFTXPC - 1);
  UNERBUS_Init(&unerbusPC);

  /**
   * INITIALIZATION OF OTHER FUNCTIONS
   */
  inicializarBoton(&myButton);
  inicializarIRs();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  CDC_AttachRxData(USBReceive);

  HAL_UART_Receive_IT(&huart1, &dataRXESP01, 1);

  HAL_TIM_Base_Start_IT(&htim1);

  MPU6050_Init(&hi2c2);

  Display_Init(&hi2c2);

  Display_DrawBitmap(Display_WIDTH, Display_HEIGHT, myDesignBKG);

  Display_SetCursor(20, 8);
  Display_WriteString("TESTING", Font_7x10, Black);

  flag1.byte = OFF;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (!timeOutButton)
		  buttonTask(&myButton);

	  if (!time100ms)
		  Do100ms();

	  if (ON10MS)
		  Do10ms();

	  aliveTask();

	  communicationTask();

	  ESP01_Task();

	  UNERBUS_Task(&unerbusESP01);

	  UNERBUS_Task(&unerbusPC);

	  I2CTasks();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 250;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CH_EN_GPIO_Port, CH_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW0_Pin */
  GPIO_InitStruct.Pin = SW0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CH_EN_Pin */
  GPIO_InitStruct.Pin = CH_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CH_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_TIM3_PARTIAL();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
