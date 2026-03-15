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
#include "motor.h"
//#include "TCRT5000.h"
#include "INFRARED.h"
#include <stdbool.h>
#include "PID.h"

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
	RECEIVE_N20 = 0xF5,
	SEND_N20 = 0xF6,
	BUTTONS = 0xF7,

	SET_IR_THRESHOLD = 0xE1,
	SET_PID_NAV = 0xE2,
	SET_PID_TURN = 0xE3,
	SET_PID_U_TURN = 0xE4,

    UNKNOWNCOMMAND = 0xFF
}_eID;

typedef enum{
	ACTION_STANDBY = 1,
	ACTION_AVANZAR = 2,
	ACTION_GIRO_DERECHA = 3,
	ACTION_GIRO_IZQUIERDA = 4,
	ACTION_GIRO_EN_U = 5,
	ACTION_CRUCE_CIEGO = 6,
	ACTION_CRUCE_T = 7,
	ACTION_CRUCE_L = 8,
	ACTION_CRUCE_J = 9,
	ACTION_CRUCE_X = 10
}_eACTIONS;

typedef enum{
	MODE_IDLE = 1,
	MODE_EXPLORE_MAZE = 2,
	MODE_SOLVE_MAZE = 3
}_eMODES;
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
}_uFlag;

typedef union{
	uint8_t		u8[4];
	int8_t		i8[4];
	uint16_t	u16[2];
	int16_t		i16[2];
	uint32_t	u32;
	int32_t		i32;
}_uWork;


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

#define TIMEOUT_BUTTON			4
#define TIMEOUT_ALIVE			50
#define TIMEOUT_MOTORS			20

#define SIZEBUFRXPC				256
#define SIZEBUFTXPC				256
#define SIZEBUFRXESP01			256
#define SIZEBUFTXESP01			256

#define HEARTBEAT_MASK			0x80000000
#define	HEARTBEAT_IDLE			0xFF00C000
#define HEARTBEAT_MODE1			0xFF00CC00
#define HEARTBEAT_MODE2			0xFF00CCC0
#define	HEARTBEAT_WIFI_READY	0xF0A0A0A0
#define	HEARTBEAT_UDP_READY		0xF0AAF0AA

#define WIFI_SSID				"FCAL"//"WiFi Velasco"//
#define WIFI_PASSWORD			"fcalconcordia.06-2019"//"ncgrmvelasco"//
#define WIFI_UDP_REMOTE_IP		"172.23.207.122"//"192.168.1.7"//
#define WIFI_UDP_LOCAL_PORT		30000
#define WIFI_UDP_REMOTE_PORT	30030

#define ON10MS					flag1.bit.b0
#define MPUENABLED				flag1.bit.b1
#define MODESTARTED				flag1.bit.b2
#define TURNING					flag1.bit.b3
#define CHOSESIDE				flag1.bit.b4
#define TURNRIGHT				flag1.bit.b5
#define TURNSMOOTH				flag1.bit.b6

/*
#define LEFT_SIDE				6
#define RIGHT_SIDE				0
#define FRONT_LEFT				5
#define FRONT_RIGHT				1
#define FRONT_1					2
#define FRONT_2					4
#define GROUND_BACK				7
#define GROUND_FRONT			3
*/

#define IR_IZQ					myADC.millimeterSamples[6]//6
#define IR_DER					myADC.millimeterSamples[0]//0
#define IR_FRONT_1				myADC.millimeterSamples[2]//2
#define IR_FRONT_2				myADC.millimeterSamples[4]//4
#define IR_DIAG_DER				myADC.millimeterSamples[1]//1
#define IR_DIAG_IZQ				myADC.millimeterSamples[5]//5
#define IR_GROUND_FRONT			myADC.millimeterSamples[3]
#define IR_GROUND_BACK			myADC.millimeterSamples[7]

#define PARED_DERECHA			flag2.bit.b0
#define DIAG_DERECHA			flag2.bit.b1
#define PARED_DELANTERA			flag2.bit.b2
#define PISO_ADELANTE			flag2.bit.b3
#define PARED_DELANTERA_2		flag2.bit.b4
#define DIAG_IZQUIERDA			flag2.bit.b5
#define PARED_IZQUIERDA			flag2.bit.b6
#define PISO_ATRAS				flag2.bit.b7

#define SENSORES_IR				flag2.byte
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
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
_uFlag flag1, flag2;
_uWork w;

/**
 * DEFINICION DE ESTRUCTURAS
 */
_sESP01Handle esp01;
_sUNERBUSHandle unerbusPC;
_sUNERBUSHandle unerbusESP01;
_sButton myButton;
_sMPUData myMPU;
_sDisplayData myDisplay;
_sMOTOR myMotor[2];
_sPID PID_Navigation, PID_Turn, PID_U_Turn;
InfraredHandle_s myADC;

_eACTIONS robotAction;
_eMODES robotMode;
/**
 * DEFINICION DE DATOS DE COMUNICACION
 */
uint8_t bufRXPC[SIZEBUFRXPC], bufTXPC[SIZEBUFTXPC];
uint8_t bufRXESP01[SIZEBUFRXESP01], bufTXESP01[SIZEBUFTXESP01], dataRXESP01;
uint8_t rxUSBData, newData;
uint8_t UPDATEDISPLAY = 0;
int8_t testds = 0;
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

char strAux[30];
/**
 * BITMAP BACKGROUND
 */
const uint8_t myDesignBKG[] __attribute__ ((section (".eeprom"), used)) =
		{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x3f, 0x80, 0x00, 0x7f, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x0f, 0xe0, 0x00, 0x1f, 0xc0, 0x00, 0x3f, 0xff, 0xff,
		0xff, 0xff, 0xff, 0x60, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x00, 0x0f, 0xe0, 0x00, 0x1f, 0xff, 0xff,
		0xff, 0xff, 0xfe, 0x30, 0x00, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x07, 0xf0, 0x00, 0x0f, 0xff, 0xff,
		0xff, 0xff, 0xfc, 0x18, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x03, 0xf8, 0x00, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xfc, 0x0f, 0xff, 0xc1, 0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0xe0, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xfc, 0x07, 0xff, 0xc1, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xc0, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xfe, 0x03, 0xff, 0xc1, 0xe0, 0x3f, 0x80, 0x00, 0xc0, 0x7f, 0x80, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xff, 0x01, 0xff, 0xc1, 0xe0, 0x1f, 0xc0, 0x00, 0x60, 0x3f, 0x01, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xff, 0x80, 0xff, 0xc1, 0xe0, 0x0f, 0xe0, 0x00, 0x30, 0x1e, 0x03, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xc0, 0x7f, 0xc1, 0xe0, 0x07, 0xf0, 0x00, 0x38, 0x0c, 0x07, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xe0, 0x3f, 0xc1, 0xe0, 0x03, 0xf8, 0x00, 0x3c, 0x00, 0x0f, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xf0, 0x1f, 0xc1, 0xe0, 0x7f, 0xff, 0xf8, 0x3e, 0x00, 0x1f, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xf8, 0x0f, 0xc1, 0xe0, 0x3f, 0xff, 0xf8, 0x3f, 0x00, 0x3f, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xfe, 0x7c, 0x07, 0xc1, 0xf0, 0x1f, 0xff, 0xf8, 0x3f, 0x80, 0x7f, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xfc, 0x3e, 0x03, 0xc0, 0xf8, 0x0f, 0xff, 0xf8, 0x3f, 0xc0, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xf8, 0x1f, 0x01, 0xc0, 0x7c, 0x07, 0xff, 0xf8, 0x3f, 0xe1, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xf0, 0x0f, 0x80, 0xe0, 0x3e, 0x03, 0xff, 0xf8, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xe0, 0x07, 0xc0, 0x70, 0x1f, 0x01, 0xff, 0xf8, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc0, 0x03, 0xe0, 0x38, 0x0f, 0x80, 0xff, 0xf8, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0x01, 0xf0, 0x1c, 0x07, 0xc0, 0x7f, 0xf8, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0x80, 0xf8, 0x0e, 0x03, 0xe0, 0x3f, 0xf8, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xc0, 0x7c, 0x07, 0x01, 0xf0, 0x1f, 0xf8, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xe0, 0x3e, 0x03, 0x80, 0xf8, 0x0f, 0xf8, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xf0, 0x00, 0x01, 0xc0, 0x7c, 0x07, 0xf8, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xf8, 0x00, 0x01, 0xe0, 0x3e, 0x03, 0xf8, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xfc, 0x00, 0x01, 0xf0, 0x1f, 0x01, 0xf8, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xfe, 0x00, 0x03, 0xf8, 0x0f, 0x80, 0xf8, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xff, 0x00, 0x07, 0xfc, 0x07, 0xc0, 0x78, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xff, 0x80, 0xff, 0xfe, 0x03, 0xe0, 0x38, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xff, 0xc0, 0x7f, 0xff, 0x01, 0xf0, 0x18, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xff, 0xe0, 0x3f, 0xff, 0x80, 0xf8, 0x08, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xff, 0xf0, 0x1f, 0xff, 0xc0, 0x7c, 0x00, 0x3f, 0xff, 0xff, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xff, 0xf8, 0x0f, 0xff, 0xe0, 0x3e, 0x00, 0x3f, 0xff, 0xfe, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xff, 0xfc, 0x07, 0xff, 0xf0, 0x1f, 0x00, 0x3f, 0xff, 0xfc, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xff, 0xfe, 0x03, 0xc1, 0xf8, 0x0f, 0x80, 0x3f, 0xff, 0xf8, 0x0f, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xff, 0xff, 0x01, 0xe0, 0x00, 0x07, 0xc0, 0x3f, 0xff, 0xf0, 0x1f, 0xff, 0xff,
		0xff, 0xff, 0xc1, 0xff, 0xff, 0x80, 0xf0, 0x00, 0x03, 0xe0, 0x3f, 0xff, 0xe0, 0x3f, 0xff, 0xff,
		0xff, 0xff, 0xc0, 0xff, 0xff, 0xc0, 0x78, 0x00, 0x01, 0xf0, 0x3f, 0xff, 0xc0, 0x7f, 0xff, 0xff,
		0xff, 0xff, 0xc0, 0x7f, 0xff, 0xe0, 0x3c, 0x00, 0x00, 0xf8, 0x3f, 0xff, 0x80, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xe0, 0x3f, 0xff, 0xf0, 0x1e, 0x00, 0x00, 0x7c, 0x3f, 0xff, 0x01, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xf0, 0x1f, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x03, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xf8, 0x0f, 0xff, 0xfc, 0x07, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xfc, 0x07, 0xff, 0xfe, 0x03, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xfe, 0x03, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x3f, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0x80, 0xc0, 0x00, 0x00, 0x7f, 0xf0, 0x00, 0x00, 0xc0, 0x70, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xc0, 0x60, 0x00, 0x00, 0x3f, 0xe0, 0x00, 0x01, 0x80, 0xf0, 0x77, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xe0, 0x30, 0x00, 0x00, 0x1f, 0xc0, 0x00, 0x03, 0x01, 0xf0, 0x77, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xf0, 0x18, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x06, 0x03, 0xf0, 0x77, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xf8, 0x0c, 0x00, 0x00, 0x07, 0x00, 0x00, 0x0c, 0x07, 0xf0, 0x07, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void ESP01DoCHPD(uint8_t value);

int ESP01WriteUSARTByte(uint8_t value);

void ESP01WriteByteToBufRX(uint8_t value);

void ESP01ChangeState(_eESP01STATUS esp01State);

void DecodeCMD(struct UNERBUSHandle *aBus, uint8_t iStartData);

void Do10ms();

void Do100ms();

void Do1000ms();

void USBReceive(uint8_t *buf, uint16_t len);

void Button_Task(_sButton *button);

void Communication_Task();

void Alive_Task();

void I2C_Tasks();

void modeTask();

void shortPress();

void longPress();

void advance();

void turning180();

void turning90(uint8_t side);

void advanceBlind();

void scanRoute();

void robotTask();
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
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&myADC.rawSamples, ADC_CHANNELS);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	Infrared_Filter(&myADC);

	if (TURNING == OFF)
		scanRoute();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		ESP01_WriteRX(dataRXESP01);
		HAL_UART_Receive_IT(&huart1, &dataRXESP01, 1);
	}
}

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
			UNERBUS_WriteByte(aBus, ACKNOWLEDGE);
			length = 2;
			break;
		case ANALOG_IR:
			UNERBUS_Write(aBus, myADC.millimeterSamples, ADC_CHANNELS);
			length = 9;
			break;
		case MPU_6050:
			UNERBUS_Write(aBus, myMPU.buffer, 14);
			length = 15;
			break;
		case DISPLAY_SSD1306:
			UPDATEDISPLAY = ON;
			break;
		case RECEIVE_N20:
			myMotor[LEFT].pow = (int8_t)(myMotor[LEFT].pulses/100);
			myMotor[RIGHT].pow = (int8_t)(myMotor[RIGHT].pulses/100);
			UNERBUS_WriteByte(aBus, (uint8_t)(myMotor[LEFT].pow));
			UNERBUS_WriteByte(aBus, (uint8_t)(myMotor[RIGHT].pow));
			length = 3;
			break;
		case SEND_N20:
			myMotor[LEFT].pow = UNERBUS_GetInt8(aBus);
			myMotor[RIGHT].pow = UNERBUS_GetInt8(aBus);
			Set_Power_Motor(&htim4, &myMotor[LEFT], &myMotor[RIGHT], myMotor[LEFT].pow, myMotor[RIGHT].pow);
			break;
		case BUTTONS:
			UNERBUS_WriteByte(aBus, robotMode);
			length = 2;
			break;
		case SET_PID_NAV:
			PID_Navigation.Kp = UNERBUS_GetUInt16(aBus);
			PID_Navigation.Ki = UNERBUS_GetUInt16(aBus);
			PID_Navigation.Kd = UNERBUS_GetUInt16(aBus);
			PID_Navigation.output = UNERBUS_GetUInt8(aBus);
			PID_Navigation.base = UNERBUS_GetUInt8(aBus);
			break;
		case SET_PID_TURN:
			PID_Turn.Kp = UNERBUS_GetUInt16(aBus);
			PID_Turn.Ki = UNERBUS_GetUInt16(aBus);
			PID_Turn.Kd = UNERBUS_GetUInt16(aBus);
			PID_Turn.output = UNERBUS_GetUInt8(aBus);
			PID_Turn.base = UNERBUS_GetUInt8(aBus);
			break;
		case SET_PID_U_TURN:
			PID_U_Turn.Kp = UNERBUS_GetUInt16(aBus);
			PID_U_Turn.Ki = UNERBUS_GetUInt16(aBus);
			PID_U_Turn.Kd = UNERBUS_GetUInt16(aBus);
			PID_U_Turn.output = UNERBUS_GetUInt8(aBus);
			PID_U_Turn.base = UNERBUS_GetUInt8(aBus);
			break;
		case SET_IR_THRESHOLD:
			for (uint8_t i=0; i<ADC_CHANNELS; i++)
				myADC.threshold[i] = UNERBUS_GetUInt8(aBus);
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
	MPUENABLED = ON;

	if (time100ms)
		time100ms--;

	if (time1000ms)
		time1000ms--;

	if (timeOutButton)
		timeOutButton--;

	Alive_Task();

	Button_Task(&myButton);

	ESP01_Timeout10ms();

	UNERBUS_Timeout(&unerbusESP01);

	UNERBUS_Timeout(&unerbusPC);

	Infrared_Convert(&myADC);

	modeTask();

	robotTask();
}

void Do100ms(){
	time100ms = 10;

	if (maskHB & myHB)
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // ON
	else
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); // OFF

	maskHB >>= 1;
	if (!maskHB)
		maskHB = HEARTBEAT_MASK;

	if (timeOutAlive)
		timeOutAlive--;
}

void Do1000ms(){
	time1000ms = 100;
	//int16_t minFrontDist = (IR_FRONT_1 < IR_FRONT_2) ? IR_FRONT_1 : IR_FRONT_2;

	Display_SetCursor(5, 5);
	sprintf(strAux, "POWL %.2d POWR %.2d", myMotor[LEFT].pow, myMotor[RIGHT].pow);
	Display_WriteString(strAux, Font_7x10, White);

	Display_SetCursor(5, 17);
	sprintf(strAux, "pid %.3d, ir1 %4d %d", testds, myADC.filteredSamples[FRONT_RIGHT], IR_DIAG_DER);
	Display_WriteString(strAux, Font_7x10, White);

	Display_SetCursor(5, 29);
	sprintf(strAux, "I%d DI%d F%d DD%d D%d", PARED_IZQUIERDA, DIAG_IZQUIERDA, PARED_DELANTERA, DIAG_DERECHA, PARED_DERECHA);
	Display_WriteString(strAux, Font_7x10, White);

	Display_SetCursor(5, 41);
	sprintf(strAux, "RM%d ACT%d Y %.3d", robotMode, robotAction, myMPU.Yaw);
	Display_WriteString(strAux, Font_7x10, White);

	UPDATEDISPLAY = ON;
}

void USBReceive(uint8_t *buf, uint16_t len){
	UNERBUS_ReceiveBuf(&unerbusPC, buf, len);
}

void Button_Task(_sButton *button){
	if (!timeOutButton){
		timeOutButton = TIMEOUT_BUTTON;
		BUTTON_Update(&myButton);
	}
}

void Communication_Task(){
	if(unerbusESP01.tx.iRead != unerbusESP01.tx.iWrite){
		w.u8[0] = unerbusESP01.tx.iWrite - unerbusESP01.tx.iRead;
		w.u8[0] &= unerbusESP01.tx.maxIndexRingBuf;
		if(ESP01_Send(unerbusESP01.tx.buf, unerbusESP01.tx.iRead, w.u8[0], unerbusESP01.tx.maxIndexRingBuf+1) == ESP01_SEND_READY){
			unerbusESP01.tx.iRead = unerbusESP01.tx.iWrite;
			myDisplay.via = 1;
		}
	}

	if(unerbusPC.tx.iRead != unerbusPC.tx.iWrite){
		if(unerbusPC.tx.iRead < unerbusPC.tx.iWrite)
			w.u8[0] = unerbusPC.tx.iWrite - unerbusPC.tx.iRead;
		else
			w.u8[0] = unerbusPC.tx.maxIndexRingBuf+1 - unerbusPC.tx.iRead;

		if(CDC_Transmit_FS(&unerbusPC.tx.buf[unerbusPC.tx.iRead], w.u8[0]) == USBD_OK){
			unerbusPC.tx.iRead += w.u8[0];
			unerbusPC.tx.iRead &= unerbusPC.tx.maxIndexRingBuf;
			myDisplay.via = 0;
		}
	}
}

void Alive_Task(){
	if (!timeOutAlive){
		UNERBUS_WriteByte(&unerbusESP01, ACKNOWLEDGE);
		UNERBUS_Send(&unerbusESP01, ALIVE, 2);
		UNERBUS_WriteByte(&unerbusPC, ACKNOWLEDGE);
		UNERBUS_Send(&unerbusPC, ALIVE, 2);
		timeOutAlive = TIMEOUT_ALIVE;
	}
}

void I2C_Tasks(){
	if (!time1000ms){
		time1000ms = 100;
	}

	if ((HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) && UPDATEDISPLAY){
		UPDATEDISPLAY = Display_UpdateScreen(&hi2c2);
	}

	if ((HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) && MPUENABLED){
		MPU6050_ReadAll(&hi2c2);
		MPU6050_GetYaw(&hi2c2);
		MPUENABLED = OFF;
	}
}

void modeTask(){
	switch (mode){
		case modeIDLE:
			myHB = HEARTBEAT_IDLE;
			robotMode = MODE_IDLE;
			robotAction = ACTION_STANDBY;
			break;
		case modeONE:
			myHB = HEARTBEAT_MODE1;
			robotMode = MODE_EXPLORE_MAZE;
			break;
		case modeTWO:
			myHB = HEARTBEAT_MODE2;
			robotMode = MODE_SOLVE_MAZE;
			break;
		default:
			break;
	}

	switch (robotMode){
		case MODE_EXPLORE_MAZE:
			if (MODESTARTED == OFF){
				robotAction = ACTION_STANDBY;
			}else{
				if (PARED_DELANTERA == OFF){
					robotAction = ACTION_AVANZAR;
					/*
					if ((PISO_ATRAS == ON)&&(!PARED_IZQUIERDA || !PARED_DERECHA)&&(TURNING == OFF)){
						if ((PARED_IZQUIERDA == ON && PARED_DERECHA == OFF)){
							robotAction = ACTION_CRUCE_L;
							PID_Reset(&PID_Turn);
						} else if ((PARED_IZQUIERDA == OFF && PARED_DERECHA == ON)){
							robotAction = ACTION_CRUCE_J;
							PID_Reset(&PID_Turn);
						}
						TURNING = ON;
						TURNSMOOTH = ON;
					} else {
						robotAction = ACTION_AVANZAR;
					}
					*/
				} else if ((PARED_DELANTERA == ON)&&(TURNING == OFF)){
					if (PARED_IZQUIERDA && !PARED_DERECHA){
						robotAction = ACTION_GIRO_DERECHA;
						PID_Reset(&PID_Turn);
					} else if (!PARED_IZQUIERDA && PARED_DERECHA){
						robotAction = ACTION_GIRO_IZQUIERDA;
						PID_Reset(&PID_Turn);
					} else if (PARED_IZQUIERDA && PARED_DERECHA){
						robotAction = ACTION_GIRO_EN_U;
						PID_Reset(&PID_U_Turn);
					} else if (!PARED_IZQUIERDA && !PARED_DERECHA){
						robotAction = ACTION_CRUCE_T;
						PID_Reset(&PID_Turn);
					}
					MPU6050_ResetYaw(&hi2c2);
					TURNING = ON;
					TURNSMOOTH = OFF;
				}
			}
			break;
		case MODE_SOLVE_MAZE:
			if (MODESTARTED == OFF){
				robotAction = ACTION_STANDBY;
				MPU6050_ResetYaw(&hi2c2);
			}else{
				robotAction = ACTION_CRUCE_X; // SACAR, SOLO PARA TEST
				if (abs(myMPU.Yaw) < 90){
					myMotor[LEFT].pow = 30;
					myMotor[RIGHT].pow = -30;
				} else {
					myMotor[LEFT].pow = 0;
					myMotor[RIGHT].pow = 0;
				}
				Set_Power_Motor(&htim4, &myMotor[LEFT], &myMotor[RIGHT], myMotor[LEFT].pow, myMotor[RIGHT].pow);
			}
			break;
		default:
			break;
	}
}

void shortPress(){
	mode++;
	MODESTARTED = OFF, TURNING = OFF;
	CHOSESIDE = OFF; TURNRIGHT = OFF; TURNSMOOTH = OFF;

	if (mode >= maxMODES)
		mode = modeIDLE;

	UNERBUS_WriteByte(&unerbusESP01, robotMode);
	UNERBUS_Send(&unerbusESP01, BUTTONS, 2);
	UNERBUS_WriteByte(&unerbusPC, robotMode);
	UNERBUS_Send(&unerbusPC, BUTTONS, 2);
}

void longPress(){
	if (robotMode != MODE_IDLE)
		MODESTARTED = ON;
}

void advance(){
	int32_t errorWall = 0;
	int8_t correction = 0;
	const int8_t DIST_WALL_REF = 50; // 50 mm DE DISTANCIA REFERENCIA HACIA LA PARED
	const int8_t WALL_DEADZONE = 5; // 5 mm COMO DISTANCIA MINIMA PARA LA CORRECCION
	int8_t currentBaseSpeed, minSpeed;

	int16_t minFrontDist = (IR_FRONT_1 < IR_FRONT_2) ? IR_FRONT_1 : IR_FRONT_2;

	if (PARED_IZQUIERDA && PARED_DERECHA){
		// CASO 1: PARED IZQUIERDA Y DERECHA, EL ERROR ES LA DIFERENCIA ENTRE LOS SENSORES LATERALES
		errorWall = IR_DER - IR_IZQ;
	}
	else if ((PARED_IZQUIERDA && DIAG_IZQUIERDA) && (!DIAG_DERECHA || !PARED_DERECHA)){
		// CASO 2: PARED IZQUIERDA PRESENTE, PARED DERECHA AUSENTE: PARED IZQUIERDA COMO REFERENCIA
		errorWall = (DIST_WALL_REF - IR_IZQ)*2; //(50-45)*2 = +10 -> corrige hacia la derecha
	}
	else if ((!DIAG_IZQUIERDA || !PARED_IZQUIERDA) && (PARED_DERECHA && DIAG_DERECHA)){
		// CASO 3: PARED IZQUIERDA AUSENTE, PARED DERECHA PRESENTE: PARED DERECHA COMO REFERENCIA
		errorWall = (IR_DER - DIST_WALL_REF)*2;
	} else if (!PARED_IZQUIERDA && !PARED_DERECHA){
		// CASO 4: PAREDES AUSENTES A LOS COSTADOS, ORIENTACION PREVIA COMO REFERENCIA
		errorWall = 0;
		//errorWall = IR_DIAG_DER - IR_DIAG_IZQ; // ESTO HACE QUE GIRE SUAVE
	}

	if (abs(errorWall) >= WALL_DEADZONE){
		// LA DIFERENCIA ENTRE LA MEDICION Y LA CORRECCION ES SIGNIFICANTE, HAY CORRECCION PID
		correction = PID_Compute(&PID_Navigation, errorWall);
		// Si correction es mayor a 0, el robot tiende a la derecha
	}
	else{
		// LA DIFERENCIA ENTRE LA MEDICION Y LA CORRECCION NO ES SIGNIFICANTE, NO HAY CORRECCION PID
		PID_Reset(&PID_Navigation);
		correction = 0;
	}

	testds = correction;

	if (minFrontDist <= 100){
		minSpeed = PID_Navigation.base * (50/100);
		if (minFrontDist <= myADC.threshold[FRONT_1]){
			// Potencia al minimo si esta por debajo del threshold
			currentBaseSpeed = minSpeed;
		} else {
			// Ecuación: Vel = VelMin + (VelMax - VelMin) * (Dist - Umbral) / (100 - Umbral)
			currentBaseSpeed = minSpeed + ((PID_Navigation.base - minSpeed) * (minFrontDist - myADC.threshold[FRONT_1])) / (100 - myADC.threshold[FRONT_1]);
		}
		myMotor[LEFT].pow = currentBaseSpeed + correction;
		myMotor[RIGHT].pow = currentBaseSpeed - correction;
	} else {
		myMotor[LEFT].pow = PID_Navigation.base + correction;
		myMotor[RIGHT].pow = PID_Navigation.base - correction;
	}

	Set_Power_Motor(&htim4, &myMotor[LEFT], &myMotor[RIGHT], myMotor[LEFT].pow, myMotor[RIGHT].pow);
	MPU6050_ResetYaw(&hi2c2);
}

void turning180(){
	int32_t errorAng = 0;
	int8_t correction = 0;
	const int8_t ANG_DEADZONE = 2; // 2 dps COMO DIFERENCIA MINIMA PARA LA CORRECCION DE LA VELOCIDAD
	static uint8_t brake_counter = 0; // NUEVO: Contador para el freno
	//int16_t minFrontDist = (IR_FRONT_1 < IR_FRONT_2) ? IR_FRONT_1 : IR_FRONT_2;

	// Si estamos en fase de frenado, esperar a que el chasis se detenga
	if (brake_counter > 0) {
		brake_counter++;
		if (brake_counter > 4) { // Esperar 5 ciclos = 50 milisegundos
			brake_counter = 0;

			PID_Reset(&PID_Navigation);
			robotAction = ACTION_CRUCE_CIEGO; // AHORA S�?, salir ciego hacia adelante
		}
		return; // Terminar la función aquí para no hacer nada más
	}
	if (CHOSESIDE == OFF){
		if (IR_IZQ <= IR_DER)
			TURNRIGHT = ON;
		else
			TURNRIGHT = OFF;
		CHOSESIDE = ON;
	}

	if (TURNRIGHT == ON)
		errorAng = 180 + myMPU.Yaw;
	else
		errorAng = 180 - myMPU.Yaw;

	if (abs(errorAng) >= ANG_DEADZONE){
		// LA DIFERENCIA ENTRE LA MEDICION Y LA CORRECCION ES SIGNIFICANTE, HAY CORRECCION PID
		correction = PID_Compute(&PID_U_Turn, errorAng);
	} else {
		// LA DIFERENCIA ENTRE LA MEDICION Y LA CORRECCION NO ES SIGNIFICANTE, NO HAY CORRECCION PID
		PID_Reset(&PID_U_Turn);
		correction = 0;
	}

	testds = correction;

	if (abs(myMPU.Yaw) < 180) {
		if (TURNRIGHT == ON){
			myMotor[LEFT].pow = PID_U_Turn.base + correction;
			myMotor[RIGHT].pow = -PID_U_Turn.base - correction;
		} else {
			myMotor[LEFT].pow = -PID_U_Turn.base + correction;
			myMotor[RIGHT].pow = PID_U_Turn.base - correction;
		}
		Set_Power_Motor(&htim4, &myMotor[LEFT], &myMotor[RIGHT], myMotor[LEFT].pow, myMotor[RIGHT].pow);
		if ((abs(myMPU.Yaw) > 150) && (IR_DER <= 50 && IR_IZQ <= 50))
			myMPU.Yaw = 180;
	} else {
		//PID_Reset(&PID_Navigation);
		//robotAction = ACTION_CRUCE_CIEGO; // EMPALME CON NUEVO CAMINO
		brake_counter = 1; // INICIAR SECUENCIA DE FRENADO

		myMotor[LEFT].pow = 0;
		myMotor[RIGHT].pow = 0;
		Set_Power_Motor(&htim4, &myMotor[LEFT], &myMotor[RIGHT], 0, 0);
	}
}

void turning90(uint8_t side){
	int32_t errorAng = 0;
	int8_t correction = 0;
	const int8_t ANG_DEADZONE = 2; // 5 dps COMO DIFERENCIA MINIMA PARA LA CORRECCION DE LA VELOCIDAD
	static uint8_t brake_counter = 0; // NUEVO: Contador para el freno
	//uint8_t PARED_IZQ = (IR_IZQ < myADC.threshold[LEFT_SENSOR]);
	//uint8_t PARED_DER = (IR_DER < myADC.threshold[RIGHT_SENSOR]);

	// Si estamos en fase de frenado, esperar a que el chasis se detenga
	if (brake_counter > 0) {
		brake_counter++;
		if (brake_counter > 4) { // Esperar 5 ciclos = 50 milisegundos
			brake_counter = 0;

			PID_Reset(&PID_Navigation);
			robotAction = ACTION_CRUCE_CIEGO; // AHORA S�?, salir ciego hacia adelante
		}
		return; // Terminar la función aquí para no hacer nada más
	}

	if (side == LEFT_SIDE)
		errorAng = 90 - myMPU.Yaw;
	else if (side == RIGHT_SIDE)
		errorAng = 90 + myMPU.Yaw;

	if (abs(errorAng) >= ANG_DEADZONE){
		// LA DIFERENCIA ENTRE LA MEDICION Y LA CORRECCION ES SIGNIFICANTE, HAY CORRECCION PID
		correction = PID_Compute(&PID_Turn, errorAng);
	} else {
		// LA DIFERENCIA ENTRE LA MEDICION Y LA CORRECCION NO ES SIGNIFICANTE, NO HAY CORRECCION PID
		PID_Reset(&PID_Turn);
		correction = 0;
	}

	testds = correction;

	if (abs(myMPU.Yaw) < 90){
		if (side == LEFT_SIDE){ // logica inversa
			myMotor[LEFT].pow = -PID_Turn.base + correction;
			myMotor[RIGHT].pow = PID_Turn.base - correction;
			if (IR_IZQ > 100 && IR_DER >= 45 && IR_FRONT_1 > 100 && abs(myMPU.Yaw) > 65)
				myMPU.Yaw = 90;
		} else if (side == RIGHT_SIDE){
			myMotor[LEFT].pow = PID_Turn.base + correction;
			myMotor[RIGHT].pow = -PID_Turn.base - correction;
			if (IR_IZQ >= 45 && IR_DER > 100 && IR_FRONT_2 > 100 && abs(myMPU.Yaw) > 65)
				myMPU.Yaw = -90;
		}
		Set_Power_Motor(&htim4, &myMotor[LEFT], &myMotor[RIGHT], myMotor[LEFT].pow, myMotor[RIGHT].pow);
	} else {
		brake_counter = 1; // INICIAR SECUENCIA DE FRENADO
		myMotor[LEFT].pow = 0;
		myMotor[RIGHT].pow = 0;
		Set_Power_Motor(&htim4, &myMotor[LEFT], &myMotor[RIGHT], 0, 0);
	}
}

void advanceBlind(){
	int32_t errorWall = 0;
	int8_t correction = 0;
	const int8_t DIST_WALL_REF = 50; // 50 mm DE DISTANCIA REFERENCIA HACIA LA PARED
	const int8_t WALL_DEADZONE = 5; // 5 mm COMO DISTANCIA MINIMA PARA LA CORRECCION
	static uint8_t counterIR7 = 0;
	uint8_t PARED_IZQ = (IR_IZQ <= myADC.threshold[LEFT_SIDE]);
	uint8_t PARED_DER = (IR_DER <= myADC.threshold[RIGHT_SIDE]);
	uint8_t DIAG_IZQ = (IR_DIAG_IZQ <= myADC.threshold[FRONT_LEFT]);
	uint8_t DIAG_DER = (IR_DIAG_DER <= myADC.threshold[FRONT_RIGHT]);

	if (IR_GROUND_BACK == myADC.threshold[GROUND_BACK]){
		counterIR7++;
		if (counterIR7 == 2){
			counterIR7 = 0;
			TURNING = OFF;
			PID_Reset(&PID_Navigation);
			robotAction = ACTION_AVANZAR;
			scanRoute();
		}
	} else {
		counterIR7 = 0;
	}

	if ((DIAG_IZQ || DIAG_DER)&&(PARED_IZQ && PARED_DER)){
		// Caso 1: paredes presentes, al menos una diagonal presente
		errorWall = IR_DER - IR_IZQ;
	} else if ((DIAG_IZQ)&&(PARED_IZQ && !PARED_DER)){
		// Caso 2: pared derecha ausente, tengo pared a la izquierda
		errorWall = (DIST_WALL_REF - IR_IZQ)*2;
	} else if ((DIAG_DER)&&(!PARED_IZQ && PARED_DER)){
		// Caso 3: pared izquierda ausente, tengo pared a la izquierda
		errorWall = (IR_DER - DIST_WALL_REF)*2;
	} else if ((!PARED_IZQ && !PARED_DER)){
		// Caso 4: paredes ausentes, sin referencia de las diagonales
		errorWall = 0;
	}


	if (abs(errorWall) >= WALL_DEADZONE){
		// LA DIFERENCIA ENTRE LA MEDICION Y LA CORRECCION ES SIGNIFICANTE, HAY CORRECCION PID
		correction = PID_Compute(&PID_Navigation, errorWall);
	}
	else{
		// LA DIFERENCIA ENTRE LA MEDICION Y LA CORRECCION NO ES SIGNIFICANTE, NO HAY CORRECCION PID
		PID_Reset(&PID_Navigation);
		correction = 0;
	}
	myMotor[LEFT].pow = PID_Navigation.base + correction;//myMotor[LEFT].base + correction;
	myMotor[RIGHT].pow = PID_Navigation.base - correction;//myMotor[RIGHT].base - correction;
	Set_Power_Motor(&htim4, &myMotor[LEFT], &myMotor[RIGHT], myMotor[LEFT].pow, myMotor[RIGHT].pow);
}

void scanRoute(){
	if (IR_DER <= myADC.threshold[RIGHT_SIDE])	// PARED DERECHA
		PARED_DERECHA = ON;
	else
		PARED_DERECHA = OFF;

	if (IR_IZQ <= myADC.threshold[LEFT_SIDE])	// PARED IZQUIERDA
		PARED_IZQUIERDA = ON;
	else
		PARED_IZQUIERDA = OFF;

	if (IR_DIAG_DER <= myADC.threshold[FRONT_RIGHT])	// DIAGONAL DERECHA
		DIAG_DERECHA = ON;
	else
		DIAG_DERECHA = OFF;

	if (IR_DIAG_IZQ <= myADC.threshold[FRONT_LEFT])	// DIAGONAL IZQUIERDA
		DIAG_IZQUIERDA = ON;
	else
		DIAG_IZQUIERDA = OFF;

	if (IR_FRONT_1 < myADC.threshold[FRONT_1] || IR_FRONT_2 <= myADC.threshold[FRONT_2])	// PARED DELANTERA
		PARED_DELANTERA = ON;
	else
		PARED_DELANTERA = OFF;

	if (IR_GROUND_FRONT == myADC.threshold[GROUND_FRONT])	// PISO DELANTERO
		PISO_ADELANTE = ON;
	else
		PISO_ADELANTE = OFF;

	if (IR_GROUND_BACK == myADC.threshold[GROUND_BACK])	// PISO TRASERO
		PISO_ATRAS = ON;
	else
		PISO_ATRAS = OFF;
}

void robotTask(){
	switch (robotAction){
		case ACTION_STANDBY:
			myMotor[LEFT].pow = 0;
			myMotor[RIGHT].pow = 0;
			Set_Power_Motor(&htim4, &myMotor[LEFT], &myMotor[RIGHT], myMotor[LEFT].pow, myMotor[RIGHT].pow);
			break;
		case ACTION_AVANZAR:
			advance();
			break;
		case ACTION_GIRO_DERECHA:
			turning90(RIGHT_SIDE);
			break;
		case ACTION_GIRO_IZQUIERDA:
			turning90(LEFT_SIDE);
			break;
		case ACTION_GIRO_EN_U:
			turning180();
			break;
		case ACTION_CRUCE_CIEGO:
			advanceBlind();
			break;
		case ACTION_CRUCE_T:
			static uint8_t toggleBit = 0;
			toggleBit ^= 1; // XOR con 1 -> se invierte cada ciclo
			if (toggleBit == ON){ // SI ES 1, GIRA A LA IZQUIERDA
				robotAction = ACTION_GIRO_IZQUIERDA;
			} else {	// SI ES 0, GIRA A LA DERECHA
				robotAction = ACTION_GIRO_DERECHA;
			}
			MPU6050_ResetYaw(&hi2c2);
			break;
		case ACTION_CRUCE_L:

			MPU6050_ResetYaw(&hi2c2);
			break;
		case ACTION_CRUCE_J:

			MPU6050_ResetYaw(&hi2c2);
			break;
		default:
			break;
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
  BUTTON_Init(&myButton, SW0_GPIO_Port, SW0_Pin);

  BUTTON_Register_PressReleasedCallback(&myButton, shortPress);

  BUTTON_Register_LongPressCallback(&myButton, longPress);

  Infrared_Init(&myADC);

  PID_Init(&PID_Navigation, 50, 0, 500, 20, 35); // VELOCIDAD MAX +/-55%, VELOCIDAD BASE 35% (50/0/80)
  PID_Init(&PID_Turn, 1, 0, 5, 15, 30); // VELOCIDAD MAX +/-35%, VELOCIDAD BASE 25% (1/2/2)
  PID_Init(&PID_U_Turn, 1, 0, 5, 15, 30); // VELOCIDAD MAX +/-35%, VELOCIDAD BASE 25% (1/3/2)
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
  MX_TIM4_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

  flag1.byte = OFF;

  Motors_Init(&myMotor[LEFT], &myMotor[RIGHT]);

  CDC_AttachRxData(USBReceive);

  HAL_UART_Receive_IT(&huart1, &dataRXESP01, 1);

  MPU6050_Init(&hi2c2);
  MPU6050_Calibrate(&hi2c2);

  Display_Init(&hi2c2);
  Display_Fill(Black);
  //Display_DrawBitmap(Display_WIDTH, Display_HEIGHT, myDesignBKG);
  UPDATEDISPLAY = ON;

  robotMode = MODE_IDLE;
  robotAction = ACTION_STANDBY;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (ON10MS)
		  Do10ms();

	  if (!time100ms)
		  Do100ms();

	  if (!time1000ms)
		  Do1000ms();

	  Communication_Task();

	  ESP01_Task();

	  UNERBUS_Task(&unerbusESP01);

	  UNERBUS_Task(&unerbusPC);

	  I2C_Tasks();
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
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
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
  htim1.Init.Period = 249;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
