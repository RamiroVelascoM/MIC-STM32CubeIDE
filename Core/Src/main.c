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
#include "stdlib.h"
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
	CONTROL_MODE = 0xF8,
	MAZE_DATA = 0xF9,
	MAZE_POSITION = 0xFA,

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
	ACTION_CRUCE_X = 10,
	ACTION_ALIGN_WALL = 11
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

typedef struct{
	int8_t		posIniX;
	int8_t 		posIniY;
	int8_t		posActX;
	int8_t 		posActY;
	int8_t		posFinX;
	int8_t 		posFinY;
	int8_t		dir;
}_sMaze;
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
#define DISTANCE_IRS			85

#define TIMEOUT_BUTTON			4
#define TIMEOUT_ALIVE			50
#define MAX_MENUS_DISPLAY		3

#define NORTE					0
#define	SUR						1
#define ESTE					2
#define OESTE					3

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
#define WIFI_UDP_REMOTE_IP		"172.23.207.122"//"192.168.1.3"//
#define WIFI_UDP_LOCAL_PORT		30000
#define WIFI_UDP_REMOTE_PORT	30030

#define ON10MS					flag1.bit.b0
#define MPUENABLED				flag1.bit.b1
#define MODESTARTED				flag1.bit.b2
#define TURNING					flag1.bit.b3
#define CHOSESIDE				flag1.bit.b4
#define TURNRIGHT				flag1.bit.b5
#define NEWCELL					flag1.bit.b6
#define UPDATEDISPLAY			flag1.bit.b7

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
_sMaze myMaze;

_eACTIONS robotAction;
_eMODES robotMode;
/**
 * DEFINICION DE DATOS DE COMUNICACION
 */
uint8_t bufRXPC[SIZEBUFRXPC], bufTXPC[SIZEBUFTXPC];
uint8_t bufRXESP01[SIZEBUFRXESP01], bufTXESP01[SIZEBUFTXESP01], dataRXESP01;
uint8_t rxUSBData, newData;
uint8_t menuDisplay = 1;//, UPDATEDISPLAY = 0;
int8_t testds = 0, testds2 = 0;
/**
 * DEFINICION DE DATOS DE MANEJO DE PROGRAMA
 */
uint8_t mode			= 0;
uint8_t time10ms 		= 40;
uint8_t time100ms		= 10;
//uint8_t time500ms		= 50;
uint8_t time1000ms		= 100;
uint8_t	timeOutButton	= TIMEOUT_BUTTON;
uint8_t timeOutAlive 	= TIMEOUT_ALIVE;
uint32_t myHB			= HEARTBEAT_IDLE;
uint32_t maskHB			= HEARTBEAT_MASK;
uint8_t timeCounter		= 0xFF;
uint8_t resetYaw 		= ON;
uint16_t DPS_Target 	= 240;

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

void turn180();

void scanRoute();

void robotTask();

void handlePath();

void turn90(uint8_t side);

void displayTask();

void handlePosition();

void handleDirection();

void funcTest();
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
			myDisplay.hs = UNERBUS_GetUInt8(aBus);
			myDisplay.min = UNERBUS_GetUInt8(aBus);
			UPDATEDISPLAY = ON;
			break;
		case RECEIVE_N20:
			//myMotor[LEFT].pow = (int8_t)(myMotor[LEFT].pulses/100);
			//myMotor[RIGHT].pow = (int8_t)(myMotor[RIGHT].pulses/100);
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
		case CONTROL_MODE:
			MODESTARTED = UNERBUS_GetUInt8(aBus);
			mode = UNERBUS_GetUInt8(aBus);
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
			//PID_Turn.base = UNERBUS_GetUInt8(aBus);
			PID_Turn.velExt = UNERBUS_GetUInt8(aBus);
			PID_Turn.velInt = UNERBUS_GetUInt8(aBus);
			DPS_Target = UNERBUS_GetUInt16(aBus);
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
			myMPU.thresholdGyro = UNERBUS_GetUInt16(aBus);
			break;
		case MAZE_DATA:
			myMaze.posIniX = UNERBUS_GetInt8(aBus);
			myMaze.posIniY = UNERBUS_GetInt8(aBus);
			myMaze.posActX = myMaze.posIniX;
			myMaze.posActY = myMaze.posIniY;
			myMaze.posFinX = UNERBUS_GetInt8(aBus);
			myMaze.posFinY = UNERBUS_GetInt8(aBus);
			myMaze.dir = UNERBUS_GetInt8(aBus);
			break;
		case MAZE_POSITION:
			UNERBUS_WriteByte(aBus, (uint8_t)(myMaze.posActX));
			UNERBUS_WriteByte(aBus, (uint8_t)(myMaze.posActY));
			UNERBUS_WriteByte(aBus, (uint8_t)(myMaze.dir));
			length = 4;
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

	scanRoute();

	modeTask();

	robotTask();

	handlePosition();
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

	srand(HAL_GetTick());

	displayTask();
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
		//UNERBUS_WriteByte(&unerbusPC, ACKNOWLEDGE);
		//UNERBUS_Send(&unerbusPC, ALIVE, 2);
		timeOutAlive = TIMEOUT_ALIVE;
	}
}

void I2C_Tasks(){
	if (!time1000ms){
		time1000ms = 100;
	}

	if ((HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) && MPUENABLED){
		MPU6050_ReadAll(&hi2c2);
		MPUENABLED = OFF;
		return;
	}

	if ((HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) && UPDATEDISPLAY && time10ms >= 4){
		// Si time10ms es mayor a 4, segun la configuracion del timer1, entonces habran transcurrido 9 de los 10 ms configurados
		UPDATEDISPLAY = Display_UpdateScreen(&hi2c2);
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
				handlePath();
			}
			break;
		case MODE_SOLVE_MAZE:
			if (MODESTARTED == OFF){
				robotAction = ACTION_STANDBY;
				MPU6050_ResetYaw(&hi2c2);
				myMaze.posActX = 0;
				myMaze.posActY = 0;
			}else{
				funcTest();
			}
			break;
		default:
			break;
	}
}

void shortPress(){
	mode++;
	MODESTARTED = OFF, TURNING = OFF;
	CHOSESIDE = OFF; TURNRIGHT = OFF; NEWCELL = OFF;

	if (mode >= maxMODES)
		mode = modeIDLE;

	UNERBUS_WriteByte(&unerbusESP01, mode+1);
	UNERBUS_Send(&unerbusESP01, BUTTONS, 2);
	//UNERBUS_WriteByte(&unerbusPC, mode+1);
	//UNERBUS_Send(&unerbusPC, BUTTONS, 2);
}

void longPress(){
	if (robotMode != MODE_IDLE){
		MODESTARTED ^= 1;
	} else {
		menuDisplay++;
		if (menuDisplay >= MAX_MENUS_DISPLAY)
			menuDisplay = 0;
	}
}

void advance(){
	int32_t errorWall = 0;
	int8_t correction = 0;

	const int8_t DIST_WALL_REF = 50; // 50 mm DE DISTANCIA REFERENCIA HACIA LA PARED
	const int8_t WALL_DEADZONE = 1; // 1 mm COMO DISTANCIA MINIMA PARA LA CORRECCION
	/*
	const uint8_t FRONT_THRESHOLD = 50; // 50 mm de UMBRAL DELANTERO
	const uint8_t BREAKING_ZONE = 60; // 55 mm de ZONA DE FRENADO HACIA EL UMBRAL
	int8_t currentBaseSpeed, minSpeed;
	int16_t minFrontDist = (IR_FRONT_1 < IR_FRONT_2) ? IR_FRONT_1 : IR_FRONT_2;
	*/
	static uint8_t resetYaw = 0;

	if (timeCounter < 0xFF && (PARED_IZQUIERDA || PARED_DERECHA)){
		timeCounter++;
	}

	if (PISO_ATRAS == ON && timeCounter != 0)
		timeCounter = 0;

	if (PARED_IZQUIERDA && PARED_DERECHA && DIAG_IZQUIERDA && DIAG_DERECHA){
		// CASO 1: PARED IZQUIERDA Y DERECHA, EL ERROR ES LA DIFERENCIA ENTRE LOS SENSORES LATERALES
		errorWall = IR_DER - IR_IZQ;
		resetYaw = OFF;
	} else if (PARED_IZQUIERDA && DIAG_IZQUIERDA){
		// CASO 2: PARED IZQUIERDA PRESENTE, PARED DERECHA AUSENTE: PARED IZQUIERDA COMO REFERENCIA
		errorWall = (DIST_WALL_REF - IR_IZQ)*2; //(50-45)*2 = +10 -> corrige hacia la derecha
		resetYaw = OFF;
	} else if (PARED_DERECHA && DIAG_DERECHA){
		// CASO 3: PARED IZQUIERDA AUSENTE, PARED DERECHA PRESENTE: PARED DERECHA COMO REFERENCIA
		errorWall = (IR_DER - DIST_WALL_REF)*2;
		resetYaw = OFF;
	}  else if (PARED_IZQUIERDA && timeCounter < 50){
		// CASO 2: PARED IZQUIERDA PRESENTE, PARED DERECHA AUSENTE: PARED IZQUIERDA COMO REFERENCIA
		errorWall = (DIST_WALL_REF - IR_IZQ)*2; //(50-45)*2 = +10 -> corrige hacia la derecha
		resetYaw = OFF;
	} else if (PARED_DERECHA && timeCounter < 50){
		// CASO 3: PARED IZQUIERDA AUSENTE, PARED DERECHA PRESENTE: PARED DERECHA COMO REFERENCIA
		errorWall = (IR_DER - DIST_WALL_REF)*2;
		resetYaw = OFF;
	} else {
		// CASO 4: PAREDES AUSENTES A LOS COSTADOS, ORIENTACION PREVIA COMO REFERENCIA
		if (resetYaw == OFF){
			MPU6050_ResetYaw(&hi2c2);
			resetYaw = ON;
		}
		errorWall = myMPU.Yaw;
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
	/*
	if (minFrontDist <= BREAKING_ZONE){
		minSpeed = PID_Navigation.base * 0.7;
		if (minFrontDist <= myADC.threshold[FRONT_1]){
			// Potencia al minimo si esta por debajo del threshold
			currentBaseSpeed = minSpeed;
		} else {
			// Ecuación: Vel = VelMin + (VelMax - VelMin) * (Dist - Umbral) / (Breaking Zone - Umbral)
			currentBaseSpeed = minSpeed + ((PID_Navigation.base - minSpeed) * (minFrontDist - FRONT_THRESHOLD)) / (BREAKING_ZONE - FRONT_THRESHOLD);
		}
		myMotor[LEFT].pow = currentBaseSpeed + correction;
		myMotor[RIGHT].pow = currentBaseSpeed - correction;
	} else {
		myMotor[LEFT].pow = PID_Navigation.base + correction;
		myMotor[RIGHT].pow = PID_Navigation.base - correction;
	}
	*/
	myMotor[LEFT].pow = PID_Navigation.base + correction;
	myMotor[RIGHT].pow = PID_Navigation.base - correction;
	Set_Power_Motor(&htim4, &myMotor[LEFT], &myMotor[RIGHT], myMotor[LEFT].pow, myMotor[RIGHT].pow);
}

void turn180(){
	int32_t errorAng = 0;
	int8_t correction = 0;
	const int8_t ANG_DEADZONE = 1; // 1 grado COMO DIFERENCIA MINIMA PARA LA CORRECCION DE LA VELOCIDAD
	static uint8_t brake_counter_180 = 0; // NUEVO: Contador para el freno

	// Si estamos en fase de frenado, esperar a que el chasis se detenga
	if (brake_counter_180 > 0) {
		brake_counter_180++;
		if (brake_counter_180 >= 1) { // Esperar 5 ciclos = 50 milisegundos
			brake_counter_180 = 0;
			TURNING = OFF; CHOSESIDE = OFF;
			MPU6050_ResetYaw(&hi2c2);
			PID_Reset(&PID_Navigation);
			robotAction = ACTION_AVANZAR;
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

	if ((abs(myMPU.Yaw) >= 177)||((abs(myMPU.Yaw) >= 160)&&(IR_IZQ <= 50 && IR_DER <= 55))) {
		brake_counter_180 = 1; // INICIAR SECUENCIA DE FRENADO
		myMotor[LEFT].pow = 0;
		myMotor[RIGHT].pow = 0;
		Set_Power_Motor(&htim4, &myMotor[LEFT], &myMotor[RIGHT], 0, 0);
		//if (brake_counter_180 == 1)
			//handleDirection();
	} else {
		if (TURNRIGHT == ON){
			myMotor[LEFT].pow = PID_U_Turn.base + correction;
			myMotor[RIGHT].pow = -PID_U_Turn.base - correction;
		} else {
			myMotor[LEFT].pow = -PID_U_Turn.base - correction;
			myMotor[RIGHT].pow = PID_U_Turn.base + correction;
		}
		Set_Power_Motor(&htim4, &myMotor[LEFT], &myMotor[RIGHT], myMotor[LEFT].pow, myMotor[RIGHT].pow);
	}
}

void turn90(uint8_t side){
	int32_t errorVel = 0;
	int8_t correction = 0;
	uint8_t DPS_DEADZONE = 1; // Un umbral de 1 dps
	//uint16_t DPS_TARGET = 250;
	//int32_t currentDPS = myMPU.GyrZ / GYRO_SENSITIVITY;
	static uint8_t brake_counter_90 = 0;

	// Si estamos en fase de frenado, esperar a que el chasis se detenga
	if (robotMode == MODE_EXPLORE_MAZE){
		if (brake_counter_90 > 0) {
			brake_counter_90++;
			if (brake_counter_90 >= 3) { // Esperar 3 ciclos = 30 milisegundos
				brake_counter_90 = 0;
				TURNING = OFF;
				MPU6050_ResetYaw(&hi2c2);
				PID_Reset(&PID_Navigation);
				robotAction = ACTION_AVANZAR;
				timeCounter = 0;
			}
			return; // Terminar la función aquí para no hacer nada más
		}
	} else if (robotMode == MODE_SOLVE_MAZE){
		if (brake_counter_90 > 0) {
			brake_counter_90++;
			if (brake_counter_90 >= 15) { // Esperar 15 ciclos = 150 milisegundos
				brake_counter_90 = 0;
				TURNING = OFF;
				timeCounter = 0;
				PID_Reset(&PID_Navigation);
				MODESTARTED = OFF;
			}
			return; // Terminar la función aquí para no hacer nada más
		}
	}

	// Si giramos a la izquierda, el giro es positivo. A la derecha, negativo.
	if (side == RIGHT_SIDE)
		errorVel = -(90 + myMPU.Yaw);//-DPS_Target - currentDPS; // Gira a la derecha, Gz disminuye
	else
		errorVel = 90 - myMPU.Yaw;//DPS_Target - currentDPS; // Gira a la izquierda, Gz aumenta

	if (abs(errorVel) >= DPS_DEADZONE){
		// LA DIFERENCIA ENTRE LA MEDICION Y LA CORRECCION ES SIGNIFICANTE, HAY CORRECCION PID
		correction = PID_Compute(&PID_Turn, errorVel);
	} else {
		// LA DIFERENCIA ENTRE LA MEDICION Y LA CORRECCION NO ES SIGNIFICANTE, NO HAY CORRECCION PID
		PID_Reset(&PID_Turn);
		correction = 0;
	}
	testds = correction;

	if ((abs(myMPU.Yaw) >= 85)||(brake_counter_90 > 0)){
		brake_counter_90 = 1; // INICIAR SECUENCIA DE FRENADO
		myMotor[LEFT].pow = 0;
		myMotor[RIGHT].pow = 0;
		Set_Power_Motor(&htim4, &myMotor[LEFT], &myMotor[RIGHT], 0, 0);
		//if (brake_counter_90 == 1)
			//handleDirection();
	} else {
		if (side == LEFT_SIDE){
			myMotor[LEFT].pow = PID_Turn.velInt - correction;
			myMotor[RIGHT].pow = PID_Turn.velExt + correction;
			if ((abs(myMPU.Yaw) >= 65)&&(IR_DIAG_IZQ <= 75 || IR_GROUND_FRONT == ON))
				brake_counter_90 = 1;
			if ((abs(myMPU.Yaw) >= 75)&&(IR_IZQ <= IR_DIAG_IZQ || IR_DER >= 50))
				brake_counter_90 = 1;
		} else if (side == RIGHT_SIDE){
			myMotor[LEFT].pow = PID_Turn.velExt - correction;
			myMotor[RIGHT].pow = PID_Turn.velInt + correction;
			if ((abs(myMPU.Yaw) >= 65)&&(IR_DIAG_DER <= 75 || IR_GROUND_FRONT == ON))
				brake_counter_90 = 1;
			if ((abs(myMPU.Yaw) >= 75)&&(IR_DER <= IR_DIAG_DER || IR_IZQ >= 50))
				brake_counter_90 = 1;
		}
		Set_Power_Motor(&htim4, &myMotor[LEFT], &myMotor[RIGHT], myMotor[LEFT].pow, myMotor[RIGHT].pow);
	}
}

void scanRoute(){
	// Variables estáticas para el debounce de la cinta (piso delantero)
	static uint8_t blackDebounce = 0;
	static uint8_t whiteDebounce = 0;

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

	if (IR_GROUND_BACK == myADC.threshold[GROUND_BACK])	// PISO TRASERO
		PISO_ATRAS = ON;
	else
		PISO_ATRAS = OFF;

	if ((IR_IZQ + IR_DER >= 135)&&(PARED_DELANTERA == OFF)){
		if (IR_IZQ >= IR_DER)
			PARED_IZQUIERDA = OFF;
		else
			PARED_DERECHA = OFF;
	}

	/**
	 * LÓGICA DE DETECCIÓN DE CELDA NUEVA (PISO ADELANTE)
	 */
	if (IR_GROUND_FRONT == myADC.threshold[GROUND_FRONT]) { // Sensor detecta NEGRO (Cinta)
		blackDebounce++;
		whiteDebounce = 0; // Reset del contador de blanco
		if (blackDebounce >= 2) {
			PISO_ADELANTE = ON;
		}
	} else { // Sensor detecta BLANCO (Piso)
		whiteDebounce++;
		blackDebounce = 0; // Reset del contador de negro
		if (whiteDebounce >= 2) {
			// Si veníamos de estar en NEGRO y ahora estamos seguros en BLANCO
			if (PISO_ADELANTE == ON) {
				//NEWCELL = ON; // Se activa la bandera de nueva celda al salir de la cinta
				handlePosition();
			}
			PISO_ADELANTE = OFF;
		}
	}
}

void handlePath(){
	int16_t minFrontDist = (IR_FRONT_1 < IR_FRONT_2) ? IR_FRONT_1 : IR_FRONT_2;
	uint8_t PARED_FRONTAL = (minFrontDist <= 55);
	testds2 = PARED_FRONTAL;

	// Variables estaticas para el debounce del camino
	static uint8_t previous_walls = 0xFF; // Inicializado en un valor irreal para forzar la primera lectura
	static uint8_t debounce_counter = 1;
	// --------------------------------------------------------------

	// Si ya estamos en una maniobra de giro, no buscar nuevos caminos
	if (TURNING == ON) {
		debounce_counter = 1;
		return;
	}

	// Empaquetamos las 3 paredes en un solo byte: [0 0 0 0 0 IZQ DER FRONT]
	uint8_t current_walls = (PARED_IZQUIERDA << 2) | (PARED_DERECHA << 1) | PARED_DELANTERA;

	if (current_walls == previous_walls) {
		if (debounce_counter < 0xFF) { // Prevenir desbordamiento
			debounce_counter++;
		}
	} else {
		// El camino cambió, reiniciamos el filtro
		previous_walls = current_walls;
		debounce_counter = 1;
	}

	// GIRO EN U
	if (PARED_FRONTAL == ON && PARED_IZQUIERDA == ON && PARED_DERECHA == ON){
		robotAction = ACTION_GIRO_EN_U;
		PID_Reset(&PID_U_Turn);
		MPU6050_ResetYaw(&hi2c2);
		TURNING = ON;
		return;
	}

	// Si no hemos visto el mismo escenario de paredes al menos 2 veces seguidas,
	// abortamos la toma de decisiones por este ciclo.
	if (debounce_counter < 2) {
		if (PARED_FRONTAL == OFF) {
			robotAction = ACTION_AVANZAR;
		}
		return;
	}

	// DETECCIÓN DE CINTA (Giros de 90 grados y Bifurcaciones)
	if (PISO_ATRAS == ON){
		if (PARED_DELANTERA == ON){
			if (PARED_IZQUIERDA == OFF && PARED_DERECHA == ON){
				// Giro a la izquierda
				robotAction = ACTION_GIRO_IZQUIERDA;
				PID_Reset(&PID_Turn);
				MPU6050_ResetYaw(&hi2c2);
				TURNING = ON;
				return;
			} else if (PARED_IZQUIERDA == ON && PARED_DERECHA == OFF){
				// Giro a la derecha
				robotAction = ACTION_GIRO_DERECHA;
				PID_Reset(&PID_Turn);
				MPU6050_ResetYaw(&hi2c2);
				TURNING = ON;
				return;
			} else if (PARED_IZQUIERDA == OFF && PARED_DERECHA == OFF){
				// Bifurcación en T: caminos a la izquierda y a la derecha
				robotAction = (rand() % 2) ? ACTION_GIRO_IZQUIERDA : ACTION_GIRO_DERECHA;
				PID_Reset(&PID_Turn);
				MPU6050_ResetYaw(&hi2c2);
				TURNING = ON;
				return;
			}
		} else if (PARED_DELANTERA == OFF){
			if (PARED_IZQUIERDA == OFF && PARED_DERECHA == ON){
				// Bifurcacion en J: caminos a la izquierda y hacia adelante
				robotAction = (rand() % 2) ? ACTION_GIRO_IZQUIERDA : ACTION_AVANZAR;
				if (robotAction == ACTION_GIRO_IZQUIERDA){
					PID_Reset(&PID_Turn);
					MPU6050_ResetYaw(&hi2c2);
					TURNING = ON;
				} else {
					PID_Reset(&PID_Navigation);
					MPU6050_ResetYaw(&hi2c2);
				}
				return;
			} else if (PARED_IZQUIERDA == ON && PARED_DERECHA == OFF){
				// Bifurcacion en L: caminos a la derecha y hacia adelante
				robotAction = (rand() % 2) ? ACTION_GIRO_DERECHA : ACTION_AVANZAR;
				if (robotAction == ACTION_GIRO_DERECHA){
					PID_Reset(&PID_Turn);
					MPU6050_ResetYaw(&hi2c2);
					TURNING = ON;
				} else {
					PID_Reset(&PID_Navigation);
					MPU6050_ResetYaw(&hi2c2);
				}
				return;
			} else if (PARED_IZQUIERDA == OFF && PARED_DERECHA == OFF){
				// Bifurcacion total: caminos a la izquierda, a la derecha y hacia adelante
				robotAction = (rand() % 3);
				if (robotAction == ACTION_GIRO_IZQUIERDA || robotAction == ACTION_GIRO_DERECHA){
					PID_Reset(&PID_Turn);
					MPU6050_ResetYaw(&hi2c2);
					TURNING = ON;
				} else {
					PID_Reset(&PID_Navigation);
					MPU6050_ResetYaw(&hi2c2);
				}
			}
		}
	}

	// AVANZAR
	if (PARED_FRONTAL == OFF)
		robotAction = ACTION_AVANZAR;
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
			turn90(RIGHT_SIDE);
			break;
		case ACTION_GIRO_IZQUIERDA:
			turn90(LEFT_SIDE);
			break;
		case ACTION_GIRO_EN_U:
			turn180();
			break;
		default:
			break;
	}
}

void displayTask(){
	Display_Clear();

	switch (menuDisplay){
		case 0:
			//Display_DrawBitmap(Display_WIDTH, Display_HEIGHT, myDesignBKG);
			Display_SetCursor(2, 5);
			sprintf(strAux, "INI %d, %d", myMaze.posIniX, myMaze.posIniY);
			Display_WriteString(strAux, Font_7x10, White);

			Display_SetCursor(2, 17);
			sprintf(strAux, "ACT %d, %d", myMaze.posActX, myMaze.posActY);
			Display_WriteString(strAux, Font_7x10, White);

			Display_SetCursor(2, 29);
			sprintf(strAux, "FIN %d, %d", myMaze.posFinX, myMaze.posFinY);
			Display_WriteString(strAux, Font_7x10, White);

			Display_SetCursor(2, 41);
			sprintf(strAux, "DIR %d", myMaze.dir);
			Display_WriteString(strAux, Font_7x10, White);
			break;
		case 1:
			Display_SetCursor(2, 2);
			sprintf(strAux, "PID %.3d", testds);
			Display_WriteString(strAux, Font_7x10, White);

			Display_SetCursor(86, 2);
			sprintf(strAux, "%.02d:%.02d", myDisplay.hs, myDisplay.min);
			Display_WriteString(strAux, Font_7x10, White);

			Display_SetCursor(2, 15);
			sprintf(strAux, "PWL PWR");
			Display_WriteString(strAux, Font_7x10, Black);

			Display_SetCursor(2, 26);
			sprintf(strAux, "%.3d %.3d", myMotor[LEFT].pow, myMotor[RIGHT].pow);
			Display_WriteString(strAux, Font_7x10, White);

			Display_SetCursor(62, 15);
			sprintf(strAux, " YAW ACT");
			Display_WriteString(strAux, Font_7x10, Black);

			Display_SetCursor(62, 26);
			sprintf(strAux, "%.04d  %d", myMPU.Yaw, robotAction);
			Display_WriteString(strAux, Font_7x10, White);

			Display_SetCursor(2, 39);
			if (MODESTARTED == ON)
				sprintf(strAux, "MODO %d  ON", robotMode);
			else
				sprintf(strAux, "MODO %d OFF", robotMode);
			Display_WriteString(strAux, Font_7x10, Black);

			Display_SetCursor(2, 51);
			sprintf(strAux, "I%d DI%d F%d DD%d D%d", PARED_IZQUIERDA, DIAG_IZQUIERDA, PARED_DELANTERA, DIAG_DERECHA, PARED_DERECHA);
			Display_WriteString(strAux, Font_7x10, White);
			break;
		case 2:
			// IR 0
			Display_SetCursor(108, 42);
			sprintf(strAux, "0");
			Display_WriteString(strAux, Font_7x10, White);
			Display_SetCursor(96, 53);
			Display_DrawFilledRectangle(7*4, 10, White);
			sprintf(strAux, "%.4d", myADC.filteredSamples[RIGHT_SIDE]);
			Display_WriteString(strAux, Font_7x10, Black);
			// IR 1
			Display_SetCursor(108, 18);
			sprintf(strAux, "1");
			Display_WriteString(strAux, Font_7x10, White);
			Display_SetCursor(96, 29);
			Display_DrawFilledRectangle(7*4, 10, White);
			sprintf(strAux, "%.4d", myADC.filteredSamples[FRONT_RIGHT]);
			Display_WriteString(strAux, Font_7x10, Black);
			// IR 2
			Display_SetCursor(82, 1);
			sprintf(strAux, "2");
			Display_WriteString(strAux, Font_7x10, White);
			Display_SetCursor(70, 12);
			Display_DrawFilledRectangle(7*4, 10, White);
			sprintf(strAux, "%.4d", myADC.filteredSamples[FRONT_1]);
			Display_WriteString(strAux, Font_7x10, Black);
			// IR 3
			Display_SetCursor(60, 18);
			sprintf(strAux, "3");
			Display_WriteString(strAux, Font_7x10, White);
			Display_SetCursor(48, 29);
			Display_DrawFilledRectangle(7*4, 10, White);
			sprintf(strAux, "%.4d", myADC.filteredSamples[GROUND_FRONT]);
			Display_WriteString(strAux, Font_7x10, Black);
			// IR 4
			Display_SetCursor(38, 1);
			sprintf(strAux, "4");
			Display_WriteString(strAux, Font_7x10, White);
			Display_SetCursor(26, 12);
			Display_DrawFilledRectangle(7*4, 10, White);
			sprintf(strAux, "%.4d", myADC.filteredSamples[FRONT_2]);
			Display_WriteString(strAux, Font_7x10, Black);
			// IR 5
			Display_SetCursor(12, 18);
			sprintf(strAux, "5");
			Display_WriteString(strAux, Font_7x10, White);
			Display_SetCursor(0, 29);
			Display_DrawFilledRectangle(7*4, 10, White);
			sprintf(strAux, "%.4d", myADC.filteredSamples[FRONT_LEFT]);
			Display_WriteString(strAux, Font_7x10, Black);
			// IR 6
			Display_SetCursor(12, 42);
			sprintf(strAux, "6");
			Display_WriteString(strAux, Font_7x10, White);
			Display_SetCursor(0, 53);
			Display_DrawFilledRectangle(7*4, 10, White);
			sprintf(strAux, "%.4d", myADC.filteredSamples[LEFT_SIDE]);
			Display_WriteString(strAux, Font_7x10, Black);
			// IR 7
			Display_SetCursor(60, 42);
			sprintf(strAux, "7");
			Display_WriteString(strAux, Font_7x10, White);
			Display_SetCursor(48, 53);
			Display_DrawFilledRectangle(7*4, 10, White);
			sprintf(strAux, "%.4d", myADC.filteredSamples[GROUND_BACK]);
			Display_WriteString(strAux, Font_7x10, Black);
			break;
		default:
			break;
	}

	UPDATEDISPLAY = ON;
}

void handlePosition(){
	//if (NEWCELL == OFF)
	//	return;

	if (robotMode != MODE_EXPLORE_MAZE)
		return;

	if (robotAction == ACTION_GIRO_DERECHA) {
		if      (myMaze.dir == NORTE) myMaze.dir = ESTE;
		else if (myMaze.dir == ESTE)  myMaze.dir = SUR;
		else if (myMaze.dir == SUR)   myMaze.dir = OESTE;
		else if (myMaze.dir == OESTE) myMaze.dir = NORTE;
	}
	else if (robotAction == ACTION_GIRO_IZQUIERDA) {
		if      (myMaze.dir == NORTE) myMaze.dir = OESTE;
		else if (myMaze.dir == OESTE) myMaze.dir = SUR;
		else if (myMaze.dir == SUR)   myMaze.dir = ESTE;
		else if (myMaze.dir == ESTE)  myMaze.dir = NORTE;
	}
	else if (robotAction == ACTION_GIRO_EN_U) {
		if      (myMaze.dir == NORTE) myMaze.dir = SUR;
		else if (myMaze.dir == SUR)   myMaze.dir = NORTE;
		else if (myMaze.dir == ESTE)  myMaze.dir = OESTE;
		else if (myMaze.dir == OESTE) myMaze.dir = ESTE;
	}

	switch (myMaze.dir){
		case NORTE:
			myMaze.posActY++;
			break;
		case SUR:
			myMaze.posActY--;
			break;
		case ESTE:
			myMaze.posActX++;
			break;
		case OESTE:
			myMaze.posActX--;
			break;
		default:
			break;
	}

	// Enviar al ESP01 (WiFi/UDP)
	UNERBUS_WriteByte(&unerbusESP01, (uint8_t)myMaze.posActX);
	UNERBUS_WriteByte(&unerbusESP01, (uint8_t)myMaze.posActY);
	UNERBUS_WriteByte(&unerbusESP01, (uint8_t)myMaze.dir);
	UNERBUS_Send(&unerbusESP01, MAZE_POSITION, 4);

	//NEWCELL = OFF;
}

void handleDirection() {
	// test
}

void funcTest(){
	static uint8_t toggleBit = 0;
	static uint8_t debounceCounter = 1;

	if (TURNING == OFF){
		robotAction = ACTION_AVANZAR;
	}

	if (PISO_ATRAS == ON && TURNING == OFF){
		debounceCounter++;
		if (debounceCounter > 2){
			debounceCounter = 0;
			toggleBit ^= 1;
			if (toggleBit == 1)
				robotAction = ACTION_GIRO_IZQUIERDA;
			else
				robotAction = ACTION_GIRO_DERECHA;
			PID_Reset(&PID_Turn);
			MPU6050_ResetYaw(&hi2c2);
			TURNING = ON;
		}
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

  PID_Init(&PID_Navigation, 45, 0, 100, 35, 35); // VELOCIDAD MAX +/-80%, VELOCIDAD BASE 35% (40/0/65 35/35)
  PID_Init(&PID_Turn, 30, 0, 60, 25, 0); // VELOCIDAD MAX = VEL EXT + PWM MAX = 85% //+/-80%, 40% * 1.6 = (1/2/2) (60/40/0 20/0)
  PID_Init(&PID_U_Turn, 10, 0, 12, 35, 35); // VELOCIDAD MAX +/-60%, VELOCIDAD BASE 35% (8/0/12 35/35)

  PID_Turn.velExt = 50; // 60
  PID_Turn.velInt = 20; // 20
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
  UPDATEDISPLAY = ON;

  robotMode = MODE_IDLE;
  robotAction = ACTION_STANDBY;

  myMaze.posIniX = 1;
  myMaze.posIniY = 1;
  myMaze.posActX = 1;
  myMaze.posActY = 1;
  myMaze.posFinX = 8;
  myMaze.posFinY = 6;
  myMaze.dir = NORTE;
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
