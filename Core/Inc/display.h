/**
 * Display library, created by Ramiro Velasco
 *
 * To use this library with SSD1306 OLED display you will need to customize the defines below.
 *
 * This library uses 2 extra files (fonts.c/h).
 * In this files are 3 different fonts you can use:
 *     - Font_7x10
 *     - Font_11x18
 *     - Font_16x26
 *
 */

#ifndef _display_H
#define _display_H

#include "stm32f1xx_hal.h"
#include "fonts.h"
#include <stdio.h>
// I2c address
#ifndef Display_I2C_ADDR
#define Display_I2C_ADDR        0x78
#endif // Display_I2C_ADDR

// Display width in pixels
#ifndef Display_WIDTH
#define Display_WIDTH           128
#endif // Display_WIDTH

// Display LCD height in pixels
#ifndef Display_HEIGHT
#define Display_HEIGHT          64
#endif // Display_HEIGHT

#ifndef Display_COM_LR_REMAP
#define Display_COM_LR_REMAP    0
#endif // Display_COM_LR_REMAP

#ifndef Display_COM_ALTERNATIVE_PIN_CONFIG
#define Display_COM_ALTERNATIVE_PIN_CONFIG    1
#endif // Display_COM_ALTERNATIVE_PIN_CONFIG

/**
 * SCREEN COLORS
 */
typedef enum {
    Black = 0x00,   // Black color, no pixel
    White = 0x01,   // Pixel is set. Color depends on LCD
} Display_COLOR;

/**
 * DISPLAY VARIABLES
 */
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} Display_t;

typedef struct{
	uint8_t hs;
	uint8_t min;
	uint8_t via;
	char mainText[4];
	char dateText[8];
	char sourceText[6];
	char lowerText[8];
}_sDisplayData;

/**
 * FUNCTION DEFINITIONS
 */
uint8_t Display_Init(I2C_HandleTypeDef *hi2c);

uint8_t Display_UpdateScreen(I2C_HandleTypeDef *hi2c);

void Display_Fill(Display_COLOR color);

void Display_DrawPixel(uint8_t x, uint8_t y, Display_COLOR color);

char Display_WriteChar(char ch, FontDef Font, Display_COLOR color);

char Display_WriteString(const char* str, FontDef Font, Display_COLOR color);

void Display_SetCursor(uint8_t x, uint8_t y);

void Display_InvertColors(void);

void Display_DrawBitmap(uint8_t W, uint8_t H, const uint8_t* pBMP);

void Display_UpdateInfo(I2C_HandleTypeDef *hi2c, _sDisplayData *mySSD);

#endif  // _display_H
