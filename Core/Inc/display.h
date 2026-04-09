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

#ifndef _DISPLAY_H
#define _DISPLAY_H

#include "stm32f1xx_hal.h"
#include "fonts.h"
#include <stdio.h>
#include <string.h>

/* --- Configuraciones de Hardware --- */
#ifndef Display_I2C_ADDR
#define Display_I2C_ADDR        0x78
#endif

#ifndef Display_WIDTH
#define Display_WIDTH           128
#endif

#ifndef Display_HEIGHT
#define Display_HEIGHT          64
#endif

#define DISPLAY_BUFFER_SIZE     (Display_WIDTH * Display_HEIGHT / 8)

/* --- Definiciones de Colores --- */
typedef enum {
    Black = 0x00,   // Píxel apagado
    White = 0x01    // Píxel encendido
} Display_COLOR;

/* --- Estructuras de Control --- */
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} Display_t;

/**
 * @brief Estructura de datos específicos para la aplicación (Basado en Ramiro)
 */
typedef struct {
	uint8_t hs;
	uint8_t min;
	uint8_t via;
	char mainText[4];
	char dateText[8];
	char sourceText[6];
	char lowerText[8];
} _sDisplayData;

/* --- Funciones de Inicialización y Control --- */
uint8_t Display_Init(I2C_HandleTypeDef *hi2c);
uint8_t Display_UpdateScreen(I2C_HandleTypeDef *hi2c);
void    Display_ON(I2C_HandleTypeDef *hi2c);
void    Display_OFF(I2C_HandleTypeDef *hi2c);
void    Display_Fill(Display_COLOR color);
void    Display_Clear(void);

/* --- Funciones de Dibujo Gráfico (Basado en Agustín) --- */
void Display_DrawPixel(uint8_t x, uint8_t y, Display_COLOR color);
void Display_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, Display_COLOR c);
void Display_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, Display_COLOR c);
void Display_DrawFilledRectangle(uint16_t w, uint16_t h, Display_COLOR c);
void Display_DrawBitmap(int16_t w, int16_t h, const unsigned char* bitmap);

/* --- Funciones de Texto --- */
void Display_SetCursor(uint8_t x, uint8_t y);
char Display_WriteChar(char ch, FontDef Font, Display_COLOR color);
char Display_WriteString(const char* str, FontDef Font, Display_COLOR color);
void Display_InvertColors(void);

/* --- Funciones de Aplicación Específica --- */
void Display_UpdateInfo(I2C_HandleTypeDef *hi2c, _sDisplayData *mySSD);

#endif
