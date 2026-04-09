/*
 * display.c
 *
 *  Created on: May 31, 2024
 *      Author: Ramiro Velasco
 */

#include "display.h"

/* Buffer de pantalla en RAM */
static uint8_t Display_Buffer[DISPLAY_BUFFER_SIZE];
static Display_t Display;

/* Variable externa para datos de usuario */
extern _sDisplayData myDisplay;

/**
 * @brief Escribe un comando al display (Bloqueante)
 */
static uint8_t Display_WriteCommand(I2C_HandleTypeDef *hi2c, uint8_t command) {
    return HAL_I2C_Mem_Write(hi2c, Display_I2C_ADDR, 0x00, 1, &command, 1, 10);
}

/**
 * @brief Inicialización completa del SSD1306 (Lógica extendida de Agustín)
 */
uint8_t Display_Init(I2C_HandleTypeDef *hi2c) {
    HAL_Delay(100); // Espera estabilización
    uint8_t status = 0;

    status += Display_WriteCommand(hi2c, 0xAE); // Display Off
    status += Display_WriteCommand(hi2c, 0x20); // Memory Mode
    status += Display_WriteCommand(hi2c, 0x10); // Page Addressing Mode
    status += Display_WriteCommand(hi2c, 0xB0); // Page Start Address
    status += Display_WriteCommand(hi2c, 0xC8); // COM Scan Direction
    status += Display_WriteCommand(hi2c, 0x00); // Low Column Address
    status += Display_WriteCommand(hi2c, 0x10); // High Column Address
    status += Display_WriteCommand(hi2c, 0x40); // Start Line Address
    status += Display_WriteCommand(hi2c, 0x81); // Contrast
    status += Display_WriteCommand(hi2c, 0xFF);
    status += Display_WriteCommand(hi2c, 0xA1); // Segment Re-map
    status += Display_WriteCommand(hi2c, 0xA6); // Normal Display
    status += Display_WriteCommand(hi2c, 0xA8); // Multiplex Ratio
    status += Display_WriteCommand(hi2c, Display_HEIGHT - 1);
    status += Display_WriteCommand(hi2c, 0xA4); // RAM Content Display
    status += Display_WriteCommand(hi2c, 0xD3); // Display Offset
    status += Display_WriteCommand(hi2c, 0x00);
    status += Display_WriteCommand(hi2c, 0xD5); // Display Clock
    status += Display_WriteCommand(hi2c, 0xF0);
    status += Display_WriteCommand(hi2c, 0xD9); // Pre-charge
    status += Display_WriteCommand(hi2c, 0x22);
    status += Display_WriteCommand(hi2c, 0xDA); // COM Pins Config
    status += Display_WriteCommand(hi2c, (0x02 | (1 << 4))); // Alternative Config
    status += Display_WriteCommand(hi2c, 0xDB); // VCOMH
    status += Display_WriteCommand(hi2c, 0x20);
    status += Display_WriteCommand(hi2c, 0x8D); // Charge Pump
    status += Display_WriteCommand(hi2c, 0x14);
    status += Display_WriteCommand(hi2c, 0xAF); // Display ON

    if (status != 0) return 1;

    /* Configuración Inicial */
    Display_Fill(Black);
    Display.CurrentX = 0;
    Display.CurrentY = 0;
    Display.Inverted = 0;
    Display.Initialized = 1;

    /* UI Inicial (Personalizada de Ramiro) */
    myDisplay.hs = 0;
    myDisplay.min = 0;

    return 0;
}

/**
 * @brief Máquina de estados para actualizar la pantalla vía DMA (No bloqueante)
 */
uint8_t Display_UpdateScreen(I2C_HandleTypeDef *hi2c) {
	static uint8_t page = 0;
	static uint8_t sub_page = 0; // Nuevo: para dividir la página de 128 bytes
	static uint8_t step = 0;

	// Tamaño del fragmento (Chunk). 16 bytes para dejar aire al MPU.
	const uint8_t CHUNK_SIZE = 16;

	switch (step) {
		case 0: // Configurar Página
			if(HAL_I2C_Mem_Write_DMA(hi2c, Display_I2C_ADDR, 0x00, 1, (uint8_t[]){0xB0 + page}, 1) == HAL_OK) step++;
			break;
		case 1: // Configurar Columna Low
			if(HAL_I2C_Mem_Write_DMA(hi2c, Display_I2C_ADDR, 0x00, 1, (uint8_t[]){0x00}, 1) == HAL_OK) step++;
			break;
		case 2: // Configurar Columna High
			if(HAL_I2C_Mem_Write_DMA(hi2c, Display_I2C_ADDR, 0x00, 1, (uint8_t[]){0x10}, 1) == HAL_OK) step++;
			break;
		case 3: // Enviar FRAGMENTO de datos (16 bytes en lugar de 128)
			if(HAL_I2C_Mem_Write_DMA(hi2c, Display_I2C_ADDR, 0x40, 1,
			   &Display_Buffer[(Display_WIDTH * page) + (sub_page * CHUNK_SIZE)], CHUNK_SIZE) == HAL_OK) {

				sub_page++;
				if (sub_page >= (Display_WIDTH / CHUNK_SIZE)) { // ¿Terminamos los 128 bytes?
					sub_page = 0;
					page++;
					step = 0; // Reiniciar para la siguiente página completa
				}
				// Nota: No cambiamos de step si sub_page < 4,
				// para que la próxima vez que entre vuelva a enviar otro chunk.
			}
			break;
	}

	if (page > 7) {
		page = 0;
		return 0; // Pantalla completa actualizada
	}
	return 1; // Sigue trabajando
}

/**
 * @brief Dibuja un píxel en el buffer
 */
void Display_DrawPixel(uint8_t x, uint8_t y, Display_COLOR color) {
    if (x >= Display_WIDTH || y >= Display_HEIGHT) return;

    if (Display.Inverted) color = (Display_COLOR)!color;

    if (color == White)
        Display_Buffer[x + (y / 8) * Display_WIDTH] |= 1 << (y % 8);
    else
        Display_Buffer[x + (y / 8) * Display_WIDTH] &= ~(1 << (y % 8));
}

/**
 * @brief Dibuja una línea (Algoritmo de Agustín optimizado)
 */
void Display_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, Display_COLOR c) {
    int16_t dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
    int16_t dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;

    while (1) {
        Display_DrawPixel(x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}

/**
 * @brief Dibuja un rectángulo sin relleno
 */
void Display_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, Display_COLOR c) {
    Display_DrawLine(x, y, x + w, y, c);
    Display_DrawLine(x, y + h, x + w, y + h, c);
    Display_DrawLine(x, y, x, y + h, c);
    Display_DrawLine(x + w, y, x + w, y + h, c);
}

/**
 * @brief Dibuja un rectángulo con relleno
 */
void Display_DrawFilledRectangle(uint16_t w, uint16_t h, Display_COLOR c) {
    for (uint16_t i = 0; i <= h; i++) {
        Display_DrawLine(Display.CurrentX, Display.CurrentY + i, Display.CurrentX + w, Display.CurrentY + i, c);
    }
}

/**
 * @brief Dibuja un Bitmap en coordenadas específicas
 */
void Display_DrawBitmap(int16_t w, int16_t h, const unsigned char* bitmap) {
	uint16_t byteWidth = (w + 7) / 8; // Calculate the width in bytes
		for (uint16_t y = 0; y < h; y++)
		{
			for (uint16_t x = 0; x < w; x++)
			{
				if (bitmap[y * byteWidth + x / 8] & (128 >> (x & 7)))
					Display_DrawPixel(x, y, White);
			}
		}
}

/**
 * @brief Escribe un carácter (Manejo de fuentes)
 */
char Display_WriteChar(char ch, FontDef Font, Display_COLOR color) {
    uint32_t i, b, j;
    if (Display_WIDTH <= (Display.CurrentX + Font.FontWidth) || Display_HEIGHT <= (Display.CurrentY + Font.FontHeight))
        return 0;

    for (i = 0; i < Font.FontHeight; i++) {
        b = Font.data[(ch - 32) * Font.FontHeight + i];
        for (j = 0; j < Font.FontWidth; j++) {
            if ((b << j) & 0x8000)
                Display_DrawPixel(Display.CurrentX + j, (Display.CurrentY + i), color);
            else
                Display_DrawPixel(Display.CurrentX + j, (Display.CurrentY + i), (Display_COLOR)!color);
        }
    }
    Display.CurrentX += Font.FontWidth;
    return ch;
}

char Display_WriteString(const char* str, FontDef Font, Display_COLOR color) {
    while (*str) {
        if (Display_WriteChar(*str, Font, color) != *str) return *str;
        str++;
    }
    return 0;
}

void Display_Fill(Display_COLOR color) {
    memset(Display_Buffer, (color == Black) ? 0x00 : 0xFF, DISPLAY_BUFFER_SIZE);
}

void Display_Clear(void) {
    Display_Fill(Black);
}

void Display_SetCursor(uint8_t x, uint8_t y) {
    Display.CurrentX = x;
    Display.CurrentY = y;
}

void Display_InvertColors(void) {
    Display.Inverted = !Display.Inverted;
}

/**
 * @brief Actualiza la información específica de la aplicación (Reloj y Vía)
 */
void Display_UpdateInfo(I2C_HandleTypeDef *hi2c, _sDisplayData *mySSD) {
	// Actualizar Hora
	Display_SetCursor(85,3);
	sprintf(mySSD->dateText, "%.2d:%.2d", mySSD->hs, mySSD->min);
	Display_WriteString(mySSD->dateText, Font_7x10, White);

	// Actualizar Vía
	Display_SetCursor(101,37);
	if (mySSD->via == 0)      Display_WriteString("USB", Font_7x10, White);
	else if (mySSD->via == 1) Display_WriteString("ESP", Font_7x10, White);
	else                      Display_WriteString("PC ", Font_7x10, White);
}
