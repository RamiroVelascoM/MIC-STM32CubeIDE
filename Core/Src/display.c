/*
 * display.c
 *
 *  Created on: May 31, 2024
 *      Author: Ramiro Velasco
 */

#include "display.h"

static uint8_t Display_Buffer[Display_WIDTH * Display_HEIGHT / 8];	// Screenbuffer

static Display_t Display;	// Screen object

/**
 *	BLOCKING METHOD TO WRITE INTO THE DISPLAY
 */
static uint8_t Display_WriteCommand(I2C_HandleTypeDef *hi2c, uint8_t command)
{
    return HAL_I2C_Mem_Write(hi2c, Display_I2C_ADDR, 0x00, 1, &command, 1, 10);
}

/**
 * NON-BLOCKING METHOD TO WRITE INTO THE DISPLAY
 */
static uint8_t Display_WriteCommand_DMA(I2C_HandleTypeDef *hi2c, uint8_t command)
{
    return HAL_I2C_Mem_Write_DMA(hi2c, Display_I2C_ADDR, 0x00, 1, &command, 1);
}

/**
 * INITIALIZE THE DISPLAY
 */
uint8_t Display_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_Delay(100);	// Wait for the screen to boot
    int status = 0;

    status += Display_WriteCommand(hi2c, 0xAE);   // Display off
    status += Display_WriteCommand(hi2c, 0x20);   // Set Memory Addressing Mode
    status += Display_WriteCommand(hi2c, 0x10);   // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    status += Display_WriteCommand(hi2c, 0xB0);   // Set Page Start Address for Page Addressing Mode,0-7
    status += Display_WriteCommand(hi2c, 0xC8);   // Set COM Output Scan Direction
    status += Display_WriteCommand(hi2c, 0x00);   // Set low column address
    status += Display_WriteCommand(hi2c, 0x10);   // Set high column address
    status += Display_WriteCommand(hi2c, 0x40);   // Set start line address
    status += Display_WriteCommand(hi2c, 0x81);   // set contrast control register
    status += Display_WriteCommand(hi2c, 0xFF);
    status += Display_WriteCommand(hi2c, 0xA1);   // Set segment re-map 0 to 127
    status += Display_WriteCommand(hi2c, 0xA6);   // Set normal display

    status += Display_WriteCommand(hi2c, 0xA8);   // Set multiplex ratio(1 to 64)
    status += Display_WriteCommand(hi2c, Display_HEIGHT - 1);

    status += Display_WriteCommand(hi2c, 0xA4);   // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    status += Display_WriteCommand(hi2c, 0xD3);   // Set display offset
    status += Display_WriteCommand(hi2c, 0x00);   // No offset
    status += Display_WriteCommand(hi2c, 0xD5);   // Set display clock divide ratio/oscillator frequency
    status += Display_WriteCommand(hi2c, 0xF0);   // Set divide ratio
    status += Display_WriteCommand(hi2c, 0xD9);   // Set pre-charge period
    status += Display_WriteCommand(hi2c, 0x22);

    status += Display_WriteCommand(hi2c, 0xDA);   // Set com pins hardware configuration
    status += Display_WriteCommand(hi2c, Display_COM_LR_REMAP << 5 | Display_COM_ALTERNATIVE_PIN_CONFIG << 4 | 0x02);

    status += Display_WriteCommand(hi2c, 0xDB);   // Set vcomh
    status += Display_WriteCommand(hi2c, 0x20);   // 0x20,0.77xVcc
    status += Display_WriteCommand(hi2c, 0x8D);   // Set DC-DC enable
    status += Display_WriteCommand(hi2c, 0x14);   //
    status += Display_WriteCommand(hi2c, 0xAF);   // Turn on Display panel

    if (status != 0)
        return 1;

    Display_Fill(Black);		// Clear screen
    Display.CurrentX = 0;		// Set default values for screen object
    Display.CurrentY = 0;
    Display.Initialized = 1;	// Checked initialization of the display

    return 0;
}

/**
 * FILL THE DISPLAY WITH A CERTAIN COLOR
 */
void Display_Fill(Display_COLOR color)
{
    uint32_t i;

    for(i = 0; i < sizeof(Display_Buffer); i++)	// Fill screenbuffer with a constant value (color)
    {
        Display_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

/**
 * UPDATE DATA OF THE DISPLAY
 */
uint8_t Display_UpdateScreen(I2C_HandleTypeDef *hi2c){
	static uint8_t updateScreenState = 0;
	static uint8_t repetition = 0;
	uint8_t IsUpdateScreenTime = 1;

	switch (updateScreenState) {
		case 0:
			Display_WriteCommand_DMA(hi2c, 0xB0 + repetition);
			break;
		case 1:
			Display_WriteCommand_DMA(hi2c, 0x00);
			break;
		case 2:
			Display_WriteCommand_DMA(hi2c, 0x10);
			break;
		case 3:
			HAL_I2C_Mem_Write_DMA(hi2c, Display_I2C_ADDR, 0x40, 1, &Display_Buffer[Display_WIDTH * repetition], Display_WIDTH);
			break;
	}
	updateScreenState++;

	if (updateScreenState > 3) {
		updateScreenState = 0;
		repetition++;

		if (repetition > 7) {
			updateScreenState = 0;
			repetition = 0;
			IsUpdateScreenTime = 0;
		}
	}
	return(IsUpdateScreenTime);
}

/**
 * DRAW A PIXEL IN THE DISPLAY
 */
void Display_DrawPixel(uint8_t x, uint8_t y, Display_COLOR color)
{
    if (x >= Display_WIDTH || y >= Display_HEIGHT)	// Don't write outside the buffer
        return;

    if (Display.Inverted)				// Check if pixel should be inverted
    	color = (Display_COLOR)!color;

    if (color == White)		// Draw in the correct color
        Display_Buffer[x + (y / 8) * Display_WIDTH] |= 1 << (y % 8);
    else
        Display_Buffer[x + (y / 8) * Display_WIDTH] &= ~(1 << (y % 8));
}


/**
 * WRITE A CHAR INTO THE DISPLAY BUFFER
 */
char Display_WriteChar(char ch, FontDef Font, Display_COLOR color)
{
    uint32_t i, b, j;

    // Check remaining space on current line
    if (Display_WIDTH <= (Display.CurrentX + Font.FontWidth) ||
        Display_HEIGHT <= (Display.CurrentY + Font.FontHeight))
    {

        return 0;	// Not enough space on current line
    }

    for (i = 0; i < Font.FontHeight; i++)	// Translate font to screenbuffer
    {
        b = Font.data[(ch - 32) * Font.FontHeight + i];
        for (j = 0; j < Font.FontWidth; j++)
        {
            if ((b << j) & 0x8000)
                Display_DrawPixel(Display.CurrentX + j, (Display.CurrentY + i), (Display_COLOR) color);
            else
                Display_DrawPixel(Display.CurrentX + j, (Display.CurrentY + i), (Display_COLOR)!color);
        }
    }
    Display.CurrentX += Font.FontWidth;	// The current space is now taken

    return ch;	// Return written char for validation
}

/**
 * WRITE STRING INTO THE DISPLAY BUFFER
 */
char Display_WriteString(const char* str, FontDef Font, Display_COLOR color)
{
    while (*str){
        if (Display_WriteChar(*str, Font, color) != *str)	// Write until null-byte
        	return *str;	// Char could not be written

        str++;	// Next char
    }
    return *str;	// Everything ok
}

/**
 * INVERT DISPLAY COLORS
 */
void Display_InvertColors(void)
{
    Display.Inverted = !Display.Inverted;
}

/**
 * SET CURSOR POSITION
 */
void Display_SetCursor(uint8_t x, uint8_t y)
{
    Display.CurrentX = x;
    Display.CurrentY = y;
}

/**
 * DRAW BITMAP
 */
void Display_DrawBitmap(uint8_t W, uint8_t H, const uint8_t* pBMP)
{
	uint16_t byteWidth = (W + 7) / 8; // Calculate the width in bytes
	for (uint16_t y = 0; y < H; y++)
	{
		for (uint16_t x = 0; x < W; x++)
		{
			if (pBMP[y * byteWidth + x / 8] & (128 >> (x & 7)))
				Display_DrawPixel(x, y, White);
		}
	}
}
