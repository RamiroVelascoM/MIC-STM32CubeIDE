/************************************************************************/
/*					Button Library (.c) - ATMEGA328P
					Ingenieria en Mecatronica - UNER					*/
/************************************************************************/

#include "button.h"
#include <stdlib.h>

uint8_t PRESSED = 0;

void inicializarBoton(_sButton *button)
{
	button->value = 0;
	button->estado = UP;
	button->event = EV_NONE;
	button->timePush = 0;
	button->timeDiff = 0;
}

void checkMEF(_sButton *button)
{
	switch (button->estado)
	{
		case UP:
			if (button->value == PRESSED)
			{
				button->estado = FALLING;
			}
			break;
		case FALLING:
			if (button->value == PRESSED)
			{
				button->estado = DOWN;
			}
			else
			{
				button->estado = UP;
			}
			break;
		case DOWN:
			if (button->value == !PRESSED)
			{
				button->estado = RISING;
			}
			break;
		case RISING:
			if (button->value == !PRESSED)
			{
				button->estado = UP;
			}
			else
			{
				button->estado = DOWN;
			}
			break;
		default:
			button->estado = UP;
			break;
	}
}

/* END Private Functions*/
