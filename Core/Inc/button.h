/**
  ******************************************************************************
  * @file    button.h
  * @author  Ramiro Velasco
  * @brief   Header file 
  ******************************************************************************
  * @attention
  *
  *
  * Copyright (c) 2024 HGE.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef button_H_
#define button_H_

#include <stdint.h>

/**
 * @brief Enumeracion de los estados de la maquina de estado de un pulsador
 * 
 */
typedef enum
{
    UP,
    DOWN,
    FALLING,
    RISING
} _eStates;
/**
 * @brief Enumeracion de los posibles eventos de un pulsador
 * 
 */
typedef enum
{
    EV_PRESSED = 0,
    EV_NOT_PRESSED = 1,
    EV_NONE = 2
} _eButtonEvent;
/**
 * @brief Estructura general de un pulsador
 * 
 */
typedef struct
{
	uint8_t			value;
    _eStates        estado;
    _eButtonEvent   event;
    uint32_t        timePush;
    uint32_t        timeDiff;
} _sButton;

/**
 * @brief Funcion para inicializar los valores de un boton/pulsador.
 * 
 * @param button Estructura del boton o pulsador de interes.
 */
void Button_Init(_sButton *button);
/**
 * @brief Maquina de estado de un boton o pulsador.
 * 
 * @param button Estructura del boton o pulsador de interes.
 */
void Button_CheckStatus(_sButton *button);

#endif /* button_H_ */
