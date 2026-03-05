/*
 * BUTTON.c
 *
 *  Created on: Aug 06, 2025
 *      Author: Ramiro Velasco
 */

#include "button.h"

// Función privada para leer el estado físico del pin.
// Devuelve '1' si está presionado (LOW), '0' si está suelto (HIGH).
// Se abstrae para facilitar la lectura del código de la FSM.
static uint8_t BUTTON_Read_Pin(_sButton *handle) {
    // La lógica es invertida debido al pull-up:
    // Pin en RESET (0V) -> Botón presionado
    // Pin en SET (3.3V) -> Botón suelto
    if (HAL_GPIO_ReadPin(handle->port, handle->pin) == GPIO_PIN_RESET) {
        return 1; // Presionado
    }
    return 0; // Suelto
}


void BUTTON_Init(_sButton *handle, GPIO_TypeDef* port, uint16_t pin)
{
    handle->port = port;
    handle->pin = pin;
    handle->state = BUTTON_STATE_RELEASED;
    handle->lastEvent = EVENT_NONE;
    handle->debounceTimerMs = 0;
    handle->longPressTimerMs = 0;
    handle->onPressReleasedCallback = NULL;
    handle->onLongPressCallback = NULL;
}

void BUTTON_Update(_sButton *handle)
{
    uint8_t isPressed = BUTTON_Read_Pin(handle);
    uint32_t currentTick = HAL_GetTick();

    switch (handle->state)
    {
        case BUTTON_STATE_RELEASED:
            if (isPressed) {
                handle->state = BUTTON_STATE_FALLING;
                handle->debounceTimerMs = currentTick;
            }
            break;

        case BUTTON_STATE_FALLING:
            if (!isPressed) {
                // Rebote, volver a empezar
                handle->state = BUTTON_STATE_RELEASED;
            }
            else if (currentTick - handle->debounceTimerMs >= DEBOUNCE_TIME_MS) {
                // Debounce confirmado, botón presionado
                handle->state = BUTTON_STATE_PRESSED;
                handle->longPressTimerMs = currentTick;
                // Opcional: generar evento de presión inmediata si se necesita
                // handle->last_event = EVENT_PRESSED;
            }
            break;

        case BUTTON_STATE_PRESSED:
            if (!isPressed) {
                // SE HA SOLTADO EL BOTÓN ANTES DEL LONG PRESS -> ¡ES UN CLICK CORTO!
                handle->state = BUTTON_STATE_RISING;
                handle->debounceTimerMs = currentTick;
            }
            else if (currentTick - handle->longPressTimerMs >= LONG_PRESS_TIME_MS) {
                // TIEMPO DE LONG PRESS CUMPLIDO
                handle->state = BUTTON_STATE_LONG_PRESS;

                // Generar evento/llamar callback de LONG_PRESS
                handle->lastEvent = EVENT_LONG_PRESS;
                if (handle->onLongPressCallback != NULL) {
                    handle->onLongPressCallback();
                }
            }
            break;

        case BUTTON_STATE_RISING:
            if (isPressed) {
                 // Rebote, volver al estado presionado (no debería pasar a long press)
                handle->state = BUTTON_STATE_PRESSED;
            }
            else if (currentTick - handle->debounceTimerMs >= DEBOUNCE_TIME_MS) {
                // DEBOUNCE DE SUBIDA CONFIRMADO
                handle->state = BUTTON_STATE_RELEASED;

                // Generar evento/llamar callback de CLICK CORTO
                handle->lastEvent = EVENT_PRESS_RELEASED;
                if (handle->onPressReleasedCallback != NULL) {
                    handle->onPressReleasedCallback();
                }
            }
            break;

        case BUTTON_STATE_LONG_PRESS:
            // En este estado, solo nos importa si se suelta el botón
            if (!isPressed) {
                handle->state = BUTTON_STATE_RELEASED;

                // Opcional: generar evento/callback de que se soltó tras un long press
                handle->lastEvent = EVENT_LONG_PRESS_RELEASED;
                // Podrías añadir un callback para este evento si lo necesitas
                // if (handle->on_long_press_released_callback != NULL) { ... }
            }
            break;
    }
}

/**
 * @brief Registra (asigna) una función de callback para el evento de pulsación corta.
 * @param handle Puntero a la estructura del botón.
 * @param callback Puntero a la función que se ejecutará (debe ser de tipo 'void funcion(void)').
 */
void BUTTON_Register_PressReleasedCallback(_sButton* handle, void (*callback)(void))
{
    if (handle != NULL) {
        handle->onPressReleasedCallback = callback;
    }
}

/**
 * @brief Registra (asigna) una función de callback para el evento de pulsación larga.
 * @param handle Puntero a la estructura del botón.
 * @param callback Puntero a la función que se ejecutará (debe ser de tipo 'void funcion(void)').
 */
void BUTTON_Register_LongPressCallback(_sButton* handle, void (*callback)(void))
{
    if (handle != NULL) {
        handle->onLongPressCallback = callback;
    }
}

BUTTON_Event_e BUTTON_GetEvent(_sButton *handle){
    // "Consumir" el evento: lo leemos y lo reseteamos
	BUTTON_Event_e event = handle->lastEvent;
    handle->lastEvent = EVENT_NONE;
    return event;
}

