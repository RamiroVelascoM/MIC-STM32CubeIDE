/*
 * BUTTON.h
 *
 *  Created on: Aug 06, 2025
 *      Author: Ramiro Velasco
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "stm32f1xx_hal.h" // Incluimos la HAL de tu micro (F1 para Bluepill)

// === Constantes Configurables ===
#define DEBOUNCE_TIME_MS      40  	// Tiempo de anti-rebote en milisegundos
#define LONG_PRESS_TIME_MS    1000 // Tiempo para considerar una pulsación larga

// === Enumeraciones ===

// Estados internos de la máquina de estados del botón.
typedef enum {
    BUTTON_STATE_RELEASED = 0, // El botón está suelto (estado inicial)
    BUTTON_STATE_FALLING,      // Detectado flanco de bajada, esperando debounce
    BUTTON_STATE_PRESSED,      // Botón presionado y estable
    BUTTON_STATE_RISING,       // Detectado flanco de subida, esperando debounce
    BUTTON_STATE_LONG_PRESS    // Botón en estado de pulsación larga
} BUTTON_State_e;

// Eventos que la librería puede reportar a la aplicación principal.
typedef enum {
    EVENT_NONE = 0,           // Sin evento
    EVENT_PRESSED,            // Evento: el botón acaba de ser presionado (útil para acciones inmediatas)
    EVENT_PRESS_RELEASED,     // Evento: pulsación corta completada (presionar y soltar)
    EVENT_LONG_PRESS,         // Evento: se ha cumplido el tiempo de pulsación larga
    EVENT_LONG_PRESS_RELEASED // Evento: se ha soltado el botón después de una pulsación larga
} BUTTON_Event_e;


// === Estructura del Handle ===

// Contiene toda la información de estado para una instancia de botón.
typedef struct BUTTON_Handler_s {
    // --- Configuración (se establece en la inicialización) ---
    GPIO_TypeDef*   port;               // Puerto GPIO del botón
    uint16_t        pin;                // Pin GPIO del botón

    // button.h

	// --- Callbacks (punteros a función) ---
	void (*onPressReleasedCallback)(void);
	void (*onLongPressCallback)(void);

    // --- Estado interno (manejado por la librería) ---
    BUTTON_State_e  state;              // Estado actual de la FSM
    uint32_t        debounceTimerMs;  // Temporizador para el anti-rebote
    uint32_t        longPressTimerMs;// Temporizador para la pulsación larga
    BUTTON_Event_e  lastEvent;         // Último evento detectado para ser leído por la app
} _sButton;


// === Prototipos de Funciones Públicas ===

/**
 * @brief Inicializa una instancia (handle) de un botón.
 * @param handle Puntero a la estructura del botón a inicializar.
 * @param port   Puerto GPIO donde está conectado el botón (ej. GPIOB).
 * @param pin    Pin GPIO donde está conectado el botón (ej. GPIO_PIN_0).
 */
void BUTTON_Init(_sButton* handle, GPIO_TypeDef* port, uint16_t pin);

/**
 * @brief Actualiza la máquina de estados del botón.
 * @note Esta función debe ser llamada periódicamente en el bucle principal.
 * @param handle Puntero a la estructura del botón a actualizar.
 */
void BUTTON_Update(_sButton* handle);

/**
 * @brief Obtiene el último evento detectado para el botón.
 * @note Al llamar esta función, el evento se consume (se resetea a EVENT_NONE).
 * @param handle Puntero a la estructura del botón.
 * @return El evento de tipo ButtonEvent_e.
 */
BUTTON_Event_e BUTTON_GetEvent(_sButton* handle);

void BUTTON_Register_PressReleasedCallback(_sButton *handle, void (*callback)(void));
void BUTTON_Register_LongPressCallback(_sButton *handle, void (*callback)(void));

#endif /* INC_BUTTON_H_ */
