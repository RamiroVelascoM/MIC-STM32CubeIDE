/**
  ******************************************************************************
  * @file    ESP01.h
  * @author  Germán E. Hachmann
  * @brief   Header file containing functions prototypes of ESP01 library.
  ******************************************************************************
  * @attention
  *
  *
  * Copyright (c) 2023 HGE.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * Version: 01b05 - 04/08/2024
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ESP01_H_
#define ESP01_H_

#include <stdint.h>

/**< Estados ESP01 */
typedef enum{
	ESP01_NOT_INIT = -1,
	ESP01_WIFI_DISCONNECTED,
	ESP01_WIFI_NOT_SETED,
	ESP01_WIFI_CONNECTING_WIFI,
	ESP01_WIFI_CONNECTED,
	ESP01_WIFI_NEW_IP,
	ESP01_UDPTCP_DISCONNECTED,
	ESP01_UDPTCP_CONNECTING,
	ESP01_UDPTCP_CONNECTED,
	ESP01_SEND_BUSY,
	ESP01_SEND_READY,
	ESP01_SEND_OK,
	ESP01_SEND_ERROR,
} _eESP01STATUS;


#define ESP01RXBUFAT		128
#define ESP01TXBUFAT		256


/**< Inicializa el driver ESP01 UDP */
typedef struct{
	void (*DoCHPD)(uint8_t value);			    /**< Puntero a una función que permite manejar el pin CH_PD del ESP01 */
	int (*WriteUSARTByte)(uint8_t value);		/**< Puntero a una función que escribe un byte en la USART, devuelve 1 si pudo escribir */
	void (*WriteByteToBufRX)(uint8_t value);	/**< Puntero a una función que escribe un byte en buffer de recepción */
//	uint8_t 			*bufRX;				    /**< Puntero al buffer donde se guardarán los datos recibidos */
//	uint16_t			*iwRX;				    /**< Puntero al índice de escritura del buffer de recepción circular */
//	uint16_t			sizeBufferRX;		  	/**< Tamaño en bytes del buffer de recepción*/
} _sESP01Handle;


/**
 * @brief ESP01_WIFI Configura y Conecta
 *
 * Conecta a una red wifi especificada.
 * Si ya hay establecida una cominicación se deconecta y conecta a la nueva red wifi.
 * Use ESP01_STWIFI para verificar el estado de la conexión
 * Esta función se debe ejecutar después de ESP01_Init
 *
 * @param [in] ssid: Especifica el nuevo ssid
 * @param [in] password: Especifica el nuevo password
 *
 */
void ESP01_SetWIFI(const char *ssid, const char *password);


/**
 * @brief ESP01_START_UDP Configura y Conecta UDP
 *
 * Comienza una comunicación UDP, siempre que este conectado a WIFI.
 * Si hay una conexión establecida la cierra y se conecta esta nueva IP:PORT
 * Use ESP01_STUDP para verificar el estado de la conexión UDP
 * Si la read WIFI no esta disponible se conecta a esta IP:PORT automáticamante
 * cuando se reestablezaca la conexión WIFI
 * Esta función se debe ejecutar después de ESP01_Init
 *
 * @param [in] RemoteIP: Especifica la IP remota a transmitir
 * @param [in] RemotePORT: Especifica el puerto remoto a transmitir
 *
 */
_eESP01STATUS ESP01_StartUDP(const char *RemoteIP, uint16_t RemotePORT, uint16_t LocalPORT);


/**
 * @brief ESP01_START_TCP Configura y Conecta UDP
 *
 * Comienza una comunicación TCP, siempre que este conectado a WIFI.
 * Si hay una conexión establecida la cierra y se conecta esta nueva IP:PORT
 * Use ESP01_STUDP para verificar el estado de la conexión UDP
 * Si la read WIFI no esta disponible se conecta a esta IP:PORT automáticamante
 * cuando se reestablezaca la conexión WIFI
 * Esta función se debe ejecutar después de ESP01_Init
 *
 * @param [in] RemoteIP: Especifica la IP remota a transmitir
 * @param [in] RemotePORT: Especifica el puerto remoto a transmitir
 *
 */
_eESP01STATUS ESP01_StartTCP(const char *RemoteIP, uint16_t RemotePORT, uint16_t LocalPORT);

/**
 * @brief ESP01_CLOSEUDP Cierra una conexión UDP
 *
 */
void ESP01_CloseUDPTCP();

/**
 * @brief ESP01_STWIFI Devuelve el estado de la conexión WIFI
 *
 */
_eESP01STATUS ESP01_StateWIFI();

/**
 * @brief ESP01_GET_LOCAL_IP Devuelve la IP del ESP01
 *
 * @retVal Devuelve NULL Cuando no tiene IP
 */
char *ESP01_GetLocalIP();

/**
 * @brief ESP01_STWIFI Devuelve el estado de la conexión UDP
 *
 */
_eESP01STATUS ESP01_StateUDPTCP();


/**
 * @brief ESP01_Send
 *
 * Envía los datos guardados en el buffer circular de transmisión.
 *
 * @param [in] length: longitud del los datos a enviar.
 * @param [in] irRingBuf: indice de lectura del buffer circular.
 * @param [in] sizeRingBuf: tamaño en bytes del buffer circular.
 *
 * @retVal Si pudo transmitir devuelve ESP01_SEND_READY
 */
_eESP01STATUS ESP01_Send(uint8_t *buf, uint16_t irRingBuf, uint16_t length, uint16_t sizeRingBuf);


/**
 * @brief ESP01_Init Inicializa el driver ESP01
 *
 * Esta función debe llamarse en PRIMER lugar para incicializar el driver
 *
 * @param [in] hESP01: Puntero a un manejador para el ESP01
 *
 */
void ESP01_Init(_sESP01Handle *hESP01);


/**
 * @brief ESP01_Timeout10ms Mantiene el timer del driver
 *
 * Esta función debe llamarse cada 10ms para mantener el timer del driver
 * En el programa principal utilice un timer de 10ms y ejecute esta función
 * 
 * Ejemplo:
 * 
 * void On10ms(){
 *     ESP01_Timeout10ms();	
 *}
 *
 */
void ESP01_Timeout10ms();

/**
 * @brief ESP01_TASK Tarea principal del driver
 *
 * Mantiene el estado del driver ESP01
 * Debe llamarse repetidamente para que el driver pueda verifcar el estado, transmitir y recibir
 *
 * Ejemplo:
 *
 * while(1){
 * .
 * .
 * .
 * ESP01_TASK();
 * .
 * .
 * .
 * }
 *
 */
void ESP01_Task();

/**
 * @brief ESP01_WRITE_RX Escribe en el buffer de receoción
 *
 * Esta función debe llamarse cada vez que se reciba un bytes por el USART
 *
 * @param [in] value: valor recibido por USART (Enviado por el ESP01)
 *
 */
void ESP01_WriteRX(uint8_t value);

/**
 * @brief ESP01_AttachChangeState estado del driver
 *
 * Esta función se llama cada vez que el driver tiene un cambio de estado
 * Para desvincular el cambio de estado utilice ESP01_AttachChangeState(NULL);  
 * Esta función se debe ejecutar después de ESP01_Init
 *
 * @param [in] aOnESP01ChangeState: puntero a función que recibe el nuevo estado del driver 
 *
 */
void ESP01_AttachChangeState(void (*aESP01ChangeState)(_eESP01STATUS esp01State));

/**
 * @brief ESP01_AttachDebugStr para depuración
 *
 * Esta función se utiliza para realizar una depuración del driver 
 * Para desvincular el la cadena de depuración utilice ESP01_AttachDebugStr(NULL);  
 * Esta función se debe ejecutar después de ESP01_Init
 *
 * @param [in] aDbgStrPtrFun: puntero a función que recibe la nueva cadena de depuración. 
 *
 */
void ESP01_AttachDebugStr(void (*aESP01DbgStr)(const char *dbgStr));

/**
 * @brief ESP01_IsHDRRST indica el estado del Hard reset del ESP01
 *
 * Esta función indica el estado del Hard reset del ESP01. 
 * Esta función se debe ejecutar después de ESP01_Init
 *
 * @retVal Devuelve 0:Si no se esta haciendo un Hard reset, 1:Si se esta haciendo un Hard reset.
 *
 */
int ESP01_IsHDRRST();


#endif /* ESP01_H_ */
