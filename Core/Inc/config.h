
#ifndef CONFIG_H
#define	CONFIG_H

#define AUTOCONNECTWIFI 1

/**
 * @brief Cadena constante para configurar el Wifi Automaticamente sin enviar datos 
 * desde la PC
 * 
 */
const unsigned char dataCwmode[]="AT+CWMODE_DEF=1\r\n";
const unsigned char dataCwdhcp[]="AT+CWDHCP_DEF=1,1\r\n";
const unsigned char dataCwjap[]="AT+CWJAP_DEF=\"WiFi Velasco\",\"ncgrmvelasco\"\r\n"; //<! Cambiara aqui el SSID y el PASS
//const unsigned char dataCwjap[]="AT+CWJAP_DEF=\"FCAL\",\"fcalconcordia.06-2019\"\r\n";
const unsigned char dataCipmux[]="AT+CIPMUX=0\r\n";
const unsigned char dataCipstart[]="AT+CIPSTART=\"UDP\",\"192.168.1.2\",30010,30001,0\r\n";
//unsigned char dataCipstart[]="AT+CIPSTART=\"UDP\",\"192.168.2.100\",30010,30001,0\r\n";//<! Cambiara aqui la IP por la de la PC a la que se quieran conectar via UDP
const unsigned char dataCipmode[]="AT+CIPMODE=1\r\n";
const unsigned char dataCipsend[]="AT+CIPSEND\r\n";


#endif