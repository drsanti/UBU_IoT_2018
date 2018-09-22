/************************************************************
 * File:    config.h                                        *
 * Author:  Dr.Santi Nuratch                                *
 *          Embedded Computing and Control Laboratory       *
 * Update:  05 July 2017, 12.36 PM                          *
 ************************************************************/


#ifndef __CONFIG_H__
#define	__CONFIG_H__


    #include <xc.h>
    //#include <p24Fxxxx.h>
    #include <p24FJ48GA002.h>
    #include <stdio.h>
    #include <stdlib.h>
    #include <stdint.h>
    #include <string.h>

#ifndef bool
    #define bool uint16_t
#endif

typedef uint16_t bool_t;

#ifndef true
#define true (1)
#endif

#ifndef false
#define false (0)
#endif

//!!
//!! Processor Clock
//!!
#define FOSC 32e6
#define FCY (FOSC / 2)

//!!
//!! This library must be defined after FCY. It uses the FCY as input parameter
//!!
#include <libpic30.h>

#define TIME_PER_TICK 1e-3

//##
//## Buffer/Queue for computer
//##
#define U1TX_BUFF_LENGTH 512        // Computer transmission buffer
#define U1RX_BUFF_LENGTH 64         // Computer receiving buffer

//##
//## Buffer/Queue for esp8266
//##
#define U2TX_BUFF_LENGTH 1024       // WiFi transmission buffer
#define U2RX_BUFF_LENGTH 1024       // WiFi receiving buffer

//##
//## These buffers are used for RxReceived of UART1 and UART2 callbacks
//##
#define U1RX_RING_BUFF_LEN 128      // Computer ring-buffer
#define U2RX_RING_BUFF_LEN 1024     // WiFi ring-buffer

//##
//## UART baudrate/speed
//##
//#define UART1_BAUDRATE		115200  // Computer baudrate
//#define UART1_FASTSPEED		1

//#define UART2_BAUDRATE		115200  // ESP8266 baudrate
//#define UART2_FASTSPEED		1

// ADC
#define ADC_NUM_CHANNELS 4
    
#endif	

