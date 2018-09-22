/************************************************************
 * File:    uart.h                                          *
 * Author:  Dr.Santi Nuratch                                *
 *          Embedded Computing and Control Laboratory       *
 * Update:  12 July  2017, 03.08 PM                         *
 * Update:  10 March 2018, 08.29 AM                         *
 ************************************************************/

#ifndef _UART_H_
#define _UART_H_

	#include "config.h"
    #include "data.h"
    #include "queue.h"

    /* U1 RX */
    typedef struct {
        uint8_t data;   // byte received data
        uint8_t flag;   // byte received flag
        uint8_t buffer[U1RX_BUFF_LENGTH];   // buffer for receiving queue
        //uint8_t *buffer; 
        Queue   queue;  // queue object 
    }uart1_rx_t;

    /* U1 TX */
    typedef struct {
        uint8_t data;   // byte data for transmission
        uint8_t flag;   // byte transmitted flag
        uint8_t empty;  // transmission buffer empty
        uint8_t buffer[U1TX_BUFF_LENGTH];   // buffer for transmission queue
        //uint8_t *buffer; 
        Queue queue;    // queue object 
    }uart1_tx_t;

    /* U1 TX,TX */
    typedef struct {
        uart1_rx_t rx;
        uart1_tx_t tx;
    }uart1_t;


    /* U2 RX */
    typedef struct {
        uint8_t data;   // byte received data
        uint8_t flag;   // byte received flag
        uint8_t buffer[U2RX_BUFF_LENGTH];   // buffer for receiving queue
        //uint8_t *buffer; 
        Queue queue;    // queue object 
    }uart2_rx_t;

    /* U2 TX */
    typedef struct {
        uint8_t data;       // byte data for transmission
        uint8_t flag;       // byte transmitted flag
        uint8_t empty;      // transmission buffer empty
        uint8_t buffer[U2TX_BUFF_LENGTH]; // buffer for transmission queue
        //uint8_t *buffer; 
        Queue queue;        // queue object 
    }uart2_tx_t;

    typedef struct {
        uart2_rx_t rx;
        uart2_tx_t tx;
    }uart2_t;
    
    
    /* U1 RING */
    typedef struct {
        uint8_t     buf[U1RX_RING_BUFF_LEN];
        uint16_t    put;
        uint16_t    get;
        uint16_t    cnt;
    }uart1_ring_t;

    /* U2 RING */
    typedef struct {
        uint8_t     buf[U2RX_RING_BUFF_LEN];
        uint16_t    put;
        uint16_t    get;
        uint16_t    cnt;
    }uart2_ring_t;


	
    // U1_TX
    #define UART1_TX_ISR_ENABLE()       (IEC0bits.U1TXIE = 1)
    #define UART1_TX_ISR_DISABLE()      (IEC0bits.U1TXIE = 0)
    
    // U1_RX
    #define UART1_RX_ISR_ENABLE()       (IEC0bits.U1RXIE = 1)
    #define UART1_RX_ISR_DISABLE()      (IEC0bits.U1RXIE = 0)



    // U2_TX
    #define UART2_TX_ISR_ENABLE()       (IEC1bits.U2TXIE = 1)
    #define UART2_TX_ISR_DISABLE()      (IEC1bits.U2TXIE = 0)
    
    // U2_RX
    #define UART2_RX_ISR_ENABLE()       (IEC1bits.U2RXIE = 1)
    #define UART2_RX_ISR_DISABLE()      (IEC1bits.U2RXIE = 0)


    #define UART_ID_1   1
    #define UART_ID_2   2
    
	void Uart1_Init(uint32_t baurate);
	void Uart2_Init(uint32_t baurate);
    #define Uart_Init(id, baurate){ if(id==UART_ID_1){Uart1_Init(baurate);}else if(id==UART_ID_2){Uart2_Init(baurate);}}
    
    #define UART1_Init  Uart1_Init
    #define UART2_Init  Uart2_Init
    #define UART_Init   Uart_Init

    #define uart1_init UART1_Init  
    #define uart2_init UART2_Init  
    #define uart_init  UART_Init
    
    //#define Uart_Init(id, br) Uart ## id ## _Init(br)
    //#define UART_Init(id, br) Uart ## id ## _Init(br)
    
    uint16_t Uart1_AsyncPutByte(const uint8_t data);
    uint16_t Uart2_AsyncPutByte(const uint8_t data);
    
    #define UART1_AsyncPutByte Uart1_AsyncPutByte
    #define UART2_AsyncPutByte Uart2_AsyncPutByte
    
    #define Uart_AsyncPutByte(id, d){ if(id==UART_ID_1){Uart1_AsyncPutByte(d);}else if(id==UART_ID_2){Uart2_AsyncPutByte(d);}}
    #define UART_AsyncPutByte Uart_AsyncPutByte

    #define uart_async_put_byte  Uart_AsyncPutByte
    #define uart1_async_put_byte Uart1_AsyncPutByte
    #define uart2_async_put_byte Uart2_AsyncPutByte

    
    uint16_t Uart1_AsyncWriteData(const data_t *data);
    uint16_t Uart2_AsyncWriteData(const data_t *data);
    
    #define UART1_AsyncWriteData Uart1_AsyncWriteData
    #define UART2_AsyncWriteData Uart2_AsyncWriteData

    #define Uart_AsyncWriteData(id, data){ if(id==UART_ID_1){UART1_AsyncWriteData(data);}else if(id==UART_ID_2){UART1_AsyncWriteData(data);}}
    #define UART_AsyncWriteData Uart_AsyncWriteData

    #define uart_async_write_data  Uart_AsyncWriteData
    #define uart1_async_write_data Uart1_AsyncWriteData
    #define uart2_async_write_data Uart2_AsyncWriteData

    
    uint16_t Uart1_AsyncWriteString(const char *message);
    uint16_t Uart2_AsyncWriteString(const char *message);
    #define Uart_AsyncWriteString(id, msg){ if(id==UART_ID_1){Uart1_AsyncWriteString(msg);}else if(id==UART_ID_2){Uart2_AsyncWriteString(msg);}}
    
    uint16_t Uart1_AsyncWriteBytes(const unsigned char *data, unsigned int length);
    uint16_t Uart2_AsyncWriteBytes(const unsigned char *data, unsigned int length);
    #define Uart_AsyncWriteBytes(id, msg){ if(id==UART_ID_1){Uart1_AsyncWriteBytes(msg);}else if(id==UART_ID_2){Uart1_AsyncWriteBytes(msg);}}

    #define UART1_AsyncWriteString Uart1_AsyncWriteString
    #define UART1_AsyncWriteBytes  Uart1_AsyncWriteBytes
    #define UART_AsyncWriteString  Uart_AsyncWriteString
    #define UART_AsyncWriteBytes   Uart_AsyncWriteBytes

    #define uart1_async_write_string Uart1_AsyncWriteString
    #define uart1_async_write_bytes  Uart1_AsyncWriteBytes
    #define uart_async_write_string  Uart_AsyncWriteString
    #define uart_async_write_bytes   Uart_AsyncWriteBytes

    void Uart1_ClearRingBuffer(void);
    void Uart2_ClearRingBuffer(void);
    
    
    //-----------------------
    void Uart1_WriteString(const char *str);
    void Uart2_WriteString(const char *str);

    void Uart1_PutByte(const unsigned char data);
    void Uart2_PutByte(const unsigned char data);

    void Uart1_WriteBytes(const unsigned char *data, unsigned int length);
    void Uart2_WriteBytes(const unsigned char *data, unsigned int length);

    unsigned char Uart1_GetByte(void);
    unsigned char Uart2_GetByte(void);
    
#endif

