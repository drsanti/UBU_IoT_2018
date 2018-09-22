/*
 ***************************************************************
 * Ex08. UART Byte Received Callback
 ***************************************************************
 * Dr.Santi Nuratch
 * Embedded Computing and Control Laboratory (ECC-Lab)
 * Department of Control System and Instrumentation Engineering
 * Faculty of Engineering, KMUTT
 * 18 September, 2018
 ***************************************************************
 */

#include "os.h"

//!! UART1 Byte Received Callback
void UartRxCallback(void *evt)
{
    //!! Pointer to void to pointer to uart_event_t
    uart_event_t *uart_event = (uart_event_t *)evt;

    //!! Data structure
    data_t uart_data = uart_event->data;

    //!! Byte data
    uint8_t byte_data = uart_data.buffer[0];

    char buff[32];
    if(byte_data == '\r') {         // CR
        sprintf(buff, "\r\nByte Received Data: [CR]");
    }
    else if(byte_data == '\n') {    // LF
        sprintf(buff, "\r\nByte Received Data: [LF]");
    }
    else {                          // Character
        sprintf(buff, "\r\nByte Received Data: %c", byte_data);
    }

    UART1_AsyncWriteString(buff);

    //!! Simple control
    if(byte_data == '1') {          // Key '1' LED0 ON
        LED0_On();
    }
    else if(byte_data == '0') {     // Key '0' LED 0 OFF
        LED0_Off();
    }
}

int main(void) {
	OS_Init();
    
    //!! Register UART1 Rx Callback
    OS_Uart1SetRxCallback(UartRxCallback);

    OS_Start();
}
