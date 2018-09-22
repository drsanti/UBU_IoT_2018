/*
 ***************************************************************
 * Ex09. UART Line Received Callback
 ***************************************************************
 * Dr.Santi Nuratch
 * Embedded Computing and Control Laboratory (ECC-Lab)
 * Department of Control System and Instrumentation Engineering
 * Faculty of Engineering, KMUTT
 * 18 September, 2018
 ***************************************************************
 */

#include "os.h"

//!! UART1 Line Received Callback
void UartLineCallback(void *evt)
{
    //!! Pointer to void to pointer to uart_event_t
    uart_event_t *uart_event = (uart_event_t *)evt;

    //!! Data structure
    data_t uart_data = uart_event->data;

    //!! Line data
    const char *line_data = (const char *)uart_data.buffer;

    //!! Print received data
    char buff[32];
    sprintf(buff, "Received %d bytes: %s", uart_data.length, line_data);
    UART1_AsyncWriteString(buff);

    //!! Simple commands to control LED0 and Beep
    if(!strcmp(line_data, "led0-on\r\n")) {
        LED0_On();      // LED0 ON
    }
    else if(!strcmp(line_data, "led0-off\r\n")) {
         LED0_Off();    // LED0 OFF
    }
    else if(!strcmp(line_data, "beep\r\n")) {
         Beep(20);      // Beep
    }
}

int main(void) {
    OS_Init();
    
    //!! Register UART1 Line Received Callback
    OS_Uart1SetLineReceivedCallback(UartLineCallback);

    OS_Start();
}
