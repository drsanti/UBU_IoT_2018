/*
 ***************************************************************
 * Ex14. Example of Node-RED supported firmware
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


    char buff[32];

    //!!
    //!! led:[0,1,2.3]:[on,off]
    //!!
    if( 0 == str_index_of_first_token(line_data, "led:") ) {
        char id_c = line_data[4];
        if(id_c >= '0' && id_c <= '3') {
            int id_i = id_c-0x30;
            char *ptr = (char *)(line_data+5);
            if( 0 == str_index_of_first_token(ptr, ":on") ) {
                LED_Set(id_i);
            } 
            else if( 0 == str_index_of_first_token(ptr, ":off")) {
                LED_Clr(id_i);
            } 
            else if( 0 == str_index_of_first_token(ptr, ":inv")) {
                LED_Inv(id_i);
            } 
            else{
                UART1_AsyncWriteString("Command Error!: "); 
                UART1_AsyncWriteString(line_data);  
            }
        }
     }

    //!! Beep sound
    else if( 0 == str_index_of_first_token(line_data, "beep:") ) {
        int i1 = str_index_of_first_token(line_data+0, ":");                // Index of the first colon
        int i2 = str_index_of_first_token(line_data+i1+1, ":") + i1 + 1;    // Index of the second colon
        int i3 = str_index_of_first_token(line_data+i2+1, "\r")+ i2 + 1;    // Index of the \r

        //!! Buffer
        char temp[32];

        //!!
        //!! Frequency (0, 2000)
        //!!
        memset(temp, 0, 32);
        memcpy(temp, line_data+i1+1, i2-i1-1);
        float freq = atof(temp);
        
        //!!
        //!! Period (in mS)
        //!!
        memset(temp, 0, 32);
        memcpy(temp, line_data+i2+1, i3-i2-1);
        float period = atof(temp);
        //sprintf(buff, "period: %3.3f\r\n", period); UART1_AsyncWriteString(buff);

        //!! Beep
        Beep_FreqSet(freq);
        Beep((uint16_t)period);
    }

    //!! ADCs
    else if( 0 == str_index_of_first_token(line_data, "adc:") ) {

        if(line_data[5] == '?') {
            char id_c = line_data[4];
            if(id_c >= '0' && id_c <= '3') {
                int id_i = id_c-0x30;   
                int adc_val = ADC_Get(id_i);
                sprintf(buff, "adc:%d:%d\r\n", id_i, adc_val);
                UART1_AsyncWriteString(buff);  
            }
        }
    }
}


//!! This callback function is shared for all switches (PBS<3:0>)
void SwitchCallback(void *param) {
    //!! cast the event-data (param) to the event_t
    event_t *evt = (event_t *)param;

    //!! point to the switch object, the sender
    switch_t *psw = evt->sender;
    
    //!! Print information to the console
    char buff[32];

    sprintf(buff, "psw:%d\r\n", psw->id);
    UART1_AsyncWriteString(buff);

}

int main(void) {
    OS_Init();
    
    //!! Register UART1 Line Received Callback
    OS_Uart1SetLineReceivedCallback(UartLineCallback);

    //!! The events can be registered in the for loop like this
    int id;
    for(id=0; id<4; id++) {
        OS_SwitchSetCallback(id, SwitchCallback);
    }

    Beep(100);

    OS_Start();
}
