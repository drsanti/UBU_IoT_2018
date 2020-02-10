/*
 ***************************************************************
 * Ex14. Example of Node-RED supported firmware
 ***************************************************************
 * Asst.Prof.Dr.Santi Nuratch
 * Embedded Computing and Control Laboratory (ECC-Lab)
 * Department of Control System and Instrumentation Engineering
 * Faculty of Engineering, KMUTT
 * 18 September, 2018
 *
 * Update: 08 November, 2019
 *  - Multiple commands are supported
 *
 ***************************************************************
 */

#include "os.h"

//!! Static commands buffer
#define CMD_MAX 10
#define CMD_LEN 32
char cmd_buff[CMD_MAX][CMD_LEN];
unsigned char cmd_put = 0;
unsigned char cmd_get = 0;
unsigned char cmd_cnt = 0;

//!! UART1 Line Received Callback
void UartLineCallback(void *evt)
{
    //!! Pointer to void to pointer to uart_event_t
    uart_event_t *uart_event = (uart_event_t *)evt;

    //!! Data structure
    data_t uart_data = uart_event->data;

    //!! Line data
    const char *line_data = (const char *)uart_data.buffer;

    //!!
    //!! Add the received command into the command buffer
    //!!

    //!! Check buffer space
    if (cmd_cnt >= CMD_MAX)
    {
        //!! Nore more space in the command buffer, ignore this command
        return;
    }
    //!! Count the received command
    cmd_cnt++;

    //!! Take a target slot of the command buffer
    char *ptr = cmd_buff[cmd_put];

    //!! Next index
    cmd_put = (cmd_put + 1) % CMD_MAX;

    //!! Copy
    unsigned char i;
    for (i = 0; i < CMD_LEN; i++)
    {
        char c = line_data[i];
        *ptr++ = c;
        if (c == 0)
        {
            return; //!! NULL terminator is detected
        }
    }
    //!! The received command too long, ignore it
    cmd_cnt = cmd_cnt > 0 ? cmd_cnt - 1 : 0;
    cmd_put = cmd_put > 0 ? cmd_put - 1 : 0;
}

//!! This callback function is shared for all switches (PBS<3:0>)
void SwitchCallback(void *param)
{
    //!! cast the event-data (param) to the event_t
    event_t *evt = (event_t *)param;

    //!! point to the switch object, the sender
    switch_t *psw = evt->sender;

    //!! Print information to the console
    char buff[32];

    sprintf(buff, "psw:%d:1\r\n", psw->id);
    UART1_AsyncWriteString(buff);
}
//!! Callback function
void Worker1(void *param)
{

    //!! Check if the command in the buffer
    if (cmd_cnt <= 0)
    {
        return;
    }

    //!! Take a command
    const char *line_data = cmd_buff[cmd_get];

    //!! Decrease the command counter
    cmd_cnt--;

    //!! Next command
    cmd_get = (cmd_get + 1) % CMD_MAX;

    //!!
    //!! led:[0,1,2,3]:[on,off]
    //!!led:n:?
    if (0 == str_index_of_first_token(line_data, "led:") && line_data[6] != '?')
    {
        char id_c = line_data[4];
        if (id_c >= '0' && id_c <= '3')
        {
            int id_i = id_c - 0x30;
            char *ptr = (char *)(line_data + 5);
            if (0 == str_index_of_first_token(ptr, ":on"))
            {
                LED_Set(id_i);
            }
            else if (0 == str_index_of_first_token(ptr, ":off"))
            {
                LED_Clr(id_i);
            }
            else if (0 == str_index_of_first_token(ptr, ":inv"))
            {
                LED_Inv(id_i);
            }
            else
            {
                UART1_AsyncWriteString("Command Error!: ");
                UART1_AsyncWriteString(line_data);
            }
        }
    }

    //!! Beep sound
    //!! beep:1000:2500 --> beep:<freq>:<period>
    if (0 == str_index_of_first_token(line_data, "beep:"))
    {
        int i1 = str_index_of_first_token(line_data + 0, ":");                // Index of the first colon
        int i2 = str_index_of_first_token(line_data + i1 + 1, ":") + i1 + 1;  // Index of the second colon
        int i3 = str_index_of_first_token(line_data + i2 + 1, "\r") + i2 + 1; // Index of the \r

        //!! Buffer
        char temp[32];

        //!!
        //!! Frequency (0, 2000)
        //!!
        memset(temp, 0, 32);
        memcpy(temp, line_data + i1 + 1, i2 - i1 - 1);
        float freq = atof(temp);

        //!!
        //!! Period (in mS)
        //!!
        memset(temp, 0, 32);
        memcpy(temp, line_data + i2 + 1, i3 - i2 - 1);
        float period = atof(temp);
        //sprintf(buff, "period: %3.3f\r\n", period); UART1_AsyncWriteString(buff);

        //!! Beep
        Beep_FreqSet(freq);
        Beep((uint16_t)period);
    }

    //!! ADCs
    //!! adc:[0,1,2,3]?
    if (0 == str_index_of_first_token(line_data, "adc:"))
    {
        char buff[32];
        if (line_data[5] == ':' && line_data[6] == '?')
        {
            char id_c = line_data[4];
            if (id_c >= '0' && id_c <= '3')
            {
                int id_i = id_c - 0x30;
                int adc_val = ADC_Get(id_i);
                sprintf(buff, "adc:%d:%d\r\n", id_i, adc_val);
                UART1_AsyncWriteString(buff);
            }
        }
    }
    //!! PSWs
    //!! psw:[0,1,2,3]?
    if (0 == str_index_of_first_token(line_data, "psw:"))
    {
        char buff[32];
        if (line_data[5] == ':' && line_data[6] == '?')
        {
            char id_c = line_data[4];
            if (id_c >= '0' && id_c <= '3')
            {
                int id_i = id_c - 0x30;
                int psw_val = PSW_Get(id_i);
                sprintf(buff, "psw:%d:%d\r\n", id_i, psw_val);
                UART1_AsyncWriteString(buff);
            }
        }
    }
    //!! LEDs
    //!! led:[0,1,2,3]?
    if (0 == str_index_of_first_token(line_data, "led:"))
    {
        char buff[32];
        if (line_data[5] == ':' && line_data[6] == '?')
        {
            char id_c = line_data[4];
            if (id_c >= '0' && id_c <= '3')
            {
                int id_i = id_c - 0x30;
                int led_val = LED_Get(id_i);
                sprintf(buff, "led:%d:%d\r\n", id_i, led_val);
                UART1_AsyncWriteString(buff);
            }
        }
    }
}

int main(void)
{
    OS_Init();

    //!! Register UART1 Line Received Callback
    OS_Uart1SetLineReceivedCallback(UartLineCallback);

    //!! The events can be registered in the for loop like this
    int id;
    for (id = 0; id < 4; id++)
    {
        OS_SwitchSetCallback(id, SwitchCallback);
    }

    //!! Create timer and its callback
    OS_TimerCreate(
        "Timer1",               // Timer name
        10,                     // time, 20 mS
        TIMER_MODE_CONTINUEOUS, // mode
        Worker1                 // callback function
    );

    Beep(50);
    LED0_On();
    LED1_On();
    LED2_On();
    LED3_On();
    __delay_ms(200);
    LED0_Off();
    LED1_Off();
    LED2_Off();
    LED3_Off();

    OS_Start();
}
