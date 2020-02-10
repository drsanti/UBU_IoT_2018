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
 * Update: 10 February, 2020
 *  - Add `fls` function -> fls:<id><period>
 *  - Add `pwm` function -> pwm:<id><period><shift><on_time>
 *  - Add `det` function -> det:<id><period><threshold>
 *  - Add `RESPONSE_OK` and `RESPONSE_ERROR` functions
 *
 ***************************************************************
 */

#include "os.h"

//!! Static commands buffer
#define CMD_MAX 20
#define CMD_LEN 32
char cmd_buff[CMD_MAX][CMD_LEN];
unsigned char cmd_put = 0;
unsigned char cmd_get = 0;
unsigned char cmd_cnt = 0;



#define UPDATE_INTERVAL_MS 10

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

inline void RESPONSE_ERROR(const char *msg)
{
    UART1_AsyncWriteString("{err:\"");
    Uart1_AsyncWriteBytes((const unsigned char *)msg, strlen(msg) - 2);
    UART1_AsyncWriteString("\"}\r\n");
}

inline void RESPONSE_OK(const char *msg)
{
    UART1_AsyncWriteString("{ok:\"");
    Uart1_AsyncWriteBytes((const unsigned char *)msg, strlen(msg) - 2);
    UART1_AsyncWriteString("\"}\r\n");
}




int _adc_val[4];            // Initialized in the main()
int _adc_old[4];            // Initialized in the main()
int _adc_del[4];            // Delta values
uint16_t _adc_thv[4]    = {20,   20,  20,  20};      // Default threshold values
uint16_t _adc_period[4] = {250, 250, 250, 250};      // 250 mS, 0 means disabled
uint16_t _adc_ticks[4]  = {0,     0,   0,   0};      // reset



//!! Callback function
void Worker1(void *param)
{

    // 10 ms, sampling
    int i;
    for(i=0; i<4; i++) {
        _adc_val[i] = (ADC_Get(i) + _adc_val[i]) >> 1;
    }

    // Loop over all adc channels
    for(i=0; i<4; i++) {

        // Disabled
        if(_adc_period[i] == 0) {
            continue;
        }

        // Increase the ticks
        _adc_ticks[i] += UPDATE_INTERVAL_MS;

        if(_adc_ticks[i] >= _adc_period[i]) {

            // Reset the ticks
            _adc_ticks[i] = 0;

            // Delta value
            _adc_del[i] = _adc_val[i] - _adc_old[i];

            // Direction (+1: inc, -1: dec)
            char dir = 0;

            // Decreasing
            if(_adc_del[i] < 0) {

                _adc_del[i] *= -1;              // Positive
                if(_adc_del[i] > _adc_thv[i]) {
                    _adc_old[i] = _adc_val[i];  // Save
                    dir = -1;                   // DecChanged
                }
            }
            // Increasing
            else {
                if(_adc_del[i] > _adc_thv[i]) {
                    _adc_old[i] = _adc_val[i];  // Save
                    dir = +1;                   //IncChanged
                }
            }
            // Print
            if( dir != 0) {
                char buff[32];
                sprintf(buff, "det:%i:%i:%s\r\n", i, _adc_val[i], (dir<0?"dec":dir>0?"inc":""));
                UART1_AsyncWriteString(buff);
            }
        }
    }


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
    if ((0 == str_index_of_first_token(line_data, "led:")) && (line_data[6] != '?'))
    {
        char ok = 0;

        char id_c = line_data[4];
        if (id_c >= '0' && id_c <= '3')
        {
            int id_i = id_c - 0x30;

            ok = 1;

            LED_ModeSet(id_i, LED_MODE_NORMAL);

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
               //ok = 0;
            }
        }else {
            //ok = 0;
        }
        if(ok!=0) {
            RESPONSE_OK(line_data);
        }else{
            RESPONSE_ERROR(line_data);
        }
    }


    //!!
    //!! fls:[0,1,2,3]:[period]
    //!! fls:0:200
    else if ((0 == str_index_of_first_token(line_data, "fls:")) && (line_data[6] != '?'))
    {
        // Character id
        char id_c = line_data[4];
        if (id_c >= '0' && id_c <= '3')
        {
            // Numeric id
            int id_i = id_c - 0x30;
            LED_ModeSet(id_i, LED_MODE_NORMAL);

            // Indexes
            int i1 = str_index_of_first_token(line_data + 0, ":");                // Index of the first colon
            int i2 = str_index_of_first_token(line_data + i1 + 1, ":") + i1 + 1;  // Index of the second colon
            int i3 = str_index_of_first_token(line_data + i2 + 1, "\r") + i2 + 1; // Index of the \r

            //!! Buffer
            char temp[32];

            // Period
            memset(temp, 0, 32);
            memcpy(temp, line_data + i2 + 1, i3 - i2 - 1);
            float period = atof(temp);
            LED_Flash(id_i, (uint16_t)period);

            RESPONSE_OK(line_data);
        }
        else
        {
            RESPONSE_ERROR(line_data);
        }
    }


    //!!
    //!! det:<id><period><threshold>
    //!! det:3:1:30
    else if ((0 == str_index_of_first_token(line_data, "det:")) && (line_data[6] != '?'))
    {
        // Numeric id
        int id = line_data[4] - 0x30;
        if (id >= 0 && id <= 3)
        {

            // Indexes
            int i0 = 3;
            int i1 = str_index_of_first_token(line_data + i0 + 1, ":")  + i0 + 1;    // Index of the first colon
            int i2 = str_index_of_first_token(line_data + i1 + 1, ":")  + i1 + 1;    // Index of the second colon
            int i3 = str_index_of_first_token(line_data + i2 + 1, "\r") + i2 + 1;    // Index of the third colon


            // Buffer
            char temp[32];

            //!!
            //!! period
            //!!
            memset(temp, 0, 32);
            memcpy(temp, line_data + i1 + 1, i2 - i1 - 1);
            float period = atof(temp);

            //!!
            //!! threshold
            //!!
            memset(temp, 0, 32);
            memcpy(temp, line_data + i2 + 1, i3 - i2 - 1);
            float threshold = atof(temp);

            _adc_period[id]= (uint16_t)period;
            _adc_thv[id] = (uint16_t)threshold;


            //sprintf(temp, "%d %d\r\n", (uint16_t)period, (uint16_t)threshold);
            //UART1_AsyncWriteString(temp);

            RESPONSE_OK(line_data);
        }
        else
        {
            RESPONSE_ERROR(line_data);
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

        RESPONSE_OK(line_data);
    }

    //!! PWM
    //!! pwm:0:1000:2500 --> beep:<id><period>:<shift><ontime>
    //!! void LED_PwmSet(uint8_t id, uint16_t period_ticks, uint16_t shift_ticks, uint16_t on_ticks);
    if (0 == str_index_of_first_token(line_data, "pwm:"))
    {

        char id = line_data[4]-0x30;
        if (id >= 0 && id <= 3) {

            int i0 = 3;
            int i1 = str_index_of_first_token(line_data + i0 + 1, ":") + i0 + 1;    // Index of the first colon
            int i2 = str_index_of_first_token(line_data + i1 + 1, ":") + i1 + 1;    // Index of the second colon
            int i3 = str_index_of_first_token(line_data + i2 + 1, ":") + i2 + 1;    // Index of the third colon
            int i4 = str_index_of_first_token(line_data + i3 + 1, "\r") + i3 + 1;   // Index of the \r

            //!! Buffer
            char temp[32];

            //!!
            //!! period
            //!!
            memset(temp, 0, 32);
            memcpy(temp, line_data + i1 + 1, i2 - i1 - 1);
            float period = atof(temp);

            //!!
            //!! shift
            //!!
            memset(temp, 0, 32);
            memcpy(temp, line_data + i2 + 1, i3 - i2 - 1);
            float shift = atof(temp);

            //!!
            //!! on_time
            //!!
            memset(temp, 0, 32);
            memcpy(temp, line_data + i3 + 1, i4 - i3 - 1);
            float ontime = atof(temp);

            LED_ModeSet(id, LED_MODE_PWM);

            //!! PWM
            LED_PwmSet(id, (uint16_t)period, (uint16_t)shift, (uint16_t)ontime);

            RESPONSE_OK(line_data);
        }
        else {
            RESPONSE_ERROR(line_data);
        }
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
        else {
            UART1_AsyncWriteString("Command Error!: ");
            UART1_AsyncWriteString(line_data);
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
        else {
            UART1_AsyncWriteString("Command Error!: ");
            UART1_AsyncWriteString(line_data);
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
        else {
            UART1_AsyncWriteString("Command Error!: ");
            UART1_AsyncWriteString(line_data);
        }
    }
}



int main(void)
{
    // Clear buffer
    int i, j;

    for(i=0; i<CMD_MAX; i++) {
        for (j = 0; j < CMD_LEN; j++)
        {
            cmd_buff[i][j] = 0;
        }
    }

    OS_Init();

    // Test
    Beep_FreqSet(4000);
    Beep(5);
    LED_Write(0x0F);
    __delay_ms(200);
    LED_Write(0x00);

    //!! Register UART1 Line Received Callback
    OS_Uart1SetLineReceivedCallback(UartLineCallback);

    //!! The events can be registered in the for loop like this
    int id;
    for (id = 0; id < 4; id++)
    {
        OS_SwitchSetCallback(id, SwitchCallback);
    }

    //!! Clear ADC's buffer
    for(i=0; i<4; i++) {
        _adc_val[i] = ADC_Get(i);
        _adc_old[i] = _adc_val[i];
    }

    //!! Create timer and its callback
    OS_TimerCreate(
        "Timer1",               // Timer name
        UPDATE_INTERVAL_MS,     // time, 10 mS
        TIMER_MODE_CONTINUEOUS, // mode
        Worker1                 // callback function
    );

    UART1_AsyncWriteString("\r\nReady\r\n");
    OS_Start();
}
