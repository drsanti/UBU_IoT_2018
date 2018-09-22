/*
 ***************************************************************
 * Ex07. Switch Event Callback function and Event Data
 ***************************************************************
 * Dr.Santi Nuratch
 * Embedded Computing and Control Laboratory (ECC-Lab)
 * Department of Control System and Instrumentation Engineering
 * Faculty of Engineering, KMUTT
 * 18 September, 2018
 ***************************************************************
 */

#include "os.h"

//!! This callback function is shared for all switches (PBS<3:0>)
void SwitchCallback(void *param) {
    //!! cast the event-data (param) to the event_t
    event_t *evt = (event_t *)param;

    //!! point to the switch object, the sender
    switch_t *psw = evt->sender;
    
    //!! Print information to the console
    char buff[32];

    //!! [1] using psw->id
    sprintf(buff, "\r\nPSW%d is pressed", psw->id);
    //!! [2] using evt->data.buffer[0]
    //sprintf(buff, "\r\nPSW%d is pressed", evt->data.buffer[0]);
    UART1_AsyncWriteString(buff);

    //!! Toggle the LED pointed by the Id of the switch
    LED_Toggle(psw->id);
}

int main(void) {
	OS_Init();          // Initialise the OS
    Uart1_AsyncWriteString("Press PSW and check the result on the terminal\r\n");
    Beep(100);
    int id;
    for(id=0; id<4; id++) {
        OS_SwitchSetCallback(id, SwitchCallback);
    }
    OS_Start();         // Start the OS
}
