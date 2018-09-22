/*
 ***************************************************************
 * Ex05. Timer Event Callback function
 ***************************************************************
 * Dr.Santi Nuratch
 * Embedded Computing and Control Laboratory (ECC-Lab)
 * Department of Control System and Instrumentation Engineering
 * Faculty of Engineering, KMUTT
 * 18 September, 2018
 ***************************************************************
 */


#include "os.h"

//!! Callback function
void Worker1(void *param) {
    LED_Toggle(LED_ID_1);   // Toggle the LED1
}

//!! Callback function
void Worker2(void *param) {
    LED_Toggle(LED_ID_3);   // Toggle the LED3
}

int main(void) {
	OS_Init();              // Initialise the OS

    //!! Create timer and its callback
    OS_TimerCreate(
            "Timer1",       // Timer name
            1000,           // time, 1000 mS
            TIMER_MODE_CONTINUEOUS, // mode 
            Worker1         // callback function
    );

    //!! Crdate timer and its callback
    OS_TimerCreate(
            "Timer2",       // Timer name
            50,             // time 50mS (20Hz)
            TIMER_MODE_CONTINUEOUS, // mode 
            Worker2         // callback function
    );

    OS_Start();             // Start the OS
}
