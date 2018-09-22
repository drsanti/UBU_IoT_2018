/*
 ***************************************************************
 * Ex04. OS Callback Function
 ***************************************************************
 * Dr.Santi Nuratch
 * Embedded Computing and Control Laboratory (ECC-Lab)
 * Department of Control System and Instrumentation Engineering
 * Faculty of Engineering, KMUTT
 * 18 September, 2018
 ***************************************************************
 */

#include "os.h"

//!! Callback function of the os ticker
void os_tick_callback(void *param) {
    LED_Toggle(LED_ID_0);

    //!! This callback function is directly called from os ticker
    //!! Do not put long execution code in this callback function
}

int main(void) {
	OS_Init();      // Initialise the OS

    //!! Register to system tick callback
    //!! In this example, the os_tick_callback will be called every 100 mS
    OS_SystemTickSetCallback(100, os_tick_callback);

    OS_Start();     // Not required for this example
}
