/*
 ***************************************************************
 * Ex01. The basic usage of the ECC-OS
 ***************************************************************
 * Dr.Santi Nuratch
 * Embedded Computing and Control Laboratory (ECC-Lab)
 * Department of Control System and Instrumentation Engineering
 * Faculty of Engineering, KMUTT
 * 18 September, 2018
 ***************************************************************
 */


#include "os.h"

//!!Global variables
//!!...

//!! Other Functions
//!!...

int main(void) {
    
    //!!Variables and hardware initialization
    //!!...
	
    OS_Init();	// Initialise the OS

    //!!
    //!! Generate PWM signal, 1Hz, 10% duty on the LED0
    LED_PwmSet(LED_ID_0, 1000, 200, 100);
    LED_PwmSet(LED_ID_1, 1000, 400, 100);
    LED_PwmSet(LED_ID_2, 1000, 600, 100);
    LED_PwmSet(LED_ID_3, 1000, 800, 100);


    //!!
    //!! Beep for 0.1 seconds
    Beep(100);

    //!!Other OS's functions
    //!!...
	
    OS_Start();	// Start the OS
}
