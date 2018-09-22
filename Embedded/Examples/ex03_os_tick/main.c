/*
 ***************************************************************
 * Ex03. OS Tick
 ***************************************************************
 * Dr.Santi Nuratch
 * Embedded Computing and Control Laboratory (ECC-Lab)
 * Department of Control System and Instrumentation Engineering
 * Faculty of Engineering, KMUTT
 * 18 September, 2018
 ***************************************************************
 */

#include "os.h"

int main(void) {
    int tick_count = 0;
	OS_Init();                      // Initialise the OS
    while(1) {
        if( OS_TickedCheck() ) {    // Is the os ticked?
            LED0_Inv();             // Toggle LED0, output frequency is 500 Hz
            tick_count++;           // Increase the counter by 1
            if(tick_count >= 500) { // Is it 500 mS?
                tick_count = 0;     // Reset the counter
                LED_Inv(LED_ID_3);  // Invert the LED3
                                    // The output frequency is 1 Hz
            }
        }
    }
    OS_Start();      // Not required for this example
}
