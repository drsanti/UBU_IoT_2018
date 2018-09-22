/*
 ***************************************************************
 * Ex06. Switch Event Callback function
 ***************************************************************
 * Dr.Santi Nuratch
 * Embedded Computing and Control Laboratory (ECC-Lab)
 * Department of Control System and Instrumentation Engineering
 * Faculty of Engineering, KMUTT
 * 18 September, 2018
 ***************************************************************
 */


#include "os.h"

void WorkerON(void *param) {
    LED_Set(LED_ID_0);  // Turn ON LED0
}
void WorkerOFF(void *param) {
    LED_Clr(LED_ID_0);   // Turn OFF LED0
}

int main(void) {
	OS_Init();          // Initialise the OS

    Uart1_AsyncWriteString("Press PSW and check the result on the LED0\r\n");
    Beep(1000);

    //!! Register the event of the PSW0
    OS_SwitchSetCallback(PSW_ID_0, WorkerON);

    //!! Register the event of the PSW3
    OS_SwitchSetCallback(PSW_ID_3, WorkerOFF);

    OS_Start();         // Start the OS
}



