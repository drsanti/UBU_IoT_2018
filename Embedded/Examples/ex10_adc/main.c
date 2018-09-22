/*
 ***************************************************************
 * Ex10. ADC
 ***************************************************************
 * Dr.Santi Nuratch
 * Embedded Computing and Control Laboratory (ECC-Lab)
 * Department of Control System and Instrumentation Engineering
 * Faculty of Engineering, KMUTT
 * 18 September, 2018
 ***************************************************************
 */

#include "os.h"

void Reader(void *a) {
    int ldr = ADC1_Get();
    char str[32];
    sprintf(str, "Light = %d\r\n", ldr);
    Uart1_AsyncWriteString(str);
}

void LED_0_Inv(void *a) {
    LED0_Inv();
}

void LED_3_Inv(void *a) {
    LED3_Inv();
}

void BeepSound(void *a) {
    Beep(100);
}

int main(void) {
    OS_Init();
    OS_TimerCreate("Light", 1000, 1, Reader);
    OS_TimerCreate("LED0", 100, 1, LED_0_Inv);
    OS_TimerCreate("LED3", 500, 1, LED_3_Inv);
    OS_TimerCreate("Beep", 2000, 1, BeepSound);
    OS_Start();
}
