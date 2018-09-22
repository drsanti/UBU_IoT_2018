 
#ifndef _ADC_H_
#define _ADC_H_

	#include "config.h"

    void ADC_Init(void);	
    int ADC_Get(unsigned char k);
    
    
    extern unsigned int ADC_Value[ADC_NUM_CHANNELS];
    
    #define ADC0_Get()  ADC_Value[0]
    #define ADC1_Get()  ADC_Value[1]
    #define ADC2_Get()  ADC_Value[2]
    #define ADC3_Get()  ADC_Value[3]

    #define ADC_Get(n) ADC_Value[n]
    void ADC_Service(void);
#endif

