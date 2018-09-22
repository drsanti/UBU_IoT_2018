#ifndef __BEEP_H__
#define __BEEP_H__

#include "config.h"

void Beep_Init(void);
void Beep_FreqSet(float freq);
void Beep_PowerSet(float power /* 0.0 - 1*/); 
void Beep(unsigned int period);
void Beep_Service(void);
#endif