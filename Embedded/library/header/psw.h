#ifndef __PSW_H__
#define __PSW_H__

#include "config.h"
#include "mcu.h"

#define PSW_NUM         4       // Number of switches

#define PSW_ID_0        0       // PB Switch #0
#define PSW_ID_1        1       // PB Switch #1
#define PSW_ID_2        2       // PB Switch #2
#define PSW_ID_3        3       // PB Switch #3

#define PSW_LOGIC_ON    0       // Active low
#define PSW_LOGIC_OFF   1
#define PSW_ALL_OFF     0x0F    // Switches are connected to lower-LSB

#define PSW_PORT        PORTB   
#define PSW_TRIS_INP    1

#define PSW0_TRIS   TRISBbits.TRISB4
#define PSW1_TRIS   TRISBbits.TRISB5
#define PSW2_TRIS   TRISBbits.TRISB6
#define PSW3_TRIS   TRISBbits.TRISB7

#define PSW0_PORT   PORTBbits.RB4
#define PSW1_PORT   PORTBbits.RB5
#define PSW2_PORT   PORTBbits.RB6
#define PSW3_PORT   PORTBbits.RB7


#define PSW0_Get()  (!PSW0_PORT)
#define PSW1_Get()  (!PSW1_PORT)
#define PSW2_Get()  (!PSW2_PORT)
#define PSW3_Get()  (!PSW3_PORT)



typedef void (*switch_callback_t)(void *);

#define PSW_STATUS_OFF              0
#define PSW_STATUS_ON               1
#define PSW_STATUS_HOLD_SLOW        2
#define PSW_STATUS_HOLD_FAST        3


#define DEBOUNCE_PERIOD            20       // debouncing period
#define WAIT_FOR_REPEAT_PERIOD     1000     // waiting interval before change to PSW_SM_REPEAT  
#define REPEAT_PULSE_PERIOD        240      // repeating interval, will be decrease to 50 internally


typedef struct {
    uint8_t     state;          // state machine
    uint8_t     id;             // id
    uint16_t    counter;        // number of repeating, including repeat-ticks
    uint8_t     status;         // status
    uint16_t    ticks;          // counter for all on-state, used internally
    switch_callback_t callback; // switch callback
}switch_t;

void PSW_Init(void);

unsigned char PSW_Get(unsigned char id);

unsigned char PSW_Scan(void);

unsigned char PSW_Read(void);

unsigned char PSW_Check(void);

int16_t OS_SwitchSetCallback(uint8_t id, switch_callback_t callback);

unsigned char PSW_Service(void);

#endif
