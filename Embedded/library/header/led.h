/************************************************************
 * File:    led.h                                           *
 * Author:  Dr.Santi Nuratch                                *
 *          Embedded Computing and Control Laboratory       *
 * Update:  05 July 2017, 12.23 PM                          *
 ************************************************************/


#ifndef __LED_H__
#define	__LED_H__

#include "mcu.h"

#define NUM_LED         4


#define LED_ID_0        0
#define LED_ID_1        1
#define LED_ID_2        2
#define LED_ID_3        3

#define LED_ON          0   
#define LED_OFF         1

#define LED_TRIS_OUT    0
#define LED_TRIS_INP    1

#define LED0_TRIS   TRISAbits.TRISA2
#define LED1_TRIS   TRISAbits.TRISA4
#define LED2_TRIS   TRISBbits.TRISB2
#define LED3_TRIS   TRISBbits.TRISB3

#define LED0_LAT    LATAbits.LATA2
#define LED1_LAT    LATAbits.LATA4
#define LED2_LAT    LATBbits.LATB2
#define LED3_LAT    LATBbits.LATB3

#define LED0_PORT   PORTAbits.RA2
#define LED1_PORT   PORTAbits.RA4
#define LED2_PORT   PORTBbits.RB2
#define LED3_PORT   PORTBbits.RB3


/*
 * Turn LED0 on
 */
#define LED0_Set()  (LED0_LAT=LED_ON)
#define LED1_Set()  (LED1_LAT=LED_ON)
#define LED2_Set()  (LED2_LAT=LED_ON)
#define LED3_Set()  (LED3_LAT=LED_ON)

#define LED0_On()   LED0_Set() 
#define LED1_On()   LED1_Set() 
#define LED2_On()   LED2_Set() 
#define LED3_On()   LED3_Set() 

#define LED0_Clr()  (LED0_LAT=LED_OFF)
#define LED1_Clr()  (LED1_LAT=LED_OFF)
#define LED2_Clr()  (LED2_LAT=LED_OFF)
#define LED3_Clr()  (LED3_LAT=LED_OFF)

#define LED0_Off()  LED0_Clr()
#define LED1_Off()  LED1_Clr()
#define LED2_Off()  LED2_Clr()
#define LED3_Off()  LED3_Clr()

#define LED0_Inv()  (LED0_LAT=!LED0_LAT)
#define LED1_Inv()  (LED1_LAT=!LED1_LAT)
#define LED2_Inv()  (LED2_LAT=!LED2_LAT)
#define LED3_Inv()  (LED3_LAT=!LED3_LAT)

#define LED0_Toggle()   LED0_Inv()
#define LED1_Toggle()   LED1_Inv()
#define LED2_Toggle()   LED2_Inv()
#define LED3_Toggle()   LED3_Inv()

#define LED0_Get()  (!LED0_PORT)
#define LED1_Get()  (!LED1_PORT)
#define LED2_Get()  (!LED2_PORT)
#define LED3_Get()  (!LED3_PORT)


//!! OPERATION MODE
#define LED_MODE_NORMAL     0
#define LED_MODE_PWM        1


#define LED_PWM_ON          0   // Active Low
#define LED_PWM_OFF         1

#define LED_STATUS_ON       0   // Active Low
#define LED_STATUS_OFF      1

#define PWM_STATE_SHIFT     0
#define PWM_STATE_ON        1
#define PWM_STATE_OFF       3

#define LED_LOGIC_ON        0   // Active Low
#define LED_LOGIC_OFF       1


typedef struct {
    uint8_t     mode;           // LED_MODE_NORMAL, LED_MODE_PWM    
    uint8_t     pwm_state;      // LED_STATE_ON, LED_STATE_OFF
    uint16_t    shift_ticks;    // shift time
    uint16_t    on_ticks;       // on time
    uint16_t    period_ticks;   // period (time measured in full cycle)
    uint16_t    ticks_count; 
    
    uint16_t    ticks_flash;    // For LED_Flash only 
}led_t;

void LED_Init(void);

void LED_Set(uint8_t k);
#define LED_Set     LED_Set
#define LED_On      LED_Set
#define led_set     LED_Set
#define led_on      LED_Set



void LED_Clr(uint8_t k);
#define LED_Clr     LED_Clr
#define LED_Off     LED_Clr
#define led_off     LED_Clr
#define led_clr     LED_Clr

void LED_Inv(uint8_t k);
#define LED_Inv     LED_Inv
#define LED_Toggle  LED_Inv
#define led_toggle  LED_Inv
#define led_inv     LED_Inv

uint8_t LED_Get(uint8_t k);
#define LED_Get     LED_Get
#define led_get     LED_Get

void LED_Write(uint8_t data);
#define LED_Write   LED_Write
#define led_write   LED_Write

uint8_t LED_Read(void);
#define LED_Read    LED_Read
#define led_read    LED_Read

void LED_ModeSet(uint8_t id, uint8_t  mode);
#define LED_ModeSet     LED_ModeSet
#define led_mode_set    LED_ModeSet

void LED_PwmSet(uint8_t id, uint16_t period_ticks, uint16_t shift_ticks, uint16_t on_ticks);
#define LED_PwmSet     LED_PwmSet
#define led_pwm_set    LED_PwmSet

void LED_PwmSetPeriod(uint8_t id, uint16_t period_ticks);
#define LED_PwmSetPeriod    LED_PwmSetPeriod
#define led_pwm_set_period  LED_PwmSetPeriod

void LED_PwmSetOnTime(uint8_t id, uint16_t on_ticks);
void LED_PwmSetShift(uint8_t id, uint16_t shift_ticks);


void LED_Flash(uint8_t id, uint16_t on_ticks);

void LED_Service(void);




#endif	


