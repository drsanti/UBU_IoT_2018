/************************************************************
 * File:    mcu.h                                           *
 * Author:  Dr.Santi Nuratch                                *
 *          Embedded Computing and Control Laboratory       *
 * Update:  05 July 2017, 11.56 AM                          *
 ************************************************************/

#ifndef __MCU_H__
    #define	__MCU_H__

    #include "config.h"

    void MCU_Init(void);
    
    //! Lock/Unlock IO mapping
    void MCU_UnlockRegisters(void);
    void MCU_LockRegisters(void);
    
    
    /*
     * alternative function name
     */
    #define MCU_UnlockRegisters MCU_UnlockRegisters
    #define MCU_LockRegisters   MCU_LockRegisters
    #define MCU_Init            MCU_Init
    #define mcu_init            MCU_Init
    #define mcu_unlock_register MCU_UnlockRegisters
    #define mcu_lock_registers  MCU_LockRegisters
    
    
    
    
    //!! ISRs control    
    #define ENTER_CRITICAL(action) {        \
        int old_ipl;                        \
        SET_AND_SAVE_CPU_IPL(old_ipl, 7);   \
        action;                             \
        RESTORE_CPU_IPL(old_ipl);           \
    }
    #define enter_critical ENTER_CRITICAL

    #define TIMER1_ISR_DISABLE()    IEC0bits.T1IE=0
    #define TIMER1_ISR_ENABLE()     IEC0bits.T1IE=1
    
    #define timer1_isr_enable TIMER1_ISR_ENABLE
    #define timer1_isr_disable TIMER1_ISR_DISABLE

#endif

