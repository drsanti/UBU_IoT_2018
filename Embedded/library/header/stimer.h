
#ifndef __STIMER_H__
    #define	__STIMER_H__
    #include "mcu.h"


#define NUM_TIMERS              5

#define TIMER_MODE_UNDEFINED   -1
#define TIMER_MODE_ONESHORT     0
#define TIMER_MODE_CONTINUEOUS  1

#define TIMER_STATUS_DELETED    0
#define TIMER_STATUS_STOPPED    1
#define TIMER_STATUS_RUNNING    2

#define TIMER_ID_UNDEFINED     -1
#define TIMER_ID_0              0
#define TIMER_ID_1              1
#define TIMER_ID_2              2
#define TIMER_ID_3              3
#define TIMER_ID_4              4


typedef void (*timer_callback_t)(void *);

typedef uint16_t timer_mode_t;

typedef uint16_t timer_status_t;

typedef struct {
    char            *name;      // timer name
    uint16_t         id;        // timer id
    uint16_t         ticks;     // timer ticks, used for define tick-period
    uint16_t         count;     // timer counter used for internal counter
    timer_mode_t     mode;      // timer mode, TIMER_MODE_ONESHORT or TIMER_MODE_CONTINUEOUS
    timer_status_t   status;    // timer status
    timer_callback_t callback;  // timer callback
}timer_t;


int OS_TimerInit(void);

timer_t *OS_TimerCreate(char *name, uint16_t ticks, timer_mode_t mode, timer_callback_t callback);

int OS_TimerDelete(timer_t *timer);

int OS_TimerSetCallback(timer_t *timer, timer_callback_t callback);

int OS_TimerSetTicks(timer_t *timer, uint16_t ticks);

int OS_TimerStop(timer_t *timer);

int OS_TimerStart(timer_t *timer);

int OS_TimerService(void);

#endif 