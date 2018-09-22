
#ifndef __EVENT_H__
#define	__EVENT_H__
    #include "mcu.h"
    #include "data.h"


#define EVT_STIMER  0
#define EVT_WORKER  1
#define EVT_SWITCH  2
/*
 * callback
 */
typedef void(*callback_t)(void *);


/*
 * event structure
 */
typedef struct {
    uint8_t     type;       // event type, STIMER, WORKER
    void *      sender;     // event sender
    data_t      data;       // event data to be passed to callback function
    callback_t  callback;   // callback function pointer
}event_t;


/*
 * event queue length
 */
#define EVENT_QUEUE_LENGTH  32


/*
 * event queue structure
 */
typedef struct {
    event_t  evt[EVENT_QUEUE_LENGTH];   // event queue buffer
    uint16_t put;   // put index
    uint16_t get;   // get index
    uint16_t cnt;   // number of events storaged in evt queue 
}event_queue_t;


/*
 * event-queue operation status, the return codes
 */
#define EVENT_QUEUE_FULL    0   // evennt-queue full, cannot  put
#define EVENT_QUEUE_EMPTY   0   // evennt-queue empty, cannot get
#define EVENT_QUEUE_OK      1   // evennt-queue ok, put of get successfully


// Puts an object, timer or worker, into the event-queue
uint16_t OS_EventPut(event_t *event);

// Returns an object, timer or counter from the event-queue
uint16_t OS_EventGet(event_t *event);

void OS_EventLoop(void);
#endif	

