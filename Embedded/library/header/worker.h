
 #ifndef __WORKER_H__
 #define __WORKER_H__

    #include "config.h"
    #include "event.h"

#define NUM_WORKERS             5   // Number of allowed workers


#define WORKER_STATUS_DELETED   0   // not created yet
#define WORKER_STATUS_WAITING   1   // wait for signal  
#define WORKER_STATUS_SLEEPING  2   // sleeping, waiting for timeout
#define WORKER_STATUS_WORKING   3   // working, ready to run


#define WORKER_ID_UNDEFINED    -1
#define WORKER_ID_0             0
#define WORKER_ID_1             1
#define WORKER_ID_2             2
#define WORKER_ID_3             3
#define WORKER_ID_4             4

typedef void (*worker_callback_t)(void *);

#define WORKER_EVENT_UNDEFINED      0   // undefined
#define WORKER_EVENT_READY          1   // ready to run
#define WORKER_EVENT_SLEEP_TIMEOUT  2   // wakeup, sleep timeout
#define WORKER_EVENT_WAKEUP         3   // wakeup
#define WORKER_EVENT_GOT_SIGNAL     4   // signal received
#define WORKER_EVENT_WAIT_TIMEOUT   5   // wait signel timeout

typedef struct {
    int    type;
    data_t data;
}worker_event_t;

typedef struct {
    uint8_t             id;         // worker id
    char *              name;       // worker name (pointer to char)
    uint8_t             status;     // worker status
    uint16_t             state;      // can be used for state machine
    uint16_t            ticks;      // tick counter, used for sleep and wait
    worker_callback_t   callback;   // callback function
    worker_event_t      event;      // worker event structure (type and data)
}worker_t;

int OS_WorkerInit(void);

worker_t* OS_WorkerCreate(char* name, worker_callback_t callback);

int OS_WorkerDelete(worker_t * worker);

int OS_WorkerSetCallback(worker_t* worker, worker_callback_t callback);

int OS_WorkerSleep(worker_t * worker, uint16_t  ticks);

int OS_WorkerWakeup(worker_t * worker);

int OS_WorkerWaitData(worker_t * worker, uint16_t  ticks);

int OS_WorkerSendData(worker_t * worker, data_t *data);

int OS_WorkerService(void);

 #endif
