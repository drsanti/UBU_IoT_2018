

#ifndef __EXEC_H__
#define	__EXEC_H__

#include "config.h"

typedef void * exec_callback_t;
typedef void * exec_sender_t;

typedef struct {
    exec_sender_t   sender;
    exec_callback_t callback;
    
}executor_t;


#define EXEC_ERROR_NONE         0
#define EXEC_ERROR_QUEUE_FULL   1
#define EXEC_ERROR_QUEUE_EMPTY  2

#define NUM_EXECUTORS           64

void Executor_Init(void);
int Executor_Put(executor_t exec);
int Executor_Get(executor_t *exec);

#endif


// USAGES:

/**********************************/
/*  PUT OBJECT TO THE EXEC-QUEUE  */
/**********************************/
//executor_t exec;
//exec.callback   = obj->callback;
//exec.parameter  = obj;
//Executor_Put(exec);

/**********************************/
/* GET OBJECT FROM THE EXEC-QUEUE */
/**********************************/
//executor_t ex;
//while(Executor_Get(&ex) == EXEC_ERROR_NONE) {
//    ((worker_callback_t)ex.callback)(ex.parameter);
//}


