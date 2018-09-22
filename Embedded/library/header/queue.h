/************************************************************
 * File:    queue.h                                         *
 * Author:  Dr.Santi Nuratch                                *
 *          Embedded Computing and Control Laboratory       *
 * Update:  04 July 2017, 08:01 PM                          *
 ************************************************************/

#ifndef __QUEUE_H__
    #define	__QUEUE_H__
    #include "config.h"


    
    #define QUEUE_ERROR_NONE    0
    #define QUEUE_ERROR_EMPTY   1
    #define QUEUE_ERROR_FULL    2

    typedef struct {
        unsigned char *buf;
        unsigned int put;
        unsigned int get;
        unsigned int cnt;
        unsigned int len;
        unsigned int err;
    }Queue;
    
    void Queue_Init(Queue *queue, unsigned char *buffer, unsigned int length);
    int  Queue_Put(Queue *queue, unsigned char data);
    int  Queue_Get(Queue *queue, unsigned char *data);

#endif 