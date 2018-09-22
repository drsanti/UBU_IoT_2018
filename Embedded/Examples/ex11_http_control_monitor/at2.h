
#ifndef __AT_COMMAND_H__
#define __AT_COMMAND_H__


#include "os.h"


//!!************************************************************
//!! AT Command Return Types
//!!************************************************************
#define AT_RETURN_READY     0       //!! Module ready
#define AT_RETURN_OK        1       //!! Command is executed
#define AT_RETURN_SEND_OK   2       //!! Data is sent
#define AT_RETURN_BUSY      3       //!! Module is now executing 
#define AT_RETURN_FAIL      4       //!! Command execution failed
#define AT_RETURN_ERROR     5       //!! Command execution error
#define AT_RETURN_UNKNOWN   6       //!! Others





//!!************************************************************
//!! AT Command Queue
//!!************************************************************

#define COMMAND_QUEUE_LENGTH    16
typedef struct {
    char    *pptr[COMMAND_QUEUE_LENGTH];
    uint8_t put;
    uint8_t get;
    uint8_t cnt;
}command_queue_t;


//!!************************************************************
//!! AT Command States 
//!!************************************************************
#define AT_STATE_READY              0
#define AT_STATE_WAIT_RESPONSE      1
#define AT_STATE_GOT_RESPONSE       2
#define AT_STATE_WAIT_TIMEOUT       3

typedef struct {
    uint8_t    state;
    char       *pComd;
    uint16_t    curSec;
    uint16_t    prvSec;
    uint8_t    timeoutTicks;
    uint8_t    targetCode;
    uint8_t    returnCode;
}at_command_t;


void AT_Init(void);
void AT_Restart(void);

bool AT_Put(const char *pComd);
bool AT_Get(char **ppComd);

//!! Free memory that used to store the last command
void AT_FreeMemory(void);   

//!! Clean all blocks of memories
void AT_CleanMemory(void); 

void AT_ProcessLine(const char *line); 

void AT_Service(void);

#endif //!! __AT_COMMAND_H__
