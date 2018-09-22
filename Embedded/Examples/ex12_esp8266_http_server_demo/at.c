
#include "at.h"
#include "stdlib.h"

#define AT_RETURN_READY     0       //!! Module ready
#define AT_RETURN_OK        1       //!! Command is executed
#define AT_RETURN_SEND_OK   2       //!! Data is sent
#define AT_RETURN_BUSY      3       //!! Module is now executing 
#define AT_RETURN_FAIL      4       //!! Command execution failed
#define AT_RETURN_ERROR     5       //!! Command execution error
#define AT_RETURN_UNKNOWN  0xFF     //!! Others

//!! Must be matched to the lines above!
//!! This constant strings are used for token ckecking, see in AT_Service() below
const char *AtKeywords[] = {
    "ready\r\n",
    "OK\r\n",
    "SEND OK\r\n",
    "busy p...\r\n",
    "FAIL\r\n",
    "ERROR\r\n",
    ""  //!! Must be ended with EMPTY string
};

//!! Command and AT objects
static command_queue_t  cmd;
static at_command_t     at;



//!!
//!!#define _AT_PUT_DEBUG_
//!!

//!! Take a block of memory from heap, and copy the command into the located memory
bool AT_Put(const char *pComd) 
{
    if(cmd.cnt >= COMMAND_QUEUE_LENGTH) 
    {
        //!! No more space for the new command
        return false;
    }

    //!! Allocate memory and copy the command
    uint8_t len = strlen(pComd);
    char *ptr = (char *)malloc(len + 1);
    strcpy(ptr, pComd);
    
    //!! Update the command object's variables
    cmd.pptr[cmd.put] = ptr;
    cmd.put = (cmd.put + 1) % COMMAND_QUEUE_LENGTH;
    cmd.cnt++;

    #ifdef _AT_PUT_DEBUG_
        char buff[32];
        sprintf(buff, "\t>>addr:0x%.4X 0x%.4X %d\r\n", (uint16_t)cmd.pptr[cmd.put-1], (uint16_t)ptr, strlen(pComd)+1);
        Uart1_AsyncWriteString(buff);
    #endif //!! _AT_PUT_DEBUG_
    return true;
}

bool AT_Get(char **ppComd) 
{
    if(cmd.cnt <= 0) 
    {
        //!! No more command in the queue
        return false;
    }
    //!! Point to a next command in the queue
    *ppComd = cmd.pptr[cmd.get];

    //!! Update command object's variables
    cmd.get = (cmd.get + 1) % COMMAND_QUEUE_LENGTH;
    cmd.cnt--;
    return true;
}


//!! Initialise all variable of the Command and AT objects
void AT_Init(void) 
{
    cmd.put = 0;
    cmd.get = 0;
    cmd.cnt = 0;

    at.state        = AT_STATE_READY;
    at.returnCode   = AT_RETURN_READY;
    at.targetCode   = AT_RETURN_OK;
    at.timeoutTicks = 0;
}


//!!
//!!#define _AT_FREE_DEBUG_
//!!
void AT_FreeMemory(void) 
{
    #ifdef _AT_FREE_DEBUG_
        char buff[32];
        sprintf(buff, "\t>>free:0x%.4X\r\n", (uint16_t)at.pComd);
        Uart1_AsyncWriteString(buff);
    #endif //!! _AT_FREE_DEBUG_

    //!! Free the memory pointed by the at.pComd
    free(at.pComd);
    at.pComd = NULL;
}




//!!
//!!#define _AT_STATE_INFO_DEBUG_
//!!#define _AT_STATE_BUSY_DEBUG_
//!!#define _AT_STATE_PUT_DEBUG_
//!!

//!! This function can be called by LineReceived and Timer
void AT_Service(void *evt) 
{
    //!! Thake the received line
    const char *line = (const char*)evt;

    //!! If the function is called by LineReceived callback,
    //!! the line variable must be not NULL
    if(line != NULL) 
    {   
        //!! Called by line received callback
        uint8_t i = 0;
        while(strcmp(AtKeywords[i], "")) 
        {
            //!! Ckeck/Match the keywords
            if(!strcmp(line, AtKeywords[i])) 
            {
                //!! Update the AT's state control variables
                at.state      = AT_STATE_GOT_RESPONSE;
                at.returnCode = i;

                #ifdef _AT_EXECUTE_LINE_DEBUG_
                    Uart1_AsyncWriteString("\r\n>>Keyword: ");
                    Uart1_AsyncWriteString(AtKeywords[i]);
                #endif //!! _AT_EXECUTE_LINE_DEBUG_
            }
            i++;
        }

        //!! Must be RETURN!!.
        //!! In this contition, this block of code, is called by LineReceived callback
        //!! to check the rokens/keywords. If the key work is matched, 
        //!! the at.state is changed to AT_STATE_GOT_RESPONSE. If not retur, the code below will destroy
        //!! the last command by executing AT_FreeMemory(), causing deadlock!!
        return;
    }
    //!!--------------------------------------------------------------------------------------


    //!!
    //!! Code below can be run if the line == NULL, 
    //!! it means that this function is called by Timer, not LineReceived
    //!!

    //!! Buffer for sprintf()
    char buff[48];

    //!! Get current time
    at.curSec = OS_TimeGet().ss; 

    //!!
    //!! Time out checking 
    //!!
    if( at.prvSec != at.curSec ) 
    {
        //!! Update the second
        at.prvSec = at.curSec; 

        //!! Running in TIMEOUT state, print report
        if( at.state == AT_STATE_WAIT_TIMEOUT ) 
        {
            //!! Print the timeout ant atate information
            sprintf(buff, "\r\nTimeout: at.state=%d retCode=%d\r\n", at.state, at.returnCode);
            Uart1_AsyncWriteString(buff);

            //
            //!! Timeout!!! WHAT TO DO?
            //
            //at.state = AT_STATE_READY;
            at.state = AT_STATE_WAIT_RESPONSE;  //!! It will wait for "ready"
            AT_Put("AT+RST\r\n");
        }
        
        //!! Waiting for response, Timeout checking
        else if(at.state == AT_STATE_WAIT_RESPONSE) 
        {
            if( ++at.timeoutTicks >= 5 ) 
            {
                //!! Timeout!
                at.timeoutTicks = 0;
                at.state        = AT_STATE_WAIT_TIMEOUT;

                //!! Free memory
                AT_FreeMemory();
            }
        } 
        else 
        {
            #ifdef _AT_STATE_INFO_DEBUG_
                sprintf(buff, "Info: at.state=%d at.retCode=%d %s\r\n", at.state, at.returnCode, at.pComd);
                Uart1_AsyncWriteString(buff);
            #endif //!! _AT_STATE_INFO_DEBUG_
        }
    }

    //!!
    //!! Is the AT_STATE_GOT_RESPONSE received?
    //!!
    if(at.state == AT_STATE_GOT_RESPONSE) 
    {
        //!! Got a response message
        if(at.returnCode == AT_RETURN_BUSY) 
        {
            //!! Response with busy, return and wait for next turn
            #ifdef _AT_STATE_BUSY_DEBUG_
                Uart1_AsyncWriteString("\tat.state AT_RETURN_BUSY\r\n");
            #endif //!! _AT_STATE_BUSY_DEBUG_
            
            //!! return
            return;
        }

        //!! The last command is executed, free memory
        AT_FreeMemory();

        //!!
        //!! ERROR
        //!!
        if(at.returnCode == AT_RETURN_ERROR) 
        {
            //!! The last command execution error
            //!! Change reset the AT's parameters
            at.returnCode = AT_RETURN_UNKNOWN;
            //at.state      = AT_STATE_READY;
            at.state = AT_STATE_WAIT_RESPONSE;  //!! It will wait for "ready"

            //!! Print error information
            Uart1_AsyncWriteString("\r\n\r\nThe Command Execution Error! Restarting...\r\n\r\n");

            //!! Restart the module
            AT_Put("AT+RST\r\n");

            //!! Return
            return;
        }
        //!!
        //!! FAIL
        //!!
        if(at.returnCode == AT_RETURN_FAIL) 
        {
             //!! The last command execution failed
            //!! Change reset the AT's parameters
            at.returnCode = AT_RETURN_UNKNOWN;
            //at.state = AT_STATE_READY;
            at.state = AT_STATE_WAIT_RESPONSE;  //!! It will wait for "ready"

            //!! Print error information
            Uart1_AsyncWriteString("\r\n\r\nThe Command Execution Failed! Restarting...\r\n\r\n");

            //!! Restart the module
            AT_Put("AT+RST\r\n");

            //!! Return
            return;
        }

        //!!
        //!! "ready",  "OK", "SEND_OK", "busy p..."
        //!!
        if(at.returnCode != AT_RETURN_UNKNOWN) 
        {   
            //!! A positive keyword is returned,
            //!! e.g.; "ready", "OK", "SEND_OK" or "busy p..."
            //!! reset the AT's state control variables
            at.returnCode = AT_RETURN_UNKNOWN;
            at.state      = AT_STATE_READY;     
        }
        else
        {
            //!! Undefined retuened code, return
            return;
        }
    }
    //!! It's in time out state, return and wait for next turn
    //!! Note: See above for the timeout checking
    else if(at.state == AT_STATE_WAIT_TIMEOUT) 
    {
        return;
    }

    //!!
    //!! Ready for new command/data
    //!!
    char *ptr = 0;
    if(AT_Get(&ptr)) 
    {
        //!! Save the memory address, it will be used in free() operation later
        at.pComd = ptr;

        //!! Change the at.atate to WAIT_RESPONSE
        at.state = AT_STATE_WAIT_RESPONSE;

        //!! Send command/data to the module
        Uart2_AsyncWriteString(ptr);

        #ifdef _AT_STATE_PUT_DEBUG_
            Uart1_AsyncWriteString("\r\n>>AT: ");
            Uart1_AsyncWriteString(ptr);
        #endif
    }

}//!! AT_StateMachines
