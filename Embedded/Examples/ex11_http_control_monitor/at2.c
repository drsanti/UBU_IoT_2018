
#include "at2.h"
#include "wifi2.h"


//!!
//#define _AT_DEBUG_
//!!


const char *AtReturnMsg[] = {
    "AT_RETURN_READY",
    "AT_RETURN_OK",
    "AT_RETURN_SEND_OK",
    "AT_RETURN_BUSY",
    "AT_RETURN_FAIL",
    "AT_RETURN_ERROR",
    "AT_RETURN_UNKNOWN"
};

//!!
#define at_print(smg) Uart1_AsyncWriteString(smg);

#define OS_RestartMcu() __asm__ volatile ("reset")

//!! Command and AT objects (Global variables)
command_queue_t  cmd;
at_command_t     at;

extern wifi_t wifi;


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

    return true;
}

//!!
//#define _AT_GET_DEBUG_
//!!
bool AT_Get(char **ppComd) 
{
    if(cmd.cnt <= 0) 
    {
        //!! No more command in the queue
        return false;
    }
    //!! Point to a next command in the queue
    *ppComd = cmd.pptr[cmd.get];


     #ifdef _AT_GET_DEBUG_
        char buff[32];
        sprintf(buff, "\t>>addr:0x%.4X\r\n", (uint16_t)*ppComd);
        Uart1_AsyncWriteString(buff);
    #endif //!! _AT_GET_DEBUG_

    //!! Update command object's variables
    cmd.get = (cmd.get + 1) % COMMAND_QUEUE_LENGTH;
    cmd.cnt--;
    return true;
}


//!!
//#define _AT_FREE_DEBUG_
//!!
void AT_FreeMemory(void) 
{
    if(at.pComd == NULL) 
    {
        return;
    }

    #ifdef _AT_FREE_DEBUG_
        char buff[32];
        sprintf(buff, "\t>>free:0x%.4X\r\n", (uint16_t)at.pComd);
        Uart1_AsyncWriteString(buff);
    #endif //!! _AT_FREE_DEBUG_

    //!! Free the memory pointed by the at.pComd
    free(at.pComd);
    at.pComd = NULL;
}


//!! Free all blocks of memories
void AT_CleanMemory(void)
{
    char *pMem = 0;
    while( AT_Get( &pMem ) )
    {
        free( pMem );
        pMem = NULL;
    }
    cmd.put = 0;
    cmd.get = 0;
    cmd.cnt = 0;
}


//!! Restart all states of AT and clean all block of memories
void AT_Restart(void)
{
    at.state        = AT_STATE_READY;
    at.pComd       = NULL;
    at.returnCode   = AT_RETURN_READY;
    at.targetCode   = AT_RETURN_OK;

    at.timeoutTicks = 0;   
    at.curSec       = 0;
    at.prvSec       = 0;

    //!! Delete all block of memories
    AT_CleanMemory();
}

//!! Initialise all variable of the Command and AT objects
void AT_Init(void) 
{
    //!! Restart the AT
    AT_Restart();
}



//!! AT Command service, called by timer
void AT_Service(void) 
{

    static uint16_t prv_sec = 0;
    uint16_t cur_sec = OS_TimeGet().ss;

    if(prv_sec != cur_sec)
    {   
        prv_sec = cur_sec;
        
        if(at.state == AT_STATE_WAIT_RESPONSE)
        {
            #define AT_TIMEOUT_VAL 10

            //!! Increase timeout ticks
            at.timeoutTicks++;

            //--- DEBUG -----------------------------------------------
            char *pBuff = malloc(32 + strlen(at.pComd));
            at_print("\tAT: Waiting for response ");
            sprintf(pBuff, "[%.2d]\r\n", AT_TIMEOUT_VAL-at.timeoutTicks);
            at_print(pBuff);
            sprintf(pBuff, "\tAT: The last command is %s\r\n", at.pComd);
            at_print(pBuff);
           
            free((void *)pBuff);
            //--- END -------------------------------------------------

            if( at.timeoutTicks >= AT_TIMEOUT_VAL )
             {
                //!! Clear the at.timeoutTicks
                at.timeoutTicks = 0;

                //!! Fource the at.state to AT_STATE_READY
                at.state = AT_STATE_READY;

                //--- DEBUG -----------------------------------------------
                #ifdef _AT_DEBUG_
                    at_print("\r\n");
                    at_print("\tAT: Timeout!\r\n");
                    at_print("\tAT: The at.state is changed to AT_STATE_READY\r\n\r\n");
                #endif
                //--- END -------------------------------------------------

                //!! Restart the AT. Clean memory, reset all state variable
                Beep(50);
                at_print("\r\n\r\n");
                at_print("\t*************************************\r\n");
                at_print("\tAT: Timeout! Restarting the system...\r\n");
                at_print("\t*************************************\r\n");
                at_print("\r\n");

                //!! Clear AT state and all block of memories
                AT_Restart();

                //!! Restart the module (optional)
                AT_Put( "AT+RST\r\n" );
                __delay_ms(100);

                //!! Restart the MCU!
                OS_RestartMcu();
            }
            return;
        }
        else if(at.state == AT_STATE_READY)
        {
            //Uart1_AsyncWriteString("AT-SRV: Waiting for new command...\r\n");
        }
        else if(at.state == AT_STATE_WAIT_TIMEOUT)
        {
            //Uart1_AsyncWriteString("\tAT: Timeout!!\r\n");
        }
        else if(at.state == AT_STATE_GOT_RESPONSE)
        {
            //!! Besure the at.timeoutTicks is reset
            at.timeoutTicks = 0;
        }
        else
        {
            //Uart1_AsyncWriteString("\tAT: Unknown state!!\r\n");
        }
    }


    //!!-----------------------------------------------------------
    //!!               S T A T E   C H E C K I N G
    //!!-----------------------------------------------------------
    if( at.state == AT_STATE_GOT_RESPONSE )
    {
        at.timeoutTicks = 0;

        //--- DEBUG -----------------------------------------------
        #ifdef _AT_DEBUG_
            if(at.returnCode != AT_RETURN_BUSY)
            {   
                at_print("\r\n");
                at_print("\tAT: Got Response!\r\n");
                at_print( "\tAT: The return code is ");
                at_print(AtReturnMsg[at.returnCode]);
                at_print("\r\n\r\n");
            }
        #endif
        //--- END -------------------------------------------------

        if(at.returnCode == AT_RETURN_OK)
        {
            //!! OK: Command is executed
            at.state = AT_STATE_READY;  //!! Ready for new command
        }
        else if(at.returnCode == AT_RETURN_SEND_OK)
        {
            //!! SEND_OK: Data is sent
            at.state = AT_STATE_READY;  //!! Ready for new command
        }
        else if(at.returnCode == AT_RETURN_READY)
        {
            //!! READY: The module boot up
            at.state = AT_STATE_READY;  //!! Ready for new command
        }
        else if(at.returnCode == AT_RETURN_BUSY)
        {
            //!! BUSY: In progress...
        }
        else if(at.returnCode == AT_RETURN_ERROR)
        {
            //!! ERROR: Something wrong!
            
            //!! Fource the at.atate to AT_STATE_READY
            at.state = AT_STATE_READY;


            //!!--- DEBUG ----------------------------------
            #ifdef _AT_DEBUG_
                at_print("\r\n\r\n");
                at_print("\tAT: AT_RETURN_ERROR\r\n");
                at_print("\tAT: The at.state is changed to \"AT_STATE_READY\"\r\n\r\n");
            #endif
            //!!--- END ------------------------------------

            //!! Restart the AT. Clean memory, reset all state variable
            //AT_Restart();
        }
        else if(at.returnCode == AT_RETURN_FAIL)
        {
            //!! FAIL: Something failed!

            //!! Fource the at.atate to AT_STATE_READY
            at.state = AT_STATE_READY;

            //!!--- DEBUG ----------------------------------
            #ifdef _AT_DEBUG_
                at_print("\r\n\r\n");
                at_print("\tAT: AT_RETURN_FAIL\r\n");
                at_print("\tAT: The at.state is changed to \"AT_STATE_READY\"\r\n\r\n");
            #endif
            //!!--- END ------------------------------------

            //!!
            WiFi_ExecuteOnFail();
        }

        //!!--- DEBUG ----------------------------------
        #ifdef _AT_DEBUG_
            if(at.pComd != 0)
            {
                char buff[32];
                sprintf(buff, "\tAT: Free 0x%.4X\r\n", (uint16_t)at.pComd);
                at_print(buff);
            }
        #endif
        //!!--- END ------------------------------------
        

        
        //!!
        if(at.state == AT_STATE_READY)
        {
            //!!--- DEBUG ----------------------------------
            #ifdef _AT_DEBUG_
                at_print("\tAT: Ready for new command...\r\n");
            #endif
            //!!--- END ------------------------------------
        }

        


        //!!
        //!! Free the memory that store the last command
        //!!
        AT_FreeMemory();

    }


    //!!-------------------------------------------------------
    if(at.state == AT_STATE_READY)
    {
        char *ptr = 0;
        if( AT_Get( &ptr ) ) 
        {
            //!! Be sure the last used is freed
            AT_FreeMemory();

            //!!--- DEBUG ----------------------------------
            #ifdef _AT_DEBUG_
                char buff[32];
                sprintf(buff, "\tAT: Alloc 0x%.4X\r\n", (uint16_t)ptr);
                at_print(buff);
                if( 0 == str_index_of_first_token(ptr, "AT+") )  //!! Ignore data, print only connand
                {
                    at_print("\tAT: Send ");
                    at_print(ptr);
                    at_print("\r\n");
                }
            #endif
            //!!--- END ------------------------------------

            //
            at.pComd = ptr;                     //!! Load current command
            at.state = AT_STATE_WAIT_RESPONSE;  //!! Change state
            at.returnCode = AT_RETURN_UNKNOWN;  //!! Initial with Unknown
            Uart2_AsyncWriteString(ptr);        //!! Write to ESP
        }
    }
}

void AT_ProcessLine(const char *line)
{

    //!!-------------------------------------------------------
    if( 0 == str_index_of_first_token(line, "OK\r\n") )
    {
        at.state = AT_STATE_GOT_RESPONSE;
        at.returnCode = AT_RETURN_OK;
    }
    else if( 0 == str_index_of_first_token(line, "SEND OK\r\n") )
    {
        at.state = AT_STATE_GOT_RESPONSE;
        at.returnCode = AT_RETURN_SEND_OK;
    }
    else if( 0 == str_index_of_first_token(line, "ready\r\n") )
    {
        at.state = AT_STATE_GOT_RESPONSE;
        at.returnCode = AT_RETURN_READY;

        Uart1_WriteString("\r\n**************************\r\n");
        Uart1_WriteString("* The WiFi Module Ready! *\r\n");
        Uart1_WriteString("**************************\r\n\r\n");
    }
    else if( 0 == str_index_of_first_token(line, "busy p...\r\n") ||  0 == str_index_of_first_token(line, "busy s...\r\n") )
    {
        at.state = AT_STATE_GOT_RESPONSE;
        at.returnCode = AT_RETURN_BUSY;
    }
    else if( 0 == str_index_of_first_token(line, "ERROR\r\n") )
    {
        at.state = AT_STATE_GOT_RESPONSE;
        at.returnCode = AT_RETURN_ERROR;
    }
    else if( 0 == str_index_of_first_token(line, "FAIL\r\n") )
    {
        at.state = AT_STATE_GOT_RESPONSE;
        at.returnCode = AT_RETURN_FAIL;
    }
    else 
    {
        //Uart1_AsyncWriteString("\tReceived line: ");
        //Uart1_AsyncWriteString(line);
        return;
    }
}
