#include "server2.h"
#include "wifi2.h"
#include "at2.h"
#include "page.h"




//const char * htmlHeader = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";
const char * htmlHeader = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";

//const char * faviconIcoHead = "HTTP/1.1 200 OK\r\nContent-Type: image/png\r\nConnection: close\r\n\r\n"; 
//const char * cssHeader      = "HTTP/1.1 200 OK\r\nContent-Type: text/css\r\nConnection:  close\r\n\r\n";
//const char * jsHeader       = "HTTP/1.1 200 OK\r\nContent-Type: text/css\r\n\r\n"; //!! No close is required!



extern at_command_t at;
extern wifi_t       wifi;

server_t server;
client_t clients[NUM_CLIENTS];


//!! Initialise all parameters of clients
void Server_ClientInit(void) 
{
    uint8_t i;
    client_t *client = clients;
    for(i=0; i<NUM_CLIENTS; i++)
    {
        client->channel      = 0xFF; //!! Connected channel [0, 4]
        client->getBuffer[0] = 0;    //!! Get buffer, 32 bytes maximum
        client->milliSeconds = 0;    //!! Milliseconds, used for multiclientle frames sending
        client->restFrames   = 0;    //!! Number of frames needed to be sent
        client->restBytes    = 0;    //!! Number of bytes needed to be sent
        client++;                    //!! Point to next client object
    }
} //!! Server_ClientInit



void Server_Service(void *evt)
{

    char buff[48];

    const char * line = (const char *)evt;


    //!! Search for new client
    if( server.client == NULL)
    {
        uint8_t   i;
        for( i=0; i<NUM_CLIENTS; i++ )
        {
            if( clients[i].state == CLIENT_STATE_REQUESTED )
            {
                //!! A new target client is found
                server.client = &clients[i];    
                break;
            }
        } 
    } 

    //!! No requested client, return
    if( server.client == NULL ) 
    {
        return;
    }

    //!! Take current client
    client_t *client = server.client;
    if( client->state == CLIENT_STATE_REQUESTED ) 
    {
        //!! Default
        client->data = NULL;

        //!! Content-Type
        bool IsHttpHeaderRequired = false;

        //!! favicon.ico
        if( !strcmp( client->getBuffer, "favicon.ico" ) )
        {
            client->data = faviconIco;  
        }
        //!! index.html
        else if( !strcmp( client->getBuffer, "" ) || !strcmp( client->getBuffer, "index.html" ) ) 
        {
            client->data = indexHtml;  
        }
        //!! index.css
        else if( !strcmp( client->getBuffer, "index.css" ) ) 
        {
            client->data = indexCss;  
        }
        //!! index.js
        else if( !strcmp( client->getBuffer, "index.js" ) ) 
        {
            client->data = indexJs;  
        }
        //!! callback
        else
        {
            if( server.callback != NULL )
            {
                server.callback( &server );

                //!! The client->data points to the memory containing html content
                if(client->data != NULL) {
                    IsHttpHeaderRequired = true;
                    client->httpHead = htmlHeader;
                }
            }    
        }

        //----------------------------------------
        //!! If there are not static pages (index.html, index.css, index.js and favicon.ico),
        //!! it means that the content is created in run-time. In this situation, the HTML-HEAD
        //!! should be prefixed (this is very impotant for iOS)
        if( IsHttpHeaderRequired == true )
        {
            //!! Construct HEADER + CONTENT
            
            int n1 = strlen( client->httpHead );
            int n2 = strlen( client->data );
            int nn = n1 + n2 + 5;
            char *page = malloc( nn );
            memset(page, 0, nn);
            memcpy(page, client->httpHead, n1);
            memcpy(page + n1, client->data, n2);
            if((uint16_t)client->data < 0x2000)
            {   //!! The callback may allocate a block of memory
                //!! Be sure it is freed here
                free( (void *) client->data );
            }

            //!! Point to the full page (header + html content)
            client->data = page;

            //!!-----------------------------------------------------------
            Uart1_AsyncWriteString("Server: Constructed HTML head + content ");
            sprintf(buff, "(%d + %d ) %d bytes\r\n", n1, n2, nn);
            Uart1_AsyncWriteString(buff);
        }

        //!! check the client->data
        if(client->data == NULL)
        {
            client->data = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n<h1 class='text-danger'>Page Not Found!!</h1>";
        }
        else
        {
            size_t pageLength = strlen(client->data);
            if(pageLength > 2048)
            {
                //!! The client->data is too long (>2048 bytes), create information page
                Uart1_WriteString("\r\n*******************************************\r\n");
                Uart1_WriteString("* The payload is greater than 2048 bytes! *\r\n");
                Uart1_WriteString("*******************************************\r\n\r\n");
                
                client->data = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n<h1 class='text-danger'>The payload is greater than 2048 bytes!</h1>";
            }
        }

        //!!
        //!! Change client's state and construct the CIPSEND
        //!!
        client->state  = CLIENT_STATE_WAIT_OK;

        //!! Construct the CIPSEND and pu it into queue
        //!! Dynamic memory or local buffer can be used to store the CIPSEND command
        size_t pageBytes = strlen(client->data);
        char *pCIPSEND = (char *)malloc(32 * sizeof(char));
        sprintf( pCIPSEND, "AT+CIPSEND=%d,%d\r\n", client->channel, pageBytes );
        AT_Put( pCIPSEND );     //!! Put it into command queue
        free((void *)pCIPSEND); //!! Don't forget to free it!

    }


    //!! Waiting for OK 
    if( client->state == CLIENT_STATE_WAIT_OK ) 
    {
        //!! The CIPSEND is sent, now waiting for OK
        if( !strcmp(line, "OK\r\n") )
        {

            //!! The OK is detected, send the DATA
            client->state = CLIENT_STATE_WAIT_SEND_OK; 

            //!!--------------------------------------------------
            if( strlen( client->data ) <= 128 ) 
            {   
                //!! Payload data is quite small, 
                //!! we can put it into the dynamic/heap memory by using the AT_Put()
                //!! The AT_Put() will allocate new block of memory and copy the client->data
                //!! into the allocated memory. The memory will be freed after all bytes are sent
                //!! Noted: Uart2 buffer and Uart2 rung buffer are 1024 bytes, see in config.h

                AT_Put( client->data ); //!! The AT_Put() copies the content to its own memory. 
                
                //Uart1_AsyncWriteString("Server: The page is less than 128 bytes\r\n");
                if((uint16_t)client->data < 0x2000)
                {   //!! The client->data is needed to be freed
                    //!!--- DEBUG --------------------------------------------
                    //char *pBuff = malloc(32 + strlen(client->data));
                    //sprintf(pBuff, "Server: Free client->data 0x%.4X, %d bytes\r\n", (uint16_t)client->data, strlen(client->data));
                    //Uart1_WriteString(pBuff);
                    //free( (void *) pBuff);
                    //!!--- END Debug ----------------------------------------

                    free( (void *) client->data);
                }

                //!! Besure they are cleared
                client->restFrames   = 0;
                client->restBytes    = 0;
            }
            else 
            {   
                //!! In case of the payload, the client->data, is greater than 500 bytes,
                //!! the payload is devided into some shorter frames, 500 bytes per frame in this case
                //!! To make sure the space of heap memory (1024 bytes) and transmission buffers (Tx, and Ring)
                //!! can be used to storage the frames of data, each frame must be sent separatly, see the block
                //!! of code below how it divided, and check in next state how they are sent
                //!!--------------------------------------------------
                uint16_t length      = strlen(client->data);
                client->restFrames   = length / 500;
                client->restBytes    = length % 500;
                client->milliSeconds = OS_TimeMilliseconds();
            }
        }
    }
    else if( client->state == CLIENT_STATE_WAIT_SEND_OK ) 
    {
        //!! The DATA (frames) are sending, waiting for SEND_OK
        if( !strcmp(line, "SEND OK\r\n") ) 
        {

            //!! Clear
            client->restFrames   = 0;
            client->restBytes    = 0;

            if((uint16_t)client->data < 0x2000) //!! Heap memory loacated around 0x1D00
            {   
                //!! Free only heap
                //!! Note: the client->data can point to constant string and heap memory
                //!!       Free only memory located in the heap area (around 0x1F42)
                //!!--- DEBUG -----------------------------------------
                sprintf(buff, "Server: Free Memory 0x%.4X\r\n", (uint16_t)client->data);
                Uart1_AsyncWriteString(buff);
                //!!--- END--------------------------------------------

                //!! Free the memory
                free((void *)client->data);
            }


            //!! The SEND_OK is received, send CIPCLOSE
            client->state = CLIENT_STATE_WAIT_CLOSE_OK;
            sprintf( buff, "AT+CIPCLOSE=%d\r\n", client->channel );
            AT_Put( buff );
        }
        else
        {   //!! Is it more frame to be sent?
            if(client->restFrames) 
            {
                //!! Be sure the previous frame is sent by waiting for 50 mS
                uint32_t currentMs = OS_TimeMilliseconds();
                if(currentMs - client->milliSeconds > 50) 
                {
                    //!! Update the time
                    client->milliSeconds = currentMs;

                    //!! Send the target frame. Note that, do not use the AT_Put()
                    //!! because the OS 's heap memory and the transmisstion buffers
                    //!! (U2TX and U2-Ring buffers) have 1024 bytes.
                    Uart2_AsyncWriteBytes(  (uint8_t *)client->data, 500 ); 
                    
                    client->data += 500;    //!! Point to next frame
                    client->restFrames--;   //!! Decrese the frame counter
                }   
            }
            else 
            {
                //!! No more full frames (500-byte frames), but some bytes are need to be sent 
                //!! After the rest bytes are sent, the module will return OK, and the block
                //!! of code will be executed, state is changed and the CIPCLOSE command will be applied
                if(client->restBytes > 0)
                {
                    Uart2_AsyncWriteBytes( (uint8_t *)client->data, client->restBytes );
                }      
            }
        }
    }
    //!! The CIPCLOSE command is applied, waiting for OK
    else if( client->state == CLIENT_STATE_WAIT_CLOSE_OK ) 
    {
        if( !strcmp(line, "OK\r\n") )
        {
            //!! The OK is received, kill the client

            //!!
            //!! Well done!
            //!!
            os_time_t time = OS_TimeGet();
            sprintf(buff, "Server: %.2d:%.2d:%.2d:%.3d: ", time.hh, time.mm, time.ss, time.ms);
            Uart1_AsyncWriteString(buff);

            sprintf(buff, "#%d Closed\r\n", client->channel);
            Uart1_AsyncWriteString(buff);

            //!! Reset
            client->state = CLIENT_STATE_DISCONNECTED; 
            client->channel = 0xFF;
            server.client = NULL;
        }
    }
    else
    {
        //!!
        //!! Something went wrong!
        //!!
        //Beep(1000);
    }
}



//!! Called by LineReceived
void Server_CreateClient(const char *line) 
{
    //!! Buffer for sprintf();
    char buff[64];

    //!! Client GET requested
    //!!
    //!! Format: +IPD,0,419:GET / HTTP/1.1
    //!!
    uint8_t channel = line[5] - 0x30;
    uint8_t idx1 = str_index_of_first_token( line, "GET /" );
    uint8_t idx2 = str_index_of_first_token( line, "HTTP/" );
    if( idx1 < 0 || idx2 < 0 || channel < 0 || channel > 4 ) 
    {
        //!! Wrong Format!!
        sprintf( buff, "\r\nServer: Wrong Format! idx1:%d idx2:%d channel:%d\r\n", idx1, idx2, channel );
        Uart1_AsyncWriteString( buff );
        return;
    }

    //-------------------------------------------------------------------
    //!! Take the GET
    char getBuff[CLIENT_GET_BUFFER_LENGTH + 1];
    idx1 += 5; idx2 -= 1;
    uint8_t nb = idx2 - idx1;
    if( nb >= CLIENT_GET_BUFFER_LENGTH ) 
    {
        nb = CLIENT_GET_BUFFER_LENGTH;
    }
    memset(getBuff, 0, CLIENT_GET_BUFFER_LENGTH);   //!! clear
    memcpy( getBuff, (line + idx1), nb );           //!! copy
    //!!------------------------------------------------------------------

    client_t *c = &clients[channel];
    if(c->state == CLIENT_STATE_REQUESTED) 
    {
        //!! Too much request!!
        Uart1_WriteString("\r\n\r\n***[*_*]***\r\n\r\n");
        //OS_RestartMcu();
        WiFi_Restart();
        return;
    }

    //!! The client slot is available
    c->state   = CLIENT_STATE_REQUESTED;
    c->channel = channel;
    memset(c->getBuffer, 0, CLIENT_GET_BUFFER_LENGTH);
    strcpy(c->getBuffer, getBuff);

    //!!------------------------------------------------------------------

    os_time_t time = OS_TimeGet();
    sprintf(buff, "\r\nServer: %.2d:%.2d:%.2d:%.3d: ", time.hh, time.mm, time.ss, time.ms);
    Uart1_AsyncWriteString(buff);

    sprintf(buff, "#%d Requesting for \"/%s\"\r\n", channel, getBuff);
    Uart1_AsyncWriteString(buff);

} //!! Server_CreateClient()



void Server_ProcessLine(const char *line)
{
    if( 0 == str_index_of_first_token(line, "+IPD,") )
    {
        Server_CreateClient(line);
    }
    else if( 1 == str_index_of_first_token(line, ",CONNECT\r\n") ) {

    }
    else if( 1 == str_index_of_first_token(line, ",CLOSED\r\n") ) {
        
    }
}


//!!************************************************************************************
//!!*      SERVER   SERVER   SERVER   SERVER   SERVER   SERVER   SERVER   SERVER       *                                    
//!!************************************************************************************

void Server_Init(const char *ssid, const char *pass, uint8_t mode, os_callback_t callback)
{
    server.state = SERVER_STATE_STOPPED;
    server.wifi  = &wifi;
    server.callback = callback;

    Server_ClientInit();

}
