#ifndef __SERVER_H__
#define __SERVER_H__

#include "os.h"
#include "wifi2.h"

#define NUM_CLIENTS                 8
#define CLIENT_GET_BUFFER_LENGTH    32

#define CLIENT_STATE_DISCONNECTED    0
#define CLIENT_STATE_REQUESTED       1
#define CLIENT_STATE_WAIT_OK         2
#define CLIENT_STATE_WAIT_SEND_OK    3
#define CLIENT_STATE_WAIT_CLOSE_OK   4
typedef struct {
    uint8_t  state;                                 //!! State index
    uint8_t  channel;                               //!! Connected channel [0, 4]
    uint32_t milliSeconds;                          //!! Milliseconds, used for multiclientle frames sending   
    uint16_t restFrames;                            //!! Number of the rest frames needed to be sent
    uint16_t restBytes;                             //!! Number of the rest bytes needed to be sent
    char     getBuffer[CLIENT_GET_BUFFER_LENGTH];   //!! Get buffer, 32 bytes maximum
    const char *data;                               //!! Pointer to data needed to be sent

    const char *httpHead;

}client_t;

//!!-------------------------------------------------------

typedef void (*server_callback_t)(void *);

#define SERVER_STATE_STOPPED  0
#define SERVER_STATE_RUNNING  1

typedef struct 
{   
    uint8_t     state;
    timer_t    *timer;
    client_t   *client;
    wifi_t     *wifi;
    server_callback_t callback;
}server_t;

void Server_Init(const char *ssid, const char *pass, uint8_t mode, os_callback_t callback);

void Server_Service(void *evt);
void Server_ProcessLine(const char *line);

#endif //!! __SERVER_H__
