
/*
 ***************************************************************
 * Ex12. Example of HTTP Server
 ***************************************************************
 * Dr.Santi Nuratch
 * Embedded Computing and Control Laboratory (ECC-Lab)
 * Department of Control System and Instrumentation Engineering
 * Faculty of Engineering, KMUTT
 * 18 September, 2018
 ***************************************************************
 */

#include "os.h"
#include "server.h"

const char *SSID = "SSID";            //!! SSID
const char *PASS = "*******";         //!! PASS


void GetCallback(void *evt) 
{
    server_t *server = (server_t *)evt;

    if(server->client != NULL)
    {
        server->client->data = "TEST DATA";
    }
}
int main(void) 
{
    OS_Init();                          //!! Initial the OS
    HTTP_ServerInit(SSID, PASS, GetCallback);  //!! Initialise the HTTP Server
    OS_Start();                         //!! Start OS
}
