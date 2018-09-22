
#include "config.h"
#include "wifi2.h"
#include "at2.h"
#include "server2.h"
#include "os.h"



#define wifi_print(msg) (Uart1_AsyncWriteString(msg)) 
extern at_command_t at;
wifi_t wifi;
void WiFi_StartSoftAP(void);
worker_t *server_start_worker = NULL;


static void WiFi_PrintInfo( void ) 
{
    if( wifi.mode != WIFI_MODE_SOFT_AP )
    {
        //!! TCP (Mode 1 and/or 3)
        Uart1_AsyncWriteString( "\r\n-----------------------\r\n" );
        Uart1_AsyncWriteString( "Station: ");
        Uart1_AsyncWriteString( wifi.ip);
        Uart1_AsyncWriteString( "\r\n");

        Uart1_AsyncWriteString( "Network: ");
        Uart1_AsyncWriteString( wifi.ssid);
        Uart1_AsyncWriteString( "\r\n-----------------------\r\n" );
    }

    if( (wifi.mode & WIFI_MODE_SOFT_AP) != 0 )
    {
        //!! AP (Mode 1 only)
        Uart1_AsyncWriteString( "\r\n-----------------------\r\n" );
        Uart1_AsyncWriteString( "SoftAP:  ");
        Uart1_AsyncWriteString( wifi.ap_ip);
        Uart1_AsyncWriteString( "\r\n");

        Uart1_AsyncWriteString( "Network: ");
        Uart1_AsyncWriteString( wifi.ap_ssid);
        Uart1_AsyncWriteString( "\r\n-----------------------\r\n" );
    }

    Beep( 50 );
} //!! WiFi_PrintInfo()


static void WiFi_UpdateIpMac( const char *line ) {
    if( line[7] == 'S' )
    {
        //!!
        //!! Station
        //!!
        // recv: +CIFSR:STAIP,"192.168.1.131"
        // recv: +CIFSR:STAMAC,"2c:3a:e8:11:1e:95"
        uint8_t idx1 = str_index_of_first_char( line, '"' );
        uint8_t idx2 = str_index_of_last_char( line, '"' );
        idx1 += 1;
        if( line[10] == 'I' ) 
        {
            memcpy( wifi.ip, line + (idx1), (idx2-idx1) );
        }
        else if( line[10] == 'M') 
        {
            memcpy( wifi.mac, line + (idx1), (idx2-idx1) );  
        }
    }
    else if( line[7] == 'A' )
    {
        //!!
        //!! SoftAP
        //!!
        // recv: +CIFSR:APIP,"192.168.0.123"
        // recv: +CIFSR:APMAC,"2e:3a:e8:11:1e:95"

        //!! Do nothing for now
    }
} //!! WiFi_UpdateIpMac


void WiFi_Service(void) 
{
    static uint16_t prv_sec = 0;
    uint16_t cur_sec = OS_TimeGet().ss;

    if( prv_sec != cur_sec ) 
    {   
        prv_sec = cur_sec;

        if( wifi.state == WIFI_STATE_DISCONNECTED ) 
        {
            
        }
        else if( wifi.state == WIFI_STATE_CONNECTED ) {
           
        }
        else if( wifi.state == WIFI_STATE_GOT_IP ) 
        {
            
        }
        else if( wifi.state == WIFI_STATE_STARTING_SRV ) 
        {
            
        }
        else if( wifi.state == WIFI_STATE_SERVER_RUNNING ) 
        {
            
        }
    }


    //!!
    if( wifi.state == WIFI_STATE_STARTING_SRV ) 
    {
        if( at.returnCode == AT_RETURN_ERROR ||  at.returnCode == AT_RETURN_FAIL )
        {
            //!!
            //!! In this point, the server cannot be started. 
            //!!
            wifi.state = WIFI_STATE_DISCONNECTED;


            WiFi_Restart();   //!! Reconnect to the network
        }
    }


} //!! WiFi_Service


//!! Note: Must be called after the Server is started!
void WiFi_StartSoftAP(void)
{
    if( ( wifi.mode & WIFI_MODE_SOFT_AP ) != 0 ) 
    {
        //!! Configuration of softAP mode
        char *pCWSAP = malloc( strlen(wifi.ap_ssid) + strlen(wifi.ap_pass) + 22 );
        sprintf( pCWSAP, "AT+CWSAP=\"%s\",\"%s\",%d,%d\r\n", wifi.ap_ssid, wifi.ap_pass, wifi.ap_chan, wifi.ap_encr );
        AT_Put( pCWSAP );
        free( (void *)pCWSAP );

        //!! Set ip address of ESP8266 softAP
        char *pCIPAP = malloc( 48 );
        sprintf( pCIPAP, "AT+CIPAP=\"%s\"\r\n", wifi.ap_ip );
        AT_Put( pCIPAP );
        free( (void *)pCIPAP );


        //!! Ask of AP_SSID and AP_IP of the SoftAP
        AT_Put( "AT+CWSAP?\r\n" ); 
        AT_Put( "AT+CIPAP?\r\n" ); 
    }
} // WiFi_StartSoftAP()

void Worker_StartHTTPServer(void *evt)  
{
    event_t  *event  = (event_t *)evt;
    worker_t *worker = (worker_t *)event->sender;

    if(worker->state == 0)  
    {
        Uart1_AsyncWriteString( "WiFi: Starting HTTP Server...\r\n" );
        AT_Put( "AT+CIPMUX=1\r\n" ); 
        worker->state++;
        OS_WorkerSleep(worker, 500);
        wifi.state = WIFI_STATE_STARTING_SRV;
        
    }
    else if(worker->state == 1) 
    {
        if(wifi.state == WIFI_STATE_STARTING_SRV)
        {
            AT_Put( "AT+CIPSERVER=1,80\r\n" );
            worker->state++;
            OS_WorkerSleep(worker, 500);
        }
    }
    else if(worker->state == 2) 
    {
        //!! Ask for IP and MAC addresses of the ICP/IP server
        if(wifi.state == WIFI_STATE_STARTING_SRV)
        {
            AT_Put( "AT+CIFSR\r\n" );
            worker->state++;
            OS_WorkerDelete(worker);
            server_start_worker = NULL;
            wifi.state = WIFI_STATE_SERVER_RUNNING;
        }
        //!! Turn off the echo of received line 
        wifi.recvEnabled = false;

        //!!
        WiFi_StartSoftAP();

        //!! If the WiFi is running in MODE2, print information in this point
        //!! For Mode1 and 3, the WiFi_PrintInfo() will be called from WIFI GOT IP
        // if( wifi.mode == WIFI_MODE_SOFT_AP )
        // {
        //     WiFi_PrintInfo();    
        // }
        
    }
} //!! Worker_StartHTTPServer



void WiFi_ProcessLine(const char *line) 
{
    if( 0 == str_index_of_first_token(line, "ready\r\n") )  
    {
        WiFi_ExecuteOnReady();
        Beep(20);
    }
    else if( 0 == str_index_of_first_token(line, "WIFI DISCONNECT\r\n") )  
    {

        if(wifi.state != WIFI_STATE_CONNECTED && wifi.state != WIFI_STATE_CONNECTING)
        {
            wifi_print("WiFi: Disconnected from network. Reconnecting...\r\n");    
        }

        wifi.state = WIFI_STATE_DISCONNECTED;
    }

    else if( 0 == str_index_of_first_token(line, "WIFI CONNECTED\r\n") )  
    {
        wifi_print("WiFi: Connected to the network!\r\n");
        wifi.state = WIFI_STATE_CONNECTED;
    }

    else if( 0 == str_index_of_first_token(line, "WIFI GOT IP\r\n") )  
    {
        wifi_print("WiFi: Got and IP address from the network!\r\n");
        wifi.state = WIFI_STATE_GOT_IP;

        //--------------------------------------------------------------------
        Uart1_AsyncWriteString( "WiFi: Got IP. Starting HTTP Server...\r\n" );
        wifi.state = WIFI_STATE_STARTING_SRV;

        if(server_start_worker != NULL) {
            OS_WorkerDelete(server_start_worker);
            server_start_worker = NULL;    
        }
        server_start_worker = OS_WorkerCreate("start", Worker_StartHTTPServer);

    }

    else if( 0 == str_index_of_first_token( line, "+CIFSR:APIP," )      ||
             0 == str_index_of_first_token( line, "+CIFSR:APMAC," )     ||
             0 == str_index_of_first_token( line, "+CIFSR:STAIP," )     ||
             0 == str_index_of_first_token( line, "+CIFSR:STAMAC," ) )  
    {
        
        //!! Update IP and MAC addresses
        WiFi_UpdateIpMac(line);

        //!! Print Information
        if( wifi.ip[0] != 0 && wifi.mac[0] != 0 )  {
            WiFi_PrintInfo();
        }
    }

    else if( 0 == str_index_of_first_token(line, "ready\r\n")  )
    {
        wifi_print("WiFi: Turn off the echo of received line\r\n");
        wifi.recvEnabled = false;
    }
    else {
        //
    }
} //!! WiFi_ProcessLine


const char *WiFiModeMsg[]={"[0]-Undefinded", "[1]-Station","[2]-SoftAP", "[3]-Station+softAP"};


void WiFi_ExecuteOnFail(void)
{
    //!!--- DEBUG ----------------------------------
    wifi_print("WiFi: WiFi state is ");
    char buff[32];
    sprintf(buff, "%d\r\n", wifi.state);
    wifi_print(buff);
    //!!--- END ------------------------------------


    if(wifi.state == WIFI_STATE_DISCONNECTED || wifi.state == WIFI_STATE_CONNECTING) 
    {
        WiFi_Restart();
    }
}


void WiFi_LineReceived(void *evt)
{
    //LED_Flash(LED_ID_0, 5);
    const char *line = (const char *)((uart_event_t *)evt)->data.buffer;

    if(wifi.recvEnabled == true)
    {
        wifi_print("//**");
        wifi_print(line);
    }

    AT_ProcessLine(line);
    WiFi_ProcessLine(line); 
    Server_ProcessLine(line);   
    Server_Service((void *)line);
}


static void WiFi_Looper(void *evt)
{
    AT_Service();
    WiFi_Service();
    Server_Service(NULL);
}



void WiFi_ExecuteOnReady(void)
{

    AT_Restart();

    if(wifi.mode != WIFI_MODE_SOFT_AP)
    {
        //!! MODE 1 and 3 need to connect to network
        //!! The Server will be started after the module receives IP

        //!! Connect to Network    
        //!!--- DEBUG ----------------------------------------------------
        wifi_print("WiFi: Connecting to network \"");
        wifi_print(wifi.ssid);
        wifi_print("\"\r\n");
        //!!--- END ------------------------------------------------------
        
        wifi.state = WIFI_STATE_CONNECTING;
        char *pCWJAP = malloc( strlen(wifi.ssid) + strlen(wifi.pass) + 17 );
        sprintf( pCWJAP, "AT+CWJAP=\"%s\",\"%s\"\r\n", wifi.ssid, wifi.pass );
        AT_Put( pCWJAP );
        free( (void *)pCWJAP );
    }
    else
    {
        // MODE 2
        if(server_start_worker != NULL) {
            OS_WorkerDelete(server_start_worker);
            server_start_worker = NULL;    
        }
        server_start_worker = OS_WorkerCreate("start", Worker_StartHTTPServer);
        
    }   
}

void WiFi_Restart(void) 
{
    //!! Reset the  wifi.state to WIFI_STATE_DISCONNECTED
    wifi.state = WIFI_STATE_DISCONNECTED;
   
    //!! Print all received line on startup
    wifi.recvEnabled = true;  

    //!! Restart the AT engine and delete all memories
    AT_Restart();

    //!! Kill the previous running server
    AT_Put("AT+CIPSERVER=0\r\n");

    //!! Restart the module and wait 0.1 seconds
    AT_Put("AT+RST\r\n"); Beep(10);

    //!! Wating for a short of time is required
    __delay_ms(100);

    //!! Restart the AT engine and delete all memories again
    AT_Restart();

    //!! Change the Operation Mode. This operation requires RESTART
    //!!--- DEBUG ----------------------------------------------------
    wifi_print("WiFi: Set WiFi mode to \"");
    wifi_print(WiFiModeMsg[wifi.mode]);   //!! mode = [1, 2, 3]
    wifi_print("\"\r\n");
    //!!--- END ------------------------------------------------------
    char *pCWMODE = malloc( 32 );
    sprintf( pCWMODE,"AT+CWMODE=%d\r\n", wifi.mode );
    AT_Put( pCWMODE );
    free( (void *)pCWMODE );

    //!! Changing the MODE requires RESTART!
    AT_Put( "AT+RST\r\n" ); Beep(10);
    wifi.recvEnabled = false;  
    wifi_print("WiFi: Restart the WiFi module...\r\n");

} //!! WiFi_Restart



void WiFi_Init(const char *ssid, const char *pass, uint8_t mode, os_callback_t callback)
{
    wifi.ssid = ssid;
    wifi.pass = pass;
    wifi.mode = mode;
    wifi.ip[0]  = 0;
    wifi.mac[0] = 0;
    wifi.ip[0]  = 0;
    wifi.mac[0] = 0;
   

    OS_Uart2SetLineReceivedCallback(WiFi_LineReceived);
    OS_TimerCreate("at", 10, 1, WiFi_Looper);
    AT_Init();
    Server_Init(ssid, pass, mode, callback);


    WiFi_Restart();


} //!! WiFi_Init





void WiFi_InitSoftAP( const char *ap_ssid, const char *ap_pass, const char *ap_ip, uint8_t ap_chan, uint8_t ap_encr )
{
    if( ( wifi.mode & WIFI_MODE_SOFT_AP ) == 0 ) 
    {
        wifi_print("WiFi: Warning!! The module is not set to SoftAP mode!\r\n");
    }

    //!!-------------------------------------------------------------
    wifi.ap_ssid = ap_ssid;
    wifi.ap_pass = ap_pass;
    wifi.ap_chan = ap_chan;
    wifi.ap_encr = ap_encr;
    wifi.ap_ip   = ap_ip;

} //!! WiFi_InitSoftAP()