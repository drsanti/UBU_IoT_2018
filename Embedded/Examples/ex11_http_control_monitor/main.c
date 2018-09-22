/*
 ***************************************************************
 * Ex11. Simple HTTP Monitoring and Control Application
 ***************************************************************
 * Dr.Santi Nuratch
 * Embedded Computing and Control Laboratory (ECC-Lab)
 * Department of Control System and Instrumentation Engineering
 * Faculty of Engineering, KMUTT
 * 18 September, 2018
 ***************************************************************
 */


#include "os.h"
#include "at2.h"
#include "wifi2.h"
#include "server2.h"

//!!--------------------------------------------------------------------------------------------
const char      *SSID       =   "ECC-Mobile";           //!! WiFi SSID
const char      *PASS       =   "**********";           //!! WiFi PASS

//!!--------------------------------------------------------------------------------------------
//!! SoftAP
const char      *AP_SSID    = "ECC-SoftAP";             //!! SoftAP/Host SSID
const char      *AP_PASS    = "embedded";               //!! 8-character or longer are required
const char      *AP_IPADDR  = "192.168.0.1";            //!! IP address
const uint8_t   AP_CHANNEL  = 2;                        //!! 2, 3, 4, 5 work (0, 1 not work)
const uint8_t   AP_ENCRYPT  = WIFI_AP_ENCRYPT_WPA_PSK;  //!! Encryption type

//!!--------------------------------------------------------------------------------------------
//!! Note: In SoftAP, using external files, e.g.; jQuery, Bootstrap and others
//!!       are not posible. However. those files can be loaded from the cache of the browser
//!!       To make the files saved in the cache, just enter IP address of the server 
//!!       (IP address of Station, not SoftAP) 
//!!--------------------------------------------------------------------------------------------


//!!
//!! led:d:d;
//!!

#define print(s) Uart1_AsyncWriteString(s)




int get_id_value( const char *line, int is, int ie ) {
    char buff[10];
    int n = ie - is - 1;
    if( n >= 8 ){
        return -1;
    }
    memcpy(buff, line + is + 1, n);
    buff[n] = 0; // Null terminate
    return( atoi( buff ) );   
}

char * Command_Executor(const char *line)
{
    int idx1 = str_index_of_first_char(line, '/');
    int idx2 = str_index_of_last_char (line, '/');
    int idx3 = str_index_of_first_char(line, ';');

    if (idx1 < 0 || idx2 < 0 || idx3 < 0 || idx2 <= idx1 || idx3 <= idx2)
    {
        //!! Wronf format!
        print("The "); print(line); print(" is not supported!\r\n");
        return NULL;
    }

    int id = get_id_value( line, idx1, idx2 );
    int val = get_id_value( line, idx2, idx3 );

    //!!
    //!! LEDs
    //!!
    if( 0 == str_index_of_first_token( line, "led" ) )
    {
        if(val == 0) 
        {   //!! OFF
            LED_Off(id);
        }
        else if( val == 1)
        {   //!! ON
            LED_On(id);
        }
        else if( val == 2)
        {   //!! TOGGLE
            LED_Inv(id);
        }

        char *pPage = (char *)malloc(64 * sizeof(char));
        const char *status = LED_Get(id) ? "ON " : "OFF";
        const char *class  = LED_Get(id) ? "danger" : "warning";
        memset(pPage, 0, 64);
        sprintf(pPage, "<h1 class='text-%s'>LED%d: %s</h1>", class, id, status);
        return ( pPage );
    }
    //!!
    //!! ADCs
    //!!
    else if( 0 == str_index_of_first_token( line, "adc" ) )
    {
        char *pPage = (char *)malloc(64 * sizeof(char));
        int adc = ADC_Get(id);
        memset(pPage, 0, 64);
        os_time_t time = OS_TimeGet();
        sprintf(pPage, "<h1 class='text-success'>%.2d:%.2d:%.2d LDR/ADC%d: %.4d</h1>", time.hh, time.mm, time.ss, id, adc);
        return ( pPage );
    }

    //!!
    //!! Beep
    //!!
    else if( 0 == str_index_of_first_token( line, "beep" ) )
    {
        Beep_FreqSet(id);   // frequency
        Beep(val);          // interval
        char *pPage = (char *)malloc(64 * sizeof(char));
        memset(pPage, 0, 64);
        sprintf(pPage, "<h1 class='text-success'>Beep %d Hz, %d mS</h1>", id, val);
        return ( pPage );
    }
    return NULL;
}


void ClientRequested(void *evt)
{
    client_t *client = ((server_t *)evt)->client;
    //Beep(50);
    client->data = Command_Executor(client->getBuffer);

}





int main(void) 
{

    OS_Init();   

    Beep_PowerSet(0.5);   

    LED_ModeSet( LED_ID_3, LED_MODE_PWM );
    LED_PwmSet( LED_ID_3, 100, 0, 2 );

    Uart1_WriteString("\r\nHTTP Server Application!\r\n"); 



    uint8_t mode;
    if( PSW1_Get() ) {
        mode = 1;  
    }
    else if( PSW2_Get() ) {
        mode = 2;  
    }
    else if( PSW3_Get() ) {
        mode = 3;    
    }
    else{
        mode = 3;     
    }

    WiFi_Init( SSID, PASS, mode, ClientRequested );
    WiFi_InitSoftAP( AP_SSID, AP_PASS, AP_IPADDR, AP_CHANNEL, AP_ENCRYPT );

    OS_Start();                                
}
