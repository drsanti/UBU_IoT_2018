#ifndef __WIFI_H__
#define __WIFI_H__

#include "esp.h"



#define WIFI_AP_CHANNEL_0               0
#define WIFI_AP_CHANNEL_1               1
#define WIFI_AP_CHANNEL_2               2
#define WIFI_AP_CHANNEL_3               3
#define WIFI_AP_CHANNEL_4               4


#define WIFI_AP_ENCRYPY_OPEN            0
//!! 1 is not defined, jump to 2
#define WIFI_AP_ENCRYPT_WPA_PSK         2  
#define WIFI_AP_ENCRYPT_WPA2_PSK        3
#define WIFI_AP_ENCRYPT_WPA_WPA2_PSK    4


//!! WiFi modes
#define WIFI_MODE_STATION           0x01   //!! Station only
#define WIFI_MODE_SOFT_AP           0x02   //!! Access point (AP) or Host
#define WIFI_MODE_AP_STATION        0x03   //!! Botr Station and AP


#define WIFI_STATE_DISCONNECTED     0
#define WIFI_STATE_CONNECTED        1
#define WIFI_STATE_GOT_IP_ADDR      2

typedef struct 
{
    uint8_t         state;
    os_callback_t   callback;           //!! State changed callback function
    const char      *ssid;              //!! Pointer to SSID
    const char      *pass;              //!! Pointer to PASS
    uint8_t         mode;               //!! Operation mode
    char            ip[16];             //!! xxx.xxx.xxx.xxx
    char            mac[18];            //!! 5c:cf:7f:23:e1:08 

    //!!----------------------------
    const char      *ap_ssid;           //!! Pointer to AP_SSID
    const char      *ap_pass;            //!! Pointer to AP_PASS
    const char      *ap_ip;             //!! Pointer to AP_IP_ADDRESS
    uint8_t          ap_chan;           //!! Cnannel [0, 4]
    uint8_t          ap_encr;           //!! Operation mode [0, 2, 3, 4]

}wifi_t;

void WiFi_InitStation( const char *ssid, const char *pass, uint8_t mode );
void WiFi_InitSoftAP( const char *ap_ssid, const char *ap_pass, const char *ap_ip, uint8_t ap_chan, uint8_t ap_encr );
void WiFi_Start( os_callback_t callback );
void WiFi_Service( void *evt );


#endif //!! __WIFI_H__