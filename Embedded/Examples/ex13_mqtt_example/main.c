/*
 ***************************************************************
 * Ex13. Simple MQTT Monitoring and Control Application
 ***************************************************************
 * Dr.Santi Nuratch
 * Embedded Computing and Control Laboratory (ECC-Lab)
 * Department of Control System and Instrumentation Engineering
 * Faculty of Engineering, KMUTT
 * 18 September, 2018
 ***************************************************************
 */
/*
  Supported commands:
    - led0-on
    - led0-off
    - led1-on
    - led1-off
    - led2-on
    - led2-off
    - led3-on
    - led4-off
    - beep
*/

#include "os.h"
#include "str.h"

#define MQTT_SERVER_DISCONNECTED    0
#define MQTT_SERVER_CONNECTED       1
typedef struct {
    uint8_t status;
}server_t;

server_t server;

#define MQTT_STATUS_DISCONNECTED    0
#define MQTT_STATUS_CONNECTED       1
#define MQTT_STATUS_SUBSCRIPED      2

#define MQTT_REQUEST_STATE_AT       0
#define MQTT_REQUEST_STATE_DATA     1

typedef struct {
    int8_t status;
    int8_t request_state;
}mqtt_t;
mqtt_t mqtt;

void MqttPing(void *evt) {
    
    LED0_Inv();
    return;
    event_t *event = (event_t *)evt;
    worker_t *worker = (worker_t *)event->sender;
    if( worker->state == 0) {
        Uart1_AsyncWriteString("\r\n<< PING >>\r\n");
        Uart2_AsyncWriteString("AT+CIPSEND=0,2\r\n");
        worker->state++;
        OS_WorkerSleep(worker, 1000);
        return;
    }
    uint8_t mqtt[2];
    uint8_t i = 0;
    mqtt[i++] = 0xc0;
    mqtt[i++] = 0x00;
    Uart2_AsyncWriteBytes(mqtt, i);
    worker->state = 0;
    OS_WorkerSleep(worker, 9500);
}

void Mqtt_Connect(uint8_t request_state) {
    uint8_t mqtt[64];
    uint8_t i=0;
    mqtt[i++] = 0x10;           // Connect
    mqtt[i++] = 0x00;           // Length of header frame, will be update later
    mqtt[i++] = 0x00;           // Protocol Name Length MSB
    mqtt[i++] = 0x04;           // Protocol Name Length LSB (4:=MQTT)
    mqtt[i++] = (uint8_t)'M';
    mqtt[i++] = (uint8_t)'Q';
    mqtt[i++] = (uint8_t)'T';
    mqtt[i++] = (uint8_t)'T';
    mqtt[i++] = 0x04;           // Version
    mqtt[i++] = 0x02;           // Connection flag
    mqtt[i++] = 0x00;           // Keep alive time MSB
    mqtt[i++] = 0x0A;           // Keep alive time LSB (10 sec)
    mqtt[i++] = 0x00;           // Client Name Length MSB
    mqtt[i++] = 0x06;           // Client Name Length (6:=EccLab)
    mqtt[i++] = (uint8_t)'E';
    mqtt[i++] = (uint8_t)'c';
    mqtt[i++] = (uint8_t)'c';
    mqtt[i++] = (uint8_t)'L';
    mqtt[i++] = (uint8_t)'a';
    mqtt[i++] = (uint8_t)'b';
    mqtt[1]   = i-2;
    if(request_state == MQTT_REQUEST_STATE_AT) {
        char cmd_buffer[32];
        sprintf(cmd_buffer, "AT+CIPSEND=0,%d\r\n", i);
        Uart2_AsyncWriteString(cmd_buffer);// __delay_ms(500);
    }
    else{
        Uart2_AsyncWriteBytes(mqtt, i);
    }
}

void MqttSubscript(void) {
    uint8_t mqtt[64];
    uint8_t i = 0;
    
    mqtt[i++] = 0x82;               // Subscript
    mqtt[i++] = 0x00;               // Remain Length, will be updated below  
    mqtt[i++] = 0xaa;               // Message ID MSB  
    mqtt[i++] = 0xbb;               // Message ID LSB  
    mqtt[i++] = 0x00;               // Topic Name Length MSB  
    mqtt[i++] = 0x03;               // Topic Name Length LSB (3:=Ecc) 
    mqtt[i++] = (uint8_t)'E';       // 'E', Topic Name, 'Ecc'  
    mqtt[i++] = (uint8_t)'c';       // 'c', Topic Name, 'Ecc'  
    mqtt[i++] = (uint8_t)'c';       // 'c', Topic Name, 'Ecc'  
    mqtt[i++] = 0x00;               // Requested QoS : 0  
    mqtt[1]   = i-2;
    char cmd_buffer[32];
    sprintf(cmd_buffer, "AT+CIPSEND=0,%d\r\n", i);
    Uart2_AsyncWriteString(cmd_buffer); 
    __delay_ms(200);
    Uart2_AsyncWriteBytes(mqtt, i);
}

void MqttPublish(void) {

    if (mqtt.status != MQTT_SERVER_CONNECTED)
    {
        LED_Flash(LED_ID_3, 20);
        return;
    }

    uint8_t mqtt[64];
    uint8_t i = 0;

    const char *PUB_TOPIC = "EccLab";
    //const char *PUB_MESSAGE = "HELLO WORLD!\r\n";

    char payload_buffer[32];
    sprintf(payload_buffer, "Light/ADC1: %d\r\n", ADC1_Get());
    const char *PUB_MESSAGE = payload_buffer;

    mqtt[i++] = 0x30;                           // MQTT Publish
    mqtt[i++] = 0xFF;                           // Remaining Length, calculated later
    mqtt[i++] = 0x00;                           // Topic Length MSB
    mqtt[i++] = strlen(PUB_TOPIC);              // Topic Length LSB

    //!! Copy the publish-topic
    uint8_t j;
    for (j = 0; j < strlen(PUB_TOPIC); j++)
    {
        mqtt[i++] = PUB_TOPIC[j];               // publish topic name
    }

    //!! Copy the publish-message
    for (j = 0; j < strlen(PUB_MESSAGE); j++)
    {
        mqtt[i++] = PUB_MESSAGE[j];             // publish message
    }

    //!! Calculate the Remaining Length
    mqtt[1] = (i - 2);                          // Remaining Length

    //!! Creates and Sends to the ESP8266
    char cmd_buffer[32];
    sprintf(cmd_buffer, "AT+CIPSEND=0,%d\r\n", i);
    Uart2_AsyncWriteString(cmd_buffer);
    __delay_ms(200);
    Uart2_AsyncWriteBytes(mqtt, i);
}

void PeriodicPublishing (void *evt)
{
    MqttPublish();
}

#define AT_STATUS_READY 0
#define AT_STATUS_BUSY  1
    typedef struct {
    uint8_t status;
    uint16_t timeout;
}at_t;
at_t at;

#define ESP_STATUS_NOT_FOUND        0
#define ESP_STATUS_POWER_ON         1
#define ESP_STATUS_AT_CWMODE        2
#define ESP_STATUS_AT_CIPMUX        3
#define ESP_STATUS_INITIALISED      4

typedef struct {
    uint8_t status;
}esp_t;
esp_t esp;



#define WIFI_STATUS_DISCONNECTED    0
#define WIFI_STATUS_CONNECTED       1
#define WIFI_STATUS_GOT_IP          2

typedef struct {
    uint8_t status;
}wifi_t;
wifi_t wifi;





void  ESP_ByteReceived(void *evt) {
    uart_event_t *e = (uart_event_t *)evt;
    Uart1_AsyncWriteData(&e->data);
}

void ESP_LineReceived(void *evt) {
    uart_event_t *e = (uart_event_t *)evt;
    //Uart1_AsyncWriteData(&e->data);
    
    uint8_t *line   = e->data.buffer;
    uint16_t length = e->data.length;
    uint8_t at_ok = 0;
    
    //!!
    //!! AT
    //!!
    if(at.status == AT_STATUS_BUSY) {
        if(-1 != str_raw_index_of_first_token((char *)line, length, (char *)"OK\r\n")) {
            at.status = AT_STATUS_READY;
            at_ok = 1;
            UART1_AsyncWriteString("Command executed\r\n");
        }
        else if(-1 != str_raw_index_of_first_token((char *)line, length, (char *)"ERROR\r\n")) {
            at.status = AT_STATUS_READY;
            UART1_AsyncWriteString((char *)line);
        }
        else {
            UART1_AsyncWriteString(">> ");
            UART1_AsyncWriteString((char *)line);
        }
    }
    
    
    //!!
    //!! ESP
    //!!
    //!! After the ESP8266 is restarted, the "ready\r\n" is received
    if(esp.status == ESP_STATUS_NOT_FOUND) {
        if(-1 != str_raw_index_of_first_token((char *)line, length, (char *)"ready\r\n")) {
            esp.status = ESP_STATUS_POWER_ON;
            UART1_AsyncWriteString("WiFi Ready\r\n");
        }
    }
    else if(at_ok && esp.status == ESP_STATUS_POWER_ON){
        esp.status = ESP_STATUS_AT_CWMODE;
        UART1_AsyncWriteString("WiFi ESP_STATUS_AT_CWMODE\r\n");
    }
    else if(at_ok && esp.status == ESP_STATUS_AT_CWMODE){
        esp.status = ESP_STATUS_AT_CIPMUX;
        UART1_AsyncWriteString("WiFi ESP_STATUS_AT_CIPMUX\r\n");
    }

    
    
    
    //!! After the ESP8266 is restarted, the "WIFI DISCONNECT\r\n" is received
   if(-1 != str_raw_index_of_first_token((char *)line, length, (char *)"WIFI DISCONNECT\r\n")) {
        wifi.status = WIFI_STATUS_DISCONNECTED;
        UART1_AsyncWriteString("WiFi Disconnected\r\n");
    }
    //!! After the ESP8266 connected to AP, the "WIFI CONNECTED\r\n" is received
    else if(-1 != str_raw_index_of_first_token((char *)line, length, (char *)"WIFI CONNECTED\r\n")) {
        wifi.status = WIFI_STATUS_CONNECTED;
        UART1_AsyncWriteString("WiFi Connected\r\n");
    }
    //!! After the ESP8266 received IP address, the "WIFI GOT IP\r\n" is received
    else if(-1 != str_raw_index_of_first_token((char *)line, length, (char *)"WIFI GOT IP\r\n")) {
        wifi.status = WIFI_STATUS_GOT_IP;
        UART1_AsyncWriteString("WiFi Got IP Address\r\n");
    }

    //!! Server
    if(server.status == MQTT_SERVER_DISCONNECTED) {
        if(-1 != str_raw_index_of_first_token((char *)line, length, (char *)"0,CONNECT\r\n") || 
           -1 != str_raw_index_of_first_token((char *)line, length, (char *)"ALREADY CONNECTED\r\n")) 
        {
            server.status = MQTT_SERVER_CONNECTED;
            UART1_AsyncWriteString("Connected to MQTT server\r\n");
        }
    }
    
    
    //!! MQTT
    if(mqtt.status == MQTT_SERVER_DISCONNECTED) {
        if(mqtt.request_state == MQTT_REQUEST_STATE_AT) {
            if(at_ok) {
                mqtt.request_state = MQTT_REQUEST_STATE_DATA;
                UART1_AsyncWriteString("MQTT --> DATA\r\n");
                Mqtt_Connect(MQTT_REQUEST_STATE_DATA);
            }
        }
        else if(mqtt.request_state == MQTT_REQUEST_STATE_DATA) {
            if(-1 != str_raw_index_of_first_token((char *)line, length, (char *)"SEND OK\r\n")) {
                mqtt.status = MQTT_SERVER_CONNECTED;
                UART1_AsyncWriteString("Switched to MQTT Protocol\r\n");
                MqttSubscript();
                OS_WorkerCreate("MqttPing", MqttPing);
                OS_Uart2SetRxCallback(ESP_ByteReceived);
            }
        }
    }


    //
    if(server.status == MQTT_SERVER_CONNECTED) {
        if(-1 != str_raw_index_of_first_token((char *)line, length, (char *)"+IPD,")) { 
            int i1 = str_raw_index_of_first_token((char *)line, length, (char *)"Ecc");
            if(-1 != i1) { 
                char *p = (char *)(line+i1);
                if( strstr( p, "led0-on" ) ) {
                    LED0_Set();
                }
                else if( strstr( p, "led0-off" ) ) {
                    LED0_Clr();
                }
                else if( strstr( p, "led1-on" ) ) {
                    LED1_Set();
                }
                else if( strstr( p, "led1-off" ) ) {
                    LED1_Clr();
                }
                else if( strstr( p, "led2-on" ) ) {
                    LED2_Set();
                }
                else if( strstr( p, "led2-off" ) ) {
                    LED2_Clr();
                }
                else if( strstr( p, "led3-on" ) ) {
                    LED3_Set();
                }
                else if( strstr( p, "led3-off" ) ) {
                    LED3_Clr();
                }
                else if( strstr( p, "beep" ) ) {
                    Beep(100);
                }
            } 
        }  
    }
   
}



int wait_at_ticks = 0;
int wait_ap_ip_ticks = 0;
void WiFiWorker(void *evt) {
    event_t *event = (event_t *)evt;
    worker_t *worker = (worker_t *)event->sender;
    
    //!! The at-command is executing, just wait
    if(at.status == AT_STATUS_BUSY) {
        Uart1_AsyncWriteString("AT Processing...\r\n");
        if(wait_at_ticks++ > 20) {
            wait_at_ticks = 0;
            at.status = AT_STATUS_READY;
            Uart1_AsyncWriteString("AT Timeout!.\r\n");
            Beep(200);
        }
        OS_WorkerSleep(worker, 200);
        LED3_Inv();
        return;
    }
    
    //!! WiFi module is not detected, reset it and wait for its response
    if(esp.status == ESP_STATUS_NOT_FOUND) {
        Uart1_AsyncWriteString("Restarting WiFi...\r\n");
        at.status = AT_STATUS_BUSY;
        Uart2_AsyncWriteString("AT+RST\r\n");
        OS_WorkerSleep(worker, 2000);
        return;
    }
    //!! WiFi module is detected, apply the CWMODE=3
    else if(esp.status == ESP_STATUS_POWER_ON) {
        Uart1_AsyncWriteString("Setting WiFi -> CWMODE=3...\r\n");
        at.status = AT_STATUS_BUSY;
        Uart2_AsyncWriteString("AT+CWMODE=3\r\n");
        OS_WorkerSleep(worker, 500);
        return;
    }
    //!! the CWMODE=3 is already applied, now apply the CIPMUX=1
    else if(esp.status == ESP_STATUS_AT_CWMODE) {
        Uart1_AsyncWriteString("Setting WiFi -> CIPMUX=1...\r\n");
        at.status = AT_STATUS_BUSY;
        Uart2_AsyncWriteString("AT+CIPMUX=1\r\n");
        OS_WorkerSleep(worker,500);
        return;
    }
    //!! the CIPMUX=1 is already applied, now just wait for moving to ESP_STATUS_INITIALISED
    else if(esp.status == ESP_STATUS_AT_CIPMUX) {
        esp.status = ESP_STATUS_INITIALISED;
        Uart1_AsyncWriteString("The WiFi module is fully initialised!\r\n");
    }

    //!!
    if(wifi.status == WIFI_STATUS_DISCONNECTED) {
        Uart1_AsyncWriteString("Waiting for AP...\r\n");
        //!! too long waiting for AP, reset the WiFi module
        if(wait_ap_ip_ticks++ > 10) {
            Uart1_AsyncWriteString("Too long waiting for AP, Restart\r\n");
            wait_ap_ip_ticks = 0;
            esp.status  = ESP_STATUS_NOT_FOUND;
            wifi.status = WIFI_STATUS_DISCONNECTED;
        }
        OS_WorkerSleep(worker, 500);
        return;
    }
    //!! waiting for IP address
    else if(wifi.status == WIFI_STATUS_CONNECTED) {
        Uart1_AsyncWriteString("Waiting for IP address...\r\n");
        //!! too long waiting for IP address, reset the WiFi module
        if(wait_ap_ip_ticks++ > 10) {
            Uart1_AsyncWriteString("Too long waiting IP address, Restart\r\n");
            wait_ap_ip_ticks = 0;
            esp.status  = ESP_STATUS_NOT_FOUND;
            wifi.status = WIFI_STATUS_DISCONNECTED;
        }
        OS_WorkerSleep(worker, 500);
        return;
    }
    
    if(wifi.status != WIFI_STATUS_GOT_IP) {
        OS_WorkerSleep(worker, 500);
        return;
    }
    
    //!!
    if(server.status != MQTT_SERVER_CONNECTED) {
        at.status = AT_STATUS_BUSY;
        Uart1_AsyncWriteString("Connecting to MQTT server...\r\n");
        Uart2_AsyncWriteString("AT+CIPSTART=0,\"TCP\",\"198.41.30.241\",1883\r\n");
        OS_WorkerSleep(worker, 500);
        return;
    }

    if(mqtt.status == MQTT_STATUS_DISCONNECTED) {
        Uart1_AsyncWriteString("MQTT connecting...\r\n");
        at.status = AT_STATUS_BUSY;
        mqtt.request_state = MQTT_REQUEST_STATE_AT;
        Mqtt_Connect(MQTT_REQUEST_STATE_AT);
        OS_WorkerSleep(worker, 1000);
        return;
    }
    else if(mqtt.status == MQTT_STATUS_CONNECTED){
        Uart1_AsyncWriteString("MQTT Connected...\r\n");
        
        
        OS_WorkerDelete(worker);
    }

    OS_WorkerSleep(worker, 1000);
}




int main(void)  {

    OS_Init();
    
    UART1_AsyncWriteString("\r\nMQTT Client Running...\r\n");
    
    
    //LED_PwmSet(LED_ID_0, 500, 0,  1);
    //LED_PwmSet(LED_ID_1, 500, 50, 1);
    //LED_PwmSet(LED_ID_2, 500, 100, 1);
    //LED_PwmSet(LED_ID_3, 500, 150, 1);
    
    
    OS_Uart2SetLineReceivedCallback(ESP_LineReceived);
    OS_WorkerCreate("WiFi Worker", WiFiWorker);

    OS_TimerCreate("publisher", 2000, 1, PeriodicPublishing);

    //OS_Uart2SetRxCallback(ESP_ByteReceived);
    //OS_SwitchSetCallback(PSW_ID_0, psw_callback);
    //OS_SwitchSetCallback(PSW_ID_1, psw_callback);
    //OS_SwitchSetCallback(PSW_ID_2, psw_callback);
    //OS_SwitchSetCallback(PSW_ID_3, psw_callback);
	
	Beep_PowerSet(0.9);
    Beep(100);

    OS_Start();

}



