/**************************************************/
/* Embedded C-Programming Example based on ECC-OS */
/*                Dr.Santi Nuratch                */
/*    Embedded Computing and Control Laboratory   */
/*           ECC-Lab | INC@KMUTT | Thailand       */
/**************************************************/
/* Description:                                   */
/* Embedded Firmware for server (INC281)          */
/**************************************************/
/* Update:                                        */
/* 27 March 2018:                                 */
/*   Initial version                              */
/* 28 March 2018:                                 */
/*   Add a "sndset" (for beep), fixed some bugs   */
/**************************************************/

/**************************************************/
/*               Supported Commands               */
/**************************************************/
/*
 * ledset, id\r\n
 * ledoff, id\r\n
 * ledinv, id\r\n
 * ledget, id\r\n
 * ledfls, id, time\r\n
 * ledpwm, id, period, shif, ton\r\n
 * pswget, id\r\n
 * adcget, id\r\n
 * sndset, id, freq, period, power\r\n
 */

/**************************************************/
/*                  Include Files                 */
/**************************************************/
#include "os.h" // Require all functions of the OS



/**************************************************/
/*          Global Variables and TypeDef          */
/**************************************************/

typedef struct {
    uint8_t  id;
    uint16_t val;
    bool     err;
}target_t;


/**************************************************/
/*               Debuging functions               */
/**************************************************/
#define _USE_DEBUG_
#ifdef _USE_DEBUG_
#define debug_print(s) Uart2_AsyncWriteString(s)
#endif

/**************************************************/
/*                Helper functions                */
/**************************************************/
bool isint(const char *line) {
    int len = strlen(line);
    int i = 0;
    char c = line[i];
    if(c == '-' || c == '+') {
        i++;
    }
    for(; i<len; i++) {
        c = line[i];
        if( c < '0' || c > '9' ) {
            return false;
        }
    }
    return true;
}


int ExecuteCommand(const char *line, target_t *target) {

    int id, idx0, idx1, idx2, idx3, idx4, len;

    #ifdef _USE_DEBUG_
    debug_print("Executing: ");
    debug_print(line);
    #endif

    //!! buffer
    char buff[48];

    //!! Initialse with err, it will be changed to non-err (false) later
    target->err = true;


    len = strlen(line);
    //!! check \r and \n
    if(line[len-2] != '\r' || line[len-1] != '\n') {
        #ifdef _USE_DEBUG_
        debug_print("Error: No \\r\\n is detected\r\n");
        #endif
        return -1;
    }

    //!! get position of the '\r'
    idx0 = str_index_of_first_char(line, '\r');

    //!! get position of the ','
    idx1 = str_index_of_first_char(line, ',');
    if(idx1 == 0){
        #ifdef _USE_DEBUG_
        debug_print("Error: No commar (,) is detected\r\n");
        #endif
        return -1;
    }

    //!!
    idx2 =  str_index_of_first_char(line + (idx1 + 1), ',');            // second parameter
    idx2 = (idx2 != -1) ? idx1 + idx2 + 1: -1;                          // actual position of the second ','
    

    //!!
    //!! Take the target id
    //!!
    int e = (idx2 == -1) ? idx0 : idx2;
    int n = e - idx1 - 1;         //!! Number of bytes to be copied
    memcpy(buff, line + (idx1 + 1), n);
    buff[n] = 0;

    //!! Check if it is a number
    if( !isint(buff) ) {
        #ifdef _USE_DEBUG_
        debug_print("Error: The id must be a number\r\n");
        #endif
        return -1;
    }

    //!! Only 0, 1, 2 and 3 are accepted
    id = atoi(buff);
    if(id < 0 || id > 3) {        //!! id can only be  0-3 in this system
        #ifdef _USE_DEBUG_
        debug_print("Error: The id must be 0, 1, 2 or 3\r\n");
        #endif
        return -1;
    }

    //!! Searching for third and fourth ','
    if(idx2 != -1) {
        idx3 =  str_index_of_first_char(line + (idx2 + 1), ',');        // third parameter
        idx3 = (idx3 != -1) ? idx2 + idx3 + 1: -1;                      // actual position of the third ','

        if(idx3 != -1) {
            idx4 =  str_index_of_first_char(line + (idx3 + 1), ',');    // fourth parameter   
            idx4 = (idx4 != -1) ? idx3 + idx4 + 1: -1;                  // actual position of the fourth ','
        }
    }


    int v2=-1, v3=-1, v4=-1;

    //!! Debug
    #ifdef _USE_DEBUG_
    debug_print("Information: ");
    sprintf(buff, "id:%d Idx0:%d Idx1:%d Idx2:%d Idx3:%d Idx4:%d\r\n", id, idx0, idx1, idx2, idx3, idx4);
    debug_print(buff);
    #endif

    if(idx2 != -1) {
        //!! If there is no third parameter, use index of \r (idx0) as an endning
        int endIdx = (idx3 == -1) ? idx0 : idx3;
        int nb = endIdx - idx2 - 1;         //!! Number of bytes to be copied
        memcpy(buff, line + idx2 + 1, nb);  //!! Copy
        buff[nb] = 0;                       //!! NULL
        //!! Check if it is a number
        if( !isint(buff) ) {
            #ifdef _USE_DEBUG_
            debug_print("Error: The second parameter must be a number\r\n");
            #endif
            return -1;
        }
        v2 = atoi(buff);                    //!! Convert to integer

        if(idx3 != -1) {                    //!! It contains the third parameter
            endIdx = (idx4 == -1) ? idx0 : idx4; //!! Use position of idx4 or '\r' as the Ending index 
            nb = endIdx - idx3 - 1;         //!! Number of bytes to be copied
            memcpy(buff, line + idx3 + 1, nb); //!! Copy
            buff[nb] = 0;                   // NULL
            //!! Check if it is a number
            if( !isint(buff) ) {
                #ifdef _USE_DEBUG_
                debug_print("Error: The third parameter must be a number\r\n");
                #endif
                return -1;
            }
            v3 = atoi(buff);                //!! Convert to integer

            if(idx4 != -1) {
                endIdx = idx0;                  //!! Use position of '\r' as the Ending index 
                nb = endIdx - idx4 - 1;         //!! Number of bytes to be copied
                memcpy(buff, line + idx4 + 1, nb); //!! Copy
                buff[nb] = 0;                   // NULL
                //!! Check if it is a number
                if( !isint(buff) ) {
                    #ifdef _USE_DEBUG_
                    debug_print("Error: The fourth parameter must be a number\r\n");
                    #endif
                    return -1;
                }
                v4 = atoi(buff);                //!! Convert to integer   
            }
        }

        //!! DEBUG
        #ifdef _USE_DEBUG_
        debug_print("Information: ");
        sprintf(buff, "v2:%d v3:%d v4:%d\r\n", v2, v3, v4);
        debug_print(buff);
        #endif
    }


    //!!
    //!! Update the target
    //!!
    target->id = id;

    //!! check command and take action of LEDs
    if( 0 == str_index_of_first_token(line, "ledset") ) {
        LED_ModeSet(id, LED_MODE_NORMAL);
        LED_On(id);
        target->err = false;
        target->val = LED_Get(id);
    }
    else if( 0 == str_index_of_first_token(line, "ledclr") ) {
        LED_ModeSet(id, LED_MODE_NORMAL);
        LED_Off(id);   
        target->err = false;
        target->val = LED_Get(id);
    }
    else if( 0 == str_index_of_first_token(line, "ledinv") ) {
        LED_ModeSet(id, LED_MODE_NORMAL);
        LED_Inv(id);    
        target->err = false;
        target->val = LED_Get(id);
    }
    else if( 0 == str_index_of_first_token(line, "ledget") ) {
        target->err = false;
        target->val = LED_Get(id);
    }
    //!! check command and take action of PSWs
    else if( 0 == str_index_of_first_token(line, "pswget") ) {
        target->err = false;
        target->val = PSW_Get(id);
    }
    //!! check command and take action of ADCs
    else if( 0 == str_index_of_first_token(line, "adcget") ) {
        target->err = false;
        target->val = ADC_Get(id);
    }
    else if( 0 == str_index_of_first_token(line, "ledfls") ) {
        if(idx2 == -1 || v2 < 1) {
            #ifdef _USE_DEBUG_
            debug_print("Error: The interval must be positive number");
            #endif
            return -1; // err
        }
        LED_ModeSet(id, LED_MODE_NORMAL);
        LED_Flash(id, v2);
        target->err = false;
        target->val = 1;
    }
    else if( 0 == str_index_of_first_token(line, "ledpwm") ) {
        if(idx2 == -1 || idx3 == -1 || idx4 == -1) {
            #ifdef _USE_DEBUG_
            debug_print("Error: The command is not supported");
            #endif
            return -1; // err
        }
        //!! v2: period
        //!! v3: phase shift (can be 0)
        //!! v4: on-time
        if(v2 < 0) {
            #ifdef _USE_DEBUG_
            debug_print("Error: The period must be positive number");
            #endif
            return -1; // err
        }
        if(v3 < 0) {
            #ifdef _USE_DEBUG_
            debug_print("Error: The phahe-shift must be positive number");
            #endif
            return -1; // err
        }
        if(v4 < 0) {
            #ifdef _USE_DEBUG_
            debug_print("Error: The time-on must be positive number");
            #endif
            return -1; // err
        }

        LED_ModeSet(id, LED_MODE_PWM);
        LED_PwmSet(id, v2, v3, v4);
        target->err = false;
        target->val = 1;
    }
    else if( 0 == str_index_of_first_token(line, "sndset") ) {
        //!! v2: period
        //!! v3: freq
        //!! v4: power
        if(v3 < 10 || v3 > 20000) {
            #ifdef _USE_DEBUG_
            debug_print("Error: The frquency must be set to 10 to 20k");
            #endif
            return -1;  //!! 10-20k are accepted
        }
        if(v4 < 0 || v4 > 100) {
            #ifdef _USE_DEBUG_
            debug_print("Error: The power must be set to 0 to 100");
            #endif
            return -1;  //!! 1-100 are accepted
        }
        if(v2 < 1 || v2 > 5000) {
            #ifdef _USE_DEBUG_
            debug_print("Error: The period must be set to 1 to 5000");
            #endif
            return -1;  //!! 1-5000mS are accepted
        }
        Beep_FreqSet(v3);
        Beep_PowerSet(v4);
        Beep(v2);
        target->err = false;
        target->val = 1;
    }
    else{
        #ifdef _USE_DEBUG_
        debug_print("Error: The command is not supported!");
        #endif
        return -1;    // err 
    }
    return 1;         // ok
} 


/**************************************************/
/*               Callback functions               */
/**************************************************/

#define RESP_TYPE_STATUS   0
#define RESP_TYPE_VALUE    1

void Uart1LineReceived(void *param) {
    //!! Pointer to void to pointer to uart_event_t
    uart_event_t *evt = (uart_event_t *)param;
    //!! Data structure
    data_t data = evt->data;
    //!! Line data
    const char *line = (const char *)data.buffer;
    //!! allocate memory
    int len =  strlen(line);
    char *buffer = (char *)malloc(len + 1);
    //!! copy
    int i, j;
    for(i=0, j=0; i<len; i++) {
        char c = line[i];
        //!! spaces and tabs are ignored
        if( (c != ' ') && (c != '\t') ) {
            //!! to lower case
            if( (c >= 'A') && (c <= 'Z') ) {
                c = c + 'a' - 'A';
            }
            buffer[j++] = c;
        }
    }
    //!! string terminate
    buffer[j] = 0; // NULL

    //!! Update new length
    len =  strlen(line);
    //!! target pbject
    target_t target;
    //!! response type
    int respType = RESP_TYPE_STATUS;

    //!! flash, ledpwm and sndset
    if( 0 == str_index_of_first_token(buffer, "ledfls") ||
        0 == str_index_of_first_token(buffer, "ledpwm") ||
        0 == str_index_of_first_token(buffer, "sndset")) {
        ExecuteCommand(buffer, &target);
        respType = RESP_TYPE_STATUS;
    }
    //!! led, psw and adc
    else if( 0 == str_index_of_first_token(buffer, "led") ||  
             0 == str_index_of_first_token(buffer, "psw") ||
             0 == str_index_of_first_token(buffer, "adc")) {
        ExecuteCommand( buffer, &target );
        respType = RESP_TYPE_VALUE;
    }
    else{
        
        #ifdef _USE_DEBUG_
        debug_print("Error: The command is not supported!");
        #endif

        //!! free the memory
        free(buffer);
        return;
    }

    //!!
    //!! Make a return message
    //!!

    //!! String termination at len-1 (remove the \r\n)
    buffer[len-2] = 0;

    //!! Echo the command
    Uart1_AsyncWriteString( buffer);

    //!! Send value (psw, adc)
    if(respType == RESP_TYPE_VALUE) {
        if( target.err ) {
            Uart1_AsyncWriteString(",err,-1\r\n");
        }
        else {
            sprintf(buffer, ",ok,%d\r\n", target.val);
            Uart1_AsyncWriteString( buffer );
        }
    }
    //!! Send status (err, ok)
    else {
        Uart1_AsyncWriteString( target.err ? ",err,-1\r\n" : ",ok,1\r\n" );   
    }

    //!! free the memory
    free(buffer);
}


/**************************************************/
/*                  main function                 */
/**************************************************/
int main(void) {

    OS_Init();	// Initialise the OS

    //!! Register UART1 Line Received Callback
    OS_Uart1SetLineReceivedCallback(Uart1LineReceived);

    Uart1_AsyncWriteString("Enter a command\r\n");

    OS_Start();	// Start the OS
}
