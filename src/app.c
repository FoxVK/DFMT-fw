 #define debughalt() __asm__ volatile (" sdbbp 0")
/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************


#include "app.h"
#include "tuner.h"
#include <usb/usb_device.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

#define AUDIO_BUFS_COUNT 3

AudioDataBufs AudioBufs[AUDIO_BUFS_COUNT];

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

typedef struct {
    struct {
    unsigned short Nothing:2;
    unsigned short BSTALL:1;
    unsigned short DTS:1;
    unsigned short NINC:1;
    unsigned short KEEP:1;
    unsigned short DATA01:1;
    unsigned short UOWN:1;
    unsigned short BYTE_COUNT;
    void * BUFFER_ADDRESS;
    } RX[2];

    struct {
    unsigned short Nothing:2;
    unsigned short BSTALL:1;
    unsigned short DTS:1;
    unsigned short NINC:1;
    unsigned short KEEP:1;
    unsigned short DATA01:1;
    unsigned short UOWN:1;
    unsigned short BYTE_COUNT;
    void * BUFFER_ADDRESS;
    } TX[2];

}BufferDT;

volatile BufferDT * BDT;

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    appData.suspended = false;
    appData.tuner_ready = 0;

    appData.usbDevHandle = -1;
    appData.audio_play = false;
    appData.noAudioData = 1;
    appData.reading = 0;
    appData.pingRequest = 0;
    appData.tuner_request = 0;
    appData.tuner_wait_for_reply = 0;

    appData.tunnel_write_handle = USB_DEVICE_TRANSFER_HANDLE_INVALID;
    
    int i,j;
    for(i=0; i<AUDIO_BUFS_COUNT; i++)
    {
        AudioBufs[i].isFree = 1;
        AudioBufs[i].trasfer_handle = USB_DEVICE_TRANSFER_HANDLE_INVALID;
        for(j=0; j<48; j++)
            AudioBufs[i].sample[j].l = AudioBufs[i].sample[j].r = j<24 ? 0:0xff;
    }
    tuner_init();
}


void APP_USBDeviceEventHandler
(
    USB_DEVICE_EVENT event,
    void * eventData,
    uintptr_t context
)
{
    USB_DEVICE_EVENT_DATA_CONFIGURED* configuredEventData;
    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
            U1TXREG = 'R';         
            //break;
        case USB_DEVICE_EVENT_DECONFIGURED:
        {
            USB_DEVICE_EndpointDisable(appData.usbDevHandle, AUDIO_EP);
            USB_DEVICE_EndpointDisable(appData.usbDevHandle, TUNNEL_EP_IN);
            USB_DEVICE_EndpointDisable(appData.usbDevHandle, TUNNEL_EP_OUT);
            appData.state = APP_STATE_USB_OPENED;
            appData.reading = 0;
            appData.audio_play = false;
            //TODO simulace cteni zvuku;
            int i;
            for(i=0; i<AUDIO_BUFS_COUNT; i++)
            {
                if(!AudioBufs[i].trasfer_handle != USB_DEVICE_TRANSFER_HANDLE_INVALID)
                {
                    USB_DEVICE_EndpointTransferCancel(appData.usbDevHandle, AUDIO_EP, AudioBufs[i].trasfer_handle);
                    AudioBufs[i].isFree = 1;
                    AudioBufs[i].trasfer_handle = USB_DEVICE_TRANSFER_HANDLE_INVALID;
                }
            }
            break;
        }
        case USB_DEVICE_EVENT_CONFIGURED:
            U1TXREG = '+';
            /* check the configuration */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED *)eventData;
            if(configuredEventData->configurationValue == 1)
            {

            }
            size_t size = sizeof(AudioBufs[0].sample);
            USB_DEVICE_EndpointEnable(appData.usbDevHandle, AUDIO_STREAMING_INTERFACE_ID, AUDIO_EP, USB_TRANSFER_TYPE_ISOCHRONOUS, size);
            USB_DEVICE_EndpointEnable(appData.usbDevHandle, TUNER_CONTROL_INTERFACE_ID, TUNNEL_EP_IN, USB_TRANSFER_TYPE_BULK, 32);
            USB_DEVICE_EndpointEnable(appData.usbDevHandle, TUNER_CONTROL_INTERFACE_ID, TUNNEL_EP_OUT, USB_TRANSFER_TYPE_BULK, 32);

            appData.state = APP_STATE_CONFIGURED;
            break;
        case USB_DEVICE_EVENT_SUSPENDED:
            U1TXREG = 'S';
            //appData.suspended = true;
            break;
        case USB_DEVICE_EVENT_RESUMED:
            U1TXREG = 's';
            appData.suspended = false;
            break;
        case USB_DEVICE_EVENT_POWER_DETECTED:
            USB_DEVICE_Attach (appData.usbDevHandle);
            break;
        case USB_DEVICE_EVENT_POWER_REMOVED:
            USB_DEVICE_Detach (appData.usbDevHandle);
            break;
        case USB_DEVICE_EVENT_ERROR:
            U1TXREG = 'E';
            break;

        case USB_DEVICE_EVENT_CONTROL_TRANSFER_SETUP_REQUEST:
        {
            USB_SETUP_PACKET * sp = eventData;
            if(sp->bRequest ==  0x0b && sp->wIndex == 0x0001) //set interace on if 1
            {
                //USB_DEVICE_ControlReceive(appData.usbDevHandle, &controlData, sizeof(controlData));

                if(sp->wValue == 0)
                {
                    int i;
                    appData.audio_play = false;

                    USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);
                    
                    U1TXREG = 'C';
                }
                else if(sp->wValue == 1)
                {
                    appData.audio_play = true;
                    USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);
                    
                    appData.noAudioData = 1;
                    U1TXREG = 'p';
                }
                else
                    USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            }
            else
                USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            break;
        }
            

        case USB_DEVICE_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:
        case USB_DEVICE_EVENT_CONTROL_TRANSFER_ABORTED:
        case USB_DEVICE_EVENT_CONTROL_TRANSFER_DATA_SENT:
            debughalt();
            break;

        case USB_DEVICE_EVENT_ENDPOINT_READ_COMPLETE:
        {
            appData.tunnel_read_count = ((USB_DEVICE_EVENT_DATA_ENDPOINT_WRITE_COMPLETE*)eventData)->length;
            if(appData.tunnel_read_data[0]&TUNNEL_PING_MASK)
                appData.pingRequest = 1;
            else
                appData.tuner_request = 1;

            //appData.reading = 0;
            U1TXREG = 'I';
            break;
        }

        case USB_DEVICE_EVENT_ENDPOINT_WRITE_COMPLETE:
        {
            USB_DEVICE_EVENT_DATA_ENDPOINT_WRITE_COMPLETE* ed = (USB_DEVICE_EVENT_DATA_ENDPOINT_WRITE_COMPLETE*)eventData;

            if(appData.tunnel_write_handle == ed->transferHandle)
            {
                appData.tunnel_write_handle = USB_DEVICE_TRANSFER_HANDLE_INVALID;
                U1TXREG = 'W';
            }
            else
            {
                int i;
                for(i=0; i<AUDIO_BUFS_COUNT; i++)
                {
                    if(AudioBufs[i].trasfer_handle == ed->transferHandle)
                    {
                        AudioBufs[i].trasfer_handle = USB_DEVICE_TRANSFER_HANDLE_INVALID;
                        AudioBufs[i].isFree = 1;
                        i=AUDIO_BUFS_COUNT;
                    }
                }
                appData.noAudioData = 1;
                //U1TXREG = 'W';
            }
            break;
        }
        default:
            debughalt();
            break;
    }
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            appData.suspended = false;
            appData.usbDevHandle = -1;


            /* Open the device layer */
            appData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE );

            if(appData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.usbDevHandle, APP_USBDeviceEventHandler, 0);
                appData.state = APP_STATE_USB_OPENED;
                BDT = ((BufferDT *)((USB_DEVICE_OBJ *)appData.usbDevHandle)->usbCDHandle);
            }
            break;
        }
            
        
        case APP_STATE_CONFIGURED:
        {
            APP_Task_configured_state();
            break; //APP_STATE_CONFIGURED
        }
            

        /* The default state should never be executed. */
        default:
        {
            break;
        }
    }

    app_tuner_updown_tasks();
}



void APP_Task_configured_state( void )
{
    //Tries to send sound frame if available
    if(appData.audio_play && appData.noAudioData)
    {

        /*static int b = 0;
        for(b=0; b<AUDIO_BUFS_COUNT;b++)
        {
            if((!AudioBufs[b].isFree) && AudioBufs[b].trasfer_handle == USB_DEVICE_TRANSFER_HANDLE_INVALID)
            {
                static void * data;
                data =  &(AudioBufs[b].sample);
                static size_t size ;
                size = sizeof(AudioBufs[b].sample);
                static USB_DEVICE_TRANSFER_HANDLE *handle;
                handle = &(AudioBufs[b].trasfer_handle);

                USB_DEVICE_RESULT r = USB_DEVICE_EndpointWrite(appData.usbDevHandle, handle, AUDIO_EP,data, size, USB_DEVICE_TRANSFER_FLAGS_DATA_COMPLETE);
                //debughalt();
                if(r == USB_DEVICE_RESULT_OK)
                {
                    //U1TXREG = 'w';
                    //U1TXREG = '0'+b;
                    b++;
                    if(b >= AUDIO_BUFS_COUNT) b=0;
                    appData.noAudioData = 0;

                }
                else if (r == USB_DEVICE_RESULT_ERROR_TRANSFER_QUEUE_FULL)
                {
                    //U1TXREG = 'f';
                    appData.noAudioData = 0;
                    AudioBufs[b].trasfer_handle = USB_DEVICE_TRANSFER_HANDLE_INVALID;
                }
                else
                {
                    //U1TXREG = 'e';
                    AudioBufs[b].trasfer_handle = USB_DEVICE_TRANSFER_HANDLE_INVALID;
                }
                b = AUDIO_BUFS_COUNT;
            }
        }*/

        AudioDataBufs *b = &AudioBufs[0];
        
        int i = tuner_audio_get(0, b->sample, 48*2);

        for(;i<48;i++)
            b->sample[i].l = b->sample[i].r = 125; //TODO remove this

        USB_DEVICE_RESULT r = USB_DEVICE_EndpointWrite(
                appData.usbDevHandle,
                &b->trasfer_handle,
                AUDIO_EP,
                b->sample,
                48*2,
                USB_DEVICE_TRANSFER_FLAGS_DATA_COMPLETE);

        appData.noAudioData = 0;
        b->trasfer_handle = USB_DEVICE_TRANSFER_HANDLE_INVALID;

    }

    //Allows receive a data for I2C tunel from host
    if(!appData.reading)
    {
        static USB_DEVICE_TRANSFER_HANDLE h; //we do not need handle
        volatile USB_DEVICE_RESULT r  = USB_DEVICE_EndpointRead(appData.usbDevHandle, &h, TUNNEL_EP_OUT, appData.tunnel_read_data, sizeof(appData.tunnel_read_data));
        if(r!=USB_DEVICE_RESULT_OK)
            Nop();
        appData.reading = 1;
        U1TXREG = 'i';
    }

    //Ping request was received
    if(appData.pingRequest)
    {
        volatile USB_DEVICE_RESULT r = USB_DEVICE_EndpointWrite(   appData.usbDevHandle, &appData.tunnel_write_handle,
                                    TUNNEL_EP_IN,
                                    &appData.tunnel_read_data, appData.tunnel_read_count,
                                    USB_DEVICE_TRANSFER_FLAGS_DATA_COMPLETE);
        if(r!=USB_DEVICE_RESULT_OK)
            Nop();
        U1TXREG = 'w';

        appData.pingRequest = 0;
        appData.reading = 0;

    }

    //A request for tuner was received
    if(appData.tuner_request)
    {
        int tuner_id = (appData.tunnel_read_data[0]&TUNNEL_TUNER_SELECT_MASK) ? 1 : 0 ;
        uint8_t * data = appData.tunnel_read_data;


        appData.tunnel_write_data[0] = 0; //reset status byte

        if(appData.tunnel_read_data[0] & TUNNEL_RW_SELECT_MASK)
        { //R
          //Second byte of read request is count of bytes to read
            tuner_read(tuner_id, appData.tunnel_write_data+1, appData.tunnel_read_data[1]);
        }
        else
        { //W
            size_t count = appData.tunnel_read_count-1;
            if(count > sizeof(appData.tunnel_write_data))
                    count = sizeof(appData.tunnel_write_data);
            tuner_write(tuner_id, data+1, count);
        }
        appData.reading = 0;
        appData.tuner_request = 0;
        appData.tuner_wait_for_reply = 1;
    }

    //We started communication with tuner and now we have to periodicallz check it's state
    if(appData.tuner_wait_for_reply)
    {
        Tuner_com_state st = tuner_com_state();
        if(st != TUNER_COM_BUSY)
        {   //Com is done
            if(st==TUNER_COM_IDLE || st==TUNER_COM_DEV_BUSY)
            {
                size_t size_to_send = 1;
                if(appData.tunnel_read_data[0] & TUNNEL_RW_SELECT_MASK)
                    size_to_send = tuner_rwed_bytes()+1; //was reading last 

                USB_DEVICE_EndpointWrite(   appData.usbDevHandle, &appData.tunnel_write_handle,
                                            TUNNEL_EP_IN,
                                            &appData.tunnel_write_data, size_to_send,
                                            USB_DEVICE_TRANSFER_FLAGS_DATA_COMPLETE);
            }
            else
            { //error
                appData.tunnel_write_data[0] = (uint8_t)st;
                USB_DEVICE_EndpointWrite(   appData.usbDevHandle, &appData.tunnel_write_handle,
                                            TUNNEL_EP_IN,
                                            &appData.tunnel_write_data, 1,
                                            USB_DEVICE_TRANSFER_FLAGS_DATA_COMPLETE);
            }

            appData.tuner_wait_for_reply = 0;
        }     
    }

    //TODO simulace cteni zvuku;
    /*
    int i;
    for(i=0; i<AUDIO_BUFS_COUNT; i++)
    {
        if(AudioBufs[i].isFree)
        {
             AudioBufs[i].isFree = 0;
            i = AUDIO_BUFS_COUNT;
        }
    }
    */
}

void app_tuner_updown_tasks()
{
#warning AZ TO BUDES ZKOUSET PRIPOJ SI ODPOR NA RESET !!!
    
    static uint8_t cmd_power_up[]   = {0x01, 0x00, 0xB0};
    static uint8_t cmd_power_down[] = {0x11};
    static uint8_t cmd_tune[]       = {0x20,0x01,0x24,0x9A,0x00}; //0x01 = inaccurate but fast tunning alowed to 93.7
    static uint8_t cmd_int_update[] = {0x14};
    static uint8_t cmd_int_clear[]  = {0x22, 0x01};
    static uint8_t cmd_dosr[]       = {0x12, 0x00, 0x01, 0x04, 0xBB, 0x80}; //FIXME 48000 = 0xBB80
    static Tuner_read_reply reply;
    static int wait_for_ready = 0;
    static int tuner_id = 0;

    static enum {
        ST_DOWN,
        ST_RST_HOLD, ST_RST_RELEASE, ST_PWRUP_S, ST_PWRUP_W, ST_TUN_S, ST_TUN_W, ST_TUNINT_CLR_S, ST_I2S_ON, ST_DASR_S, ST_DASR_W,
        ST_UP,
        ST_POWERING_DOWN,
    } state = ST_DOWN;



    static bool susp = false;
    if(susp!=appData.suspended)
    {
        U1TXREG = !appData.suspended ? 'A' : 'B';
        susp = appData.suspended;
    }

    if((   appData.suspended  && state!=ST_DOWN) ||
       ( (!appData.suspended) && state!=ST_UP))
           
    {

        Tuner_com_state tun_s = tuner_com_state();

        if( tun_s == TUNER_COM_BUSY)
            return;

        

        if(wait_for_ready)
        {
            static int asked = 0;
            if(tun_s == TUNER_COM_DEV_BUSY || (tun_s == TUNER_COM_IDLE)&&!asked)
            {
                tuner_read(tuner_id, &reply, 1);
                asked = 1;
                return;
            }
            else
            {
                asked = 0;
                wait_for_ready = 0;

                if(reply.STATUS.bits.ERR == 1) //TODO remove
                    Nop();
            }
        }

        if(appData.suspended)
            U1TXREG = '#';
        else
            U1TXREG = '*';
        
        switch(state)
        {
            case ST_DOWN:
                if(appData.suspended)
                    break;
            case ST_RST_HOLD:
                tuner_hold_in_rst(1);
                state = ST_RST_RELEASE;
                U1TXREG = '0';
                break;

            case ST_RST_RELEASE:
                tuner_hold_in_rst(0);
                //TODO rclk start
                state = ST_PWRUP_S;
                U1TXREG = '1';
                break;

            case ST_PWRUP_S:
                tuner_write(0, cmd_power_up, sizeof(cmd_power_up));
                wait_for_ready = 1;
                state = ST_PWRUP_W;
                U1TXREG = '2';
                break;

            case ST_PWRUP_W:
                if(tun_s != TUNER_COM_IDLE)
                    state = ST_PWRUP_S; //try it again
                else
                    state = ST_TUN_S;
                break;

            case ST_TUN_S:

                tuner_write(0, cmd_tune, sizeof(cmd_tune));
                wait_for_ready = 1;
                state = ST_TUN_W;
                U1TXREG = '3';
                break;

             case ST_TUN_W:
                 if(reply.STATUS.bits.STCINT)
                     state = ST_TUNINT_CLR_S;
                 else
                 {
                    tuner_write(tuner_id, cmd_int_update, sizeof(cmd_int_update));
                    wait_for_ready = 1;
                 }
                 
                break;

            case ST_TUNINT_CLR_S:
                tuner_write(tuner_id, cmd_int_clear, sizeof(cmd_int_clear));
                wait_for_ready = 1;
                state = ST_I2S_ON;
                U1TXREG = '4';
                break;

            case ST_I2S_ON:
                tuner_audio_run(tuner_id, 1);
                state = ST_DASR_S;
                break;

            case ST_DASR_S:
                tuner_write(tuner_id, cmd_dosr, sizeof(cmd_dosr));
                wait_for_ready = 1;
                state = ST_UP;
                appData.tuner_ready = 1;
                U1TXREG = '5';
                break;

            case ST_UP:
                if(!appData.suspended)
                    break;
            case ST_POWERING_DOWN:
                appData.tuner_ready = 0;
                tuner_write(tuner_id, cmd_power_down, sizeof(cmd_power_down));
                tuner_audio_run(tuner_id, 0);
                //TODO rclk stop
                state = ST_DOWN;
                U1TXREG = '6';
                break;





        }
    }

}

/*******************************************************************************
 End of File
 */
