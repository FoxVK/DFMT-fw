#define debughalt() __asm__ volatile (" sdbbp 0")

#define PACKET_SIZE (48*2*2)
//#define NO_USB_ATTACH
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



APP_DATA appData;


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
            appData.state = APP_STATE_USB_OPENED;
            appData.reading = 0;
            appData.audio_play = false;

            USB_DEVICE_EndpointDisable(appData.usbDevHandle, AUDIO_EP);

            if(appData.tunnel_write_handle != USB_DEVICE_TRANSFER_HANDLE_INVALID)
                USB_DEVICE_EndpointTransferCancel(appData.usbDevHandle, TUNNEL_EP_IN, appData.tunnel_write_handle);
            USB_DEVICE_EndpointDisable(appData.usbDevHandle, TUNNEL_EP_IN);
            USB_DEVICE_EndpointDisable(appData.usbDevHandle, TUNNEL_EP_OUT);
            break;
        }
        case USB_DEVICE_EVENT_CONFIGURED:
            U1TXREG = '+';
            /* check the configuration */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED *)eventData;
            if(configuredEventData->configurationValue == 1)
            {

            }
            USB_DEVICE_EndpointEnable(appData.usbDevHandle, AUDIO_STREAMING_INTERFACE_ID, AUDIO_EP, USB_TRANSFER_TYPE_ISOCHRONOUS, PACKET_SIZE);
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
            #ifndef NO_USB_ATTACH
                USB_DEVICE_Attach (appData.usbDevHandle);
            #else
                #warning "USB ATTACH DISABLED"
            #endif
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
            //debughalt();
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
                appData.noAudioData = 1;
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

        int16_t* data = tuner_audio_get(0);
        USB_DEVICE_TRANSFER_HANDLE handle;


        USB_DEVICE_RESULT r = USB_DEVICE_EndpointWrite(
                appData.usbDevHandle,
                &handle,
                AUDIO_EP,
                data,
                PACKET_SIZE,
                USB_DEVICE_TRANSFER_FLAGS_DATA_COMPLETE);

        appData.noAudioData = 0;
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
}


void app_tuner_updown_tasks()
{
    
    static uint8_t cmd_power_up[]   = {0x01, 0x00, 0xB5};
    static uint8_t cmd_power_down[] = {0x11};
    static uint8_t cmd_tune[]       = {0x20,0x01,0x24,0x9A,0x00}; //0x01 = inaccurate but fast tunning alowed to 93.7
    static uint8_t cmd_int_update[] = {0x14};
    static uint8_t cmd_int_clear[]  = {0x22, 0x01};
    static uint8_t cmd_dosr[]       = {0x12, 0x00, 0x01, 0x04, 0xBB, 0x80}; //sample rate
    static uint8_t cmd_refclk_presc[]= {0x12, 0x00, 0x02, 0x02, 0x10, 0x2F}; //1538461,538461538 / 47 -> 32733,2Hz bit 12 = 1 (colck from dclk) 0x102f
    static uint8_t cmd_refclk_freq[]= {0x12, 0x00, 0x02, 0x01, 0x7F, 0xDD}; //32733 0x7fdd

    static uint8_t * cmd_list[] = {
        NULL,
        cmd_power_up,
        cmd_refclk_freq, cmd_refclk_presc,
        cmd_tune, cmd_int_update, cmd_int_clear,
        cmd_dosr,
        NULL
    };

    static size_t cmd_sizes[] =
    {
        0,
        sizeof(cmd_power_up),
        sizeof(cmd_refclk_freq) ,sizeof(cmd_refclk_presc),
        sizeof(cmd_tune), sizeof(cmd_int_update), sizeof(cmd_int_clear),
        sizeof(cmd_dosr),
        0,

    };

    static enum {
        ST_DOWN = 0,
        ST_PWRUP,
        ST_RCLK_FREQ, ST_RCLK_PRESC,
        ST_TUNE, ST_TUNE_INT_UPDATE, ST_TUNE_INT_CLR,
        ST_DOSR,
        ST_UP,
    } state[] = {ST_DOWN, ST_DOWN};

    static enum {
        CST_DONE, CST_CHCK_REPLY, CST_READING
    } com_st[2] = {CST_DONE, CST_DONE};

    static Tuner_read_reply reply[2];
    static int tid = 1;     //ID of currently porcessed tunner

   
           
    {

        Tuner_com_state tun_s = tuner_com_state();

        static bool bus = false;
        if(tun_s == TUNER_COM_BUSY)
        {
            if(!bus)
                U1TXREG = 'B';
            bus = true;
        }
        else
        {
            if(bus)
                U1TXREG = 'b';
            bus = false;
        }


        if( tun_s == TUNER_COM_BUSY)
            return;

        tid = (tid+1)&1;

        Nop();

        switch(com_st[tid])
        {
            case CST_DONE:
                break;

            case CST_CHCK_REPLY:
                tuner_read(tid, &reply[tid], 1);
                com_st[tid] = CST_READING;
                return;

            case CST_READING:
                if(reply[tid].STATUS.bits.CTS)
                    com_st[tid] = CST_DONE;
                else
                {
                    com_st[tid] = CST_CHCK_REPLY;
                }
                return;
        }

        switch(state[tid])
        {
            case ST_UP:
                if(appData.suspended)
                {
                    tuner_hold_in_rst(1);
                    tuner_audio_run(tid, 0);
                    state[tid] = ST_DOWN;
                }
                break;

            case ST_DOWN:
                if(!appData.suspended)
                {
                    tuner_audio_run(tid, 1);
                    tuner_hold_in_rst(0);
                    state[tid]++;
                }
                break;

            case ST_TUNE_INT_CLR:
                if(!reply[tid].STATUS.bits.STCINT)
                {
                    state[tid] = ST_TUNE_INT_UPDATE;
                    break;
                }
                //do not put break here or another state

            default:
                if(cmd_list[state[tid]]==NULL)
                    debughalt();
                tuner_write(tid, cmd_list[state[tid]], cmd_sizes[state[tid]]);
                com_st[tid] = CST_CHCK_REPLY;
                state[tid]++;
                break;

        }

    }

}

/*******************************************************************************
 End of File
 */
