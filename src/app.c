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
    appData.usbDevHandle = -1;
    appData.noData = 1;
    appData.reading = 0;
    appData.pingRequest = 0;

    appData.tunel_write_handle = USB_DEVICE_TRANSFER_HANDLE_INVALID;
    
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
            USB_DEVICE_EndpointDisable(appData.usbDevHandle, AUDIO_EP);
            USB_DEVICE_EndpointDisable(appData.usbDevHandle, TUNER_EP_IN);
            USB_DEVICE_EndpointDisable(appData.usbDevHandle, TUNER_EP_OUT);
            appData.state = APP_STATE_USB_OPENED;
            break;
        case USB_DEVICE_EVENT_CONFIGURED:
            U1TXREG = '+';
            /* check the configuration */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED *)eventData;
            if(configuredEventData->configurationValue == 1)
            {

            }
            size_t size = sizeof(AudioBufs[0].sample);
            USB_DEVICE_EndpointEnable(appData.usbDevHandle, 0, AUDIO_EP, USB_TRANSFER_TYPE_ISOCHRONOUS, size);
            USB_DEVICE_EndpointEnable(appData.usbDevHandle, 1, TUNER_EP_IN, USB_TRANSFER_TYPE_BULK, 32);
            USB_DEVICE_EndpointEnable(appData.usbDevHandle, 1, TUNER_EP_OUT, USB_TRANSFER_TYPE_BULK, 32);

            //appData.state = APP_STATE_CONFIGURED;
            break;
        case USB_DEVICE_EVENT_SUSPENDED:
            U1TXREG = 'S';
            appData.suspended = true;
            break;
        case USB_DEVICE_EVENT_RESUMED:
            U1TXREG = 's';
            appData.suspended = false;
            break;
        case USB_DEVICE_EVENT_POWER_DETECTED:
            U1TXREG = 'P';
            USB_DEVICE_Attach (appData.usbDevHandle);
            break;
        case USB_DEVICE_EVENT_POWER_REMOVED:
            U1TXREG = 'p';
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
                    appData.state == APP_STATE_CONFIGURED;
                    //
                    USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);
                    //USB_DEVICE_EndpointDisable(appData.usbDevHandle, 0x01|(1<<7));
                    U1TXREG = 'C';
                }
                else if(sp->wValue == 1)
                {
                    appData.state = APP_STATE_PLAY;
                    USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);
                    
                    appData.noData = 1;
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
            appData.tunel_read_count = ((USB_DEVICE_EVENT_DATA_ENDPOINT_WRITE_COMPLETE*)eventData)->length;
            if(appData.tunel_read_data[0]&TUNEL_PING_MASK) //FIXME / no jeslti toto pojede....
                USB_DEVICE_EndpointWrite(appData.usbDevHandle, &appData.tunel_write_handle, TUNER_EP_IN, &appData.tunel_read_data, appData.tunel_read_count, USB_DEVICE_TRANSFER_FLAGS_DATA_COMPLETE);

            break;
        }

        case USB_DEVICE_EVENT_ENDPOINT_WRITE_COMPLETE:
        {
            USB_DEVICE_EVENT_DATA_ENDPOINT_WRITE_COMPLETE* ed = (USB_DEVICE_EVENT_DATA_ENDPOINT_WRITE_COMPLETE*)eventData;

            if(appData.tunel_write_handle == ed->transferHandle)
            {
                appData.tunel_write_handle = USB_DEVICE_TRANSFER_HANDLE_INVALID;
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
                appData.noData = 1;
                U1TXREG = 'W';
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

        case APP_STATE_PLAY:
        {


            if(!appData.noData)
                break;
            static int b = 0;

            static void * data;
            data =  &(AudioBufs[b].sample);
            static size_t size ;
            size = sizeof(AudioBufs[b].sample);
            static USB_DEVICE_TRANSFER_HANDLE *handle;
            handle = &(AudioBufs[b].trasfer_handle);
            
            //USB_DEVICE_AUDIO_RESULT r ;//= USB_DEVICE_AUDIO_Write(0, handle, AUDIO_STREAMING_INTERFACE_ID, data, size);
            BDT = (U1BDTP1<<8) | 0xA0000000;//((BufferDT *)((USB_DEVICE_OBJ *)appData.usbDevHandle)->usbCDHandle);11
            BDT+=1;
            USB_DEVICE_RESULT r = USB_DEVICE_EndpointWrite(appData.usbDevHandle, handle, 0x01|(1<<7),data, size, USB_DEVICE_TRANSFER_FLAGS_DATA_COMPLETE);
            //debughalt();
            if(r == USB_DEVICE_RESULT_OK)
            {
                U1TXREG = 'w';
                U1TXREG = '0'+b;
                b++;
                if(b >= AUDIO_BUFS_COUNT) b=0;
                appData.noData = 0;
                
            }
            else if (r == USB_DEVICE_RESULT_ERROR_TRANSFER_QUEUE_FULL)
            {
                U1TXREG = 'f';
                appData.noData = 0;
            }
            else
            {
                U1TXREG = 'e';
            }
            break;
        }
            

        /* The default state should never be executed. */
        default:
        {
            break;
        }
    }
}


/*******************************************************************************
 End of File
 */
