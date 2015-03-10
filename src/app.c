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

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    appData.usbDevHandle = -1;
    appData.noData = 1;
    
    int i,j;
    for(i=0; i<AUDIO_BUFS_COUNT; i++)
    {
        AudioBufs[i].isFree = 1;
        AudioBufs[i].trasfer_handle = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
        for(j=0; j<48; j++)
            AudioBufs[i].sample[j].l = AudioBufs[i].sample[j].r = 128;
    }

    tuner_init();
}

void APP_USBDeviceAudioEventHandler
(
    USB_DEVICE_AUDIO_INDEX iAudio ,
    USB_DEVICE_AUDIO_EVENT event ,
    void * pData,
    uintptr_t context
);

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
            appData.state = APP_STATE_USB_OPENED;
            break;
        case USB_DEVICE_EVENT_CONFIGURED:
            U1TXREG = 'C';
            /* check the configuration */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED *)eventData;
            if(configuredEventData->configurationValue == 1)
            {
                /* the device is in configured state */
                USB_DEVICE_AUDIO_EventHandlerSet
                (
                    0,
                    APP_USBDeviceAudioEventHandler ,
                    (uintptr_t)NULL
                );
            }
            appData.state = APP_STATE_CONFIGURED;
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
        default:
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

        case APP_STATE_PLAY:
        {

            if(!appData.noData)
                break;
            static int b = 0;

            static void * data;
            data =  &(AudioBufs[b].sample);
            static size_t size ;
            size = sizeof(AudioBufs[b].sample);
            static USB_DEVICE_AUDIO_TRANSFER_HANDLE *handle;
            handle = &(AudioBufs[b].trasfer_handle);
            
            USB_DEVICE_AUDIO_RESULT r = USB_DEVICE_AUDIO_Write(0, handle, AUDIO_STREAMING_INTERFACE_ID, data, size>1);
            if(!r)
            {
                U1TXREG = 'w';
                U1TXREG = '0'+b;
                b++;
                if(b >= AUDIO_BUFS_COUNT) b=0;
                appData.noData = 0;
                
            }
            else if (r == USB_DEVICE_AUDIO_RESULT_ERROR_TRANSFER_QUEUE_FULL)
                U1TXREG = 'f';
            else
            {
                U1TXREG = 'e';
                r = USB_DEVICE_AUDIO_Write(0, &(AudioBufs[b].trasfer_handle),AUDIO_STREAMING_INTERFACE_ID , &(AudioBufs[b].sample), sizeof(AudioBufs[b].sample));

            }
        }
            break;

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}


void APP_USBDeviceAudioEventHandler
(
    USB_DEVICE_AUDIO_INDEX iAudio ,
    USB_DEVICE_AUDIO_EVENT event ,
    void * pData,
    uintptr_t context
)
{
    USB_DEVICE_AUDIO_EVENT_DATA_INTERFACE_SETTING_CHANGED *interfaceInfo;
    USB_DEVICE_AUDIO_EVENT_DATA_WRITE_COMPLETE *eventData;
    uint8_t entityID;
    uint8_t controlSelector;
    if ( iAudio == 0 )
    {
        switch (event)
        {
            case USB_DEVICE_AUDIO_EVENT_INTERFACE_SETTING_CHANGED:
                /* We have received a request from USB host to change the Interface-
                   Alternate setting.*/
            
                interfaceInfo = (USB_DEVICE_AUDIO_EVENT_DATA_INTERFACE_SETTING_CHANGED *)pData;
                if(interfaceInfo->interfaceAlternateSetting == 1)
                {
                    appData.state = APP_STATE_PLAY;
                    //SYS_DEBUG_Message(" Play ");
                    U1TXREG = 'I';
                }
                else
                {
                    appData.state = APP_STATE_CONFIGURED;
                    //SYS_DEBUG_Message(" Stop ");
                    /*int i;
                    for(i=0; i<AUDIO_BUFS_COUNT;i++)
                    {
                        if(AudioBufs[i].trasfer_handle!=USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID)
                            USB_DEVICE_AUDIO_TransferCancel(0, AudioBufs[i].trasfer_handle);
                    }
                    U1TXREG = 'i';*/
                }
                //appData.state = APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD;
                
            break;

            case USB_DEVICE_AUDIO_EVENT_READ_COMPLETE:
            /*
                readEventData = (USB_DEVICE_AUDIO_EVENT_DATA_READ_COMPLETE *)pData;
                //We have received an audio frame from the Host.
                //Now send this audio frame to Audio Codec for Playback.
                if (readEventData->handle == appData.readTransferHandle)
                {
                    if (appData.state == APP_IDLE)
                        appData.state = APP_SEND_DATA_TO_CODEC;
                    else
                        appData.state = APP_STATE_ERROR;
                }
            */
                U1TXREG = '<';
            break;

            case USB_DEVICE_AUDIO_EVENT_WRITE_COMPLETE:
            {
                U1TXREG = 'W';
                eventData = (USB_DEVICE_AUDIO_EVENT_DATA_WRITE_COMPLETE *)pData;
                Nop();
                int i;
                for(i=0; i<AUDIO_BUFS_COUNT;i++)
                {
                    if(eventData->handle == AudioBufs[i].trasfer_handle)
                    {
                        AudioBufs[i].isFree = 1;
                        AudioBufs[i].trasfer_handle = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
                        U1TXREG = '0'+i;
                        i=AUDIO_BUFS_COUNT;
                        appData.noData = 1;
                    }
                }
            }
            break;

            case USB_DEVICE_AUDIO_EVENT_CONTROL_SET_CUR:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_GET_CUR:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_SET_MIN:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_GET_MIN:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_SET_MAX:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_GET_MAX:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_SET_RES:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_GET_RES:
            case USB_DEVICE_AUDIO_EVENT_ENTITY_GET_MEM:
                /* Stall request */
                USB_DEVICE_ControlStatus (appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            break;
            case USB_DEVICE_AUDIO_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:
                /*
                USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK );
                if (appData.currentAudioControl == APP_USB_AUDIO_MUTE_CONTROL)
                {
                    appData.state = APP_MUTE_AUDIO_PLAYBACK;
                    appData.currentAudioControl = APP_USB_CONTROL_NONE;
                    //Handle Mute Control Here.
                }*/
                U1TXREG = '=';
                /* Stall request */
                USB_DEVICE_ControlStatus (appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            break;
            case  USB_DEVICE_AUDIO_EVENT_CONTROL_TRANSFER_DATA_SENT:
                //no action required
            break;
            default:
                SYS_ASSERT ( false , "Invalid callback" );
            break;
        } //end of switch ( callback )
    }//end of if  if ( iAudio == 0 )
}//end of function APP_AudioEventCallback

/*******************************************************************************
 End of File
 */
