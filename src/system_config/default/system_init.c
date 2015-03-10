/*******************************************************************************
  System Initialization File

  File Name:
    system_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, configuration bits, and allocates
    any necessary global system resources, such as the systemObjects structure
    that contains the object handles to all the MPLAB Harmony module objects in
    the system.
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

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

#include "system_config.h"
#include "system_definitions.h"
#include "app.h"
#include "usb_desc_structs.h"


// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************

/*** DEVCFG0 ***/

#pragma config DEBUG =      ON
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx1
#pragma config PWP =        0x3f
#pragma config BWP =        OFF
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      PRIPLL
#pragma config FSOSCEN =    OFF
#pragma config IESO =       OFF
#pragma config POSCMOD =    HS
#pragma config OSCIOFNC =   OFF
#pragma config FPBDIV =     DIV_1
#pragma config FCKSM =      CSDCMD
#pragma config WDTPS =      PS1048576
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     OFF
#pragma config FWDTWINSZ =  WINSZ_25

/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_2
#pragma config FPLLMUL =    MUL_20
#pragma config FPLLODIV =   DIV_2
#pragma config UPLLIDIV =   DIV_2
#pragma config UPLLEN =     ON

/*** DEVCFG3 ***/

#pragma config USERID =     0xcdef
#pragma config PMDL1WAY =   OFF
#pragma config IOL1WAY =    OFF
#pragma config FUSBIDIO =   OFF
#pragma config FVBUSONIO =  ON


// *****************************************************************************
// *****************************************************************************
// Section: Library/Stack Initialization Data
// *****************************************************************************
// *****************************************************************************/

//<editor-fold defaultstate="collapsed" desc="USB Stack Configuration">

/**************************************************
 * USB Device Function Driver Init Data
 **************************************************/
    const USB_DEVICE_AUDIO_INIT audioInit0 =
    {
        .queueSizeRead = 0,
        .queueSizeWrite = 2
    };
/**************************************************
 * USB Device Layer Function Driver Registration 
 * Table
 **************************************************/
const USB_DEVICE_FUNCTION_REGISTRATION_TABLE funcRegistrationTable[1] =
{
    /* Function 1 */
    { 
        .configurationValue = 1,    /* Configuration value */ 
        .interfaceNumber = 0,       /* First interfaceNumber of this function */
        .numberOfInterfaces = 2,    /* Number of interfaces */
        .speed = USB_SPEED_FULL,    /* Function Speed */ 
        .funcDriverIndex = 0,  /* Index of Audio Function Driver */
        .driver = (void*)USB_DEVICE_AUDIO_FUNCTION_DRIVER,   /* USB Audio function data exposed to device layer */
        .funcDriverInit = (void*)&audioInit0,   /* Function driver init data*/
    },
};

/*******************************************
 * USB Device Layer Descriptors
 *******************************************/


const USB_dev_decriptor dev_desc =
{
    .bLength            = sizeof(USB_dev_decriptor),
    .bDescriptorType    = 0x01,
    .bcdUSB             = 0x0200, //2.0
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x0a06, //FIXME
    .idProduct          = 0xff02, //FIXME
    .bcdDevice          = 0x0001,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 0,
    .bNumConfigurations = 1
};

/*
const USB_configuration_decriptor conf_desc =
{
    .bLength = sizeof(USB_configuration_decriptor),
    .bDescriptorType = 0x02,
    .wTotalLength = sizeof(conf_desc),
    .bNumInterfaces = 0x00,
    .bConfigurationValue = 1, //Value used by Set Configuration to select this configuration
    .iConfiguration = 0,        //string
    .bmAttributes = 0b10000000,
    .bMaxPower = 50
};
*/
const uint8_t conf_desc[] =
{
    /* USB Microphone Configuration Descriptor */
    0x09,//sizeof(USB_CFG_DSC),     // Size of this descriptor in bytes
    USB_DESCRIPTOR_CONFIGURATION,   // CONFIGURATION descriptor type
    0x64,0x00,                      // Total length of data for this cfg
    2,                              // Number of interfaces in this cfg
    1,                              // Index value of this configuration
    0,                              // Configuration string index
    USB_ATTRIBUTE_DEFAULT,          // Attributes, see usb_device.h
    50,                             // Max power consumption (2X mA) //FIXME:

    /* USB Microphone Standard AC Interface Descriptor	*/
    0x09,//sizeof(USB_INTF_DSC),    // Size of this descriptor in bytes
    USB_DESCRIPTOR_INTERFACE,       // INTERFACE descriptor type
    AUDIO_CONTROL_INTERFACE_ID,     // Interface Number
    0x00,                           // Alternate Setting Number
    0x00,                           // Number of endpoints in this intf
    USB_AUDIO_CLASS_CODE,           // Class code
    USB_AUDIO_AUDIOCONTROL,         // Subclass code
    0x00,                           // Protocol code
    0x00,                           // Interface string index

    /* USB Microphone Class-specific AC Interface Descriptor  */
    0x09,                           // Size of this descriptor, in bytes.
    USB_AUDIO_CS_INTERFACE,         // CS_INTERFACE Descriptor Type
    USB_AUDIO_HEADER,               // HEADER descriptor subtype
    0x00,0x01,                      // Audio Device compliant to the USB Audio specification version 1.00
    0x1E,0x00,                      // Total number of bytes returned for the class-specific AudioControl interface descriptor.
                                    // Includes the combined length of this descriptor header and all Unit and Terminal descriptors.
    0x01,                           // The number of AudioStreaming interfaces in the Audio Interface Collection to which this AudioControl interface belongs
    0x01,                           // AudioStreaming interface 1 belongs to this AudioControl interface.

    /*USB Microphone Input Terminal Descriptor */ 
    0x0C,                           // Size of the descriptor, in bytes
    USB_AUDIO_CS_INTERFACE,         // CS_INTERFACE Descriptor Type
    USB_AUDIO_INPUT_TERMINAL,       // INPUT_TERMINAL descriptor subtype
    ID_INPUT_TERMINAL,              // ID of this Terminal.
    0x10,0x07,                      //Terminal is Fm Rx
    0x00,                           // No association
    0x02,//0x01,                    //two chans
    0x03,0x00,//0x00,0x00,          // L+R cannels
    0x00,                           // Unused.
    0x00,                           // Unused.

    /* USB Microphone Output Terminal Descriptor */
    0x09,                           // Size of the descriptor, in bytes (bLength)
    USB_AUDIO_CS_INTERFACE,         // CS_INTERFACE Descriptor Type (bDescriptorType)
    USB_AUDIO_OUTPUT_TERMINAL,      // OUTPUT_TERMINAL descriptor subtype (bDescriptorSubtype)
    ID_OUTPUT_TERMINAL,             // ID of this Terminal. (bTerminalID)
    0x01,0x03,                      // USB Streaming. (wTerminalType
    0x00,                           // unused (bAssocTerminal)
    ID_INPUT_TERMINAL,              // From Input Terminal.(bSourceID)
    0x00,                           // unused  (iTerminal)

    /* USB Microphone Standard AS Interface Descriptor (Alt. Set. 0) */
    0x09,                           // Size of the descriptor, in bytes (bLength)
    USB_DESCRIPTOR_INTERFACE,       // INTERFACE descriptor type (bDescriptorType)
    AUDIO_STREAMING_INTERFACE_ID,   // Index of this interface. (bInterfaceNumber)
    0x00,                           // Index of this alternate setting. (bAlternateSetting)
    0x00,                           // 0 endpoints. (bNumEndpoints)
    USB_AUDIO_CLASS_CODE,           // AUDIO (bInterfaceClass)
    USB_AUDIO_AUDIOSTREAMING,       // AUDIO_STREAMING (bInterfaceSubclass)
    0x00,                           // Unused. (bInterfaceProtocol)
    0x00,                           // Unused. (iInterface)

    /* USB Microphone Standard AS Interface Descriptor (Alt. Set. 1) */
    0x09,                           // Size of the descriptor, in bytes (bLength)
    USB_DESCRIPTOR_INTERFACE,       // INTERFACE descriptor type (bDescriptorType)
    AUDIO_STREAMING_INTERFACE_ID,   // Index of this interface. (bInterfaceNumber)
    0x01,                           // Index of this alternate setting. (bAlternateSetting)
    0x01,                           // 1 endpoint (bNumEndpoints)
    USB_AUDIO_CLASS_CODE,           // AUDIO (bInterfaceClass)
    USB_AUDIO_AUDIOSTREAMING,       // AUDIO_STREAMING (bInterfaceSubclass)
    0x00,                           // Unused. (bInterfaceProtocol)
    0x00,                           // Unused. (iInterface)

    /*  USB Microphone Class-specific AS General Interface Descriptor */
    0x07,                           // Size of the descriptor, in bytes (bLength)
    USB_AUDIO_CS_INTERFACE,         // CS_INTERFACE Descriptor Type (bDescriptorType)
    USB_AUDIO_AS_GENERAL,           // GENERAL subtype (bDescriptorSubtype)
    ID_OUTPUT_TERMINAL,             // Unit ID of the Output Terminal.(bTerminalLink)
    0x01,                           // Interface delay. (bDelay)
    0x02, 0x00,                     // PCM8 (formats str29)

    /*  USB Microphone Type I Format Type Descriptor */
    0x0B,                           // Size of the descriptor, in bytes (bLength)
    USB_AUDIO_CS_INTERFACE,         // CS_INTERFACE Descriptor Type (bDescriptorType)
    USB_AUDIO_FORMAT_TYPE,          // FORMAT_TYPE subtype. (bDescriptorSubtype)
    0x01,                           // FORMAT_TYPE_I. (bFormatType)
    0x02,                           // two ch (bNrChannels)
    0x01,                           // One byte per audio subframe.(bSubFrameSize)
    0x08,                           // 8 bits per sample.(bBitResolution)
    0x01,                           // One frequency supported. (bSamFreqType)
    0x80,0xBB,0x00,                 // 48khz = BB80 (tSamFreq)

    /*  USB Microphone Standard Endpoint Descriptor */
    0x09,                           // Size of the descriptor, in bytes (bLength)
    0x05,                           // ENDPOINT descriptor (bDescriptorType)
    0x01 | (1<<7),                  // Endpoint number. (bEndpointAddress) | direction
    0x01,                           // Isochronous, not shared. (bmAttributes)
    0x60,0x00,                      // 48*2=96=0x0060 bytes per packet (wMaxPacketSize)
    0x01,                           // One packet per frame.(bInterval)
    0x00,                           // Unused. (bRefresh)
    0x00,                           // Unused. (bSynchAddress)

    /* USB Microphone Class-specific Isoc. Audio Data Endpoint Descriptor*/
    0x07,                           // Size of the descriptor, in bytes (bLength)
    USB_AUDIO_CS_ENDPOINT,          // CS_ENDPOINT Descriptor Type (bDescriptorType)
    USB_AUDIO_AS_GENERAL,           // GENERAL subtype. (bDescriptorSubtype)
    0x00,                           // No sampling frequency control, no pitch control, no packet padding.(bmAttributes)
    0x00,                           // Unused. (bLockDelayUnits)
    0x00,0x00                       // Unused. (wLockDelay)
};

struct Str_zero
{
    USB_string_descriptor_hdr hdr;
    unsigned short langid;
};

static struct Str_zero str_zero = {
    .hdr.bDescriptorType = 0x03,
    .hdr.bLength = sizeof(str_zero),
    .langid = 0x0409
};

/*******************************************
 *  Language code string descriptor
 *******************************************/
const struct
{
    uint8_t bLength;
    uint8_t bDscType;
    uint16_t string[1];
}sd000=
{
    sizeof(sd000),
    USB_DESCRIPTOR_STRING,
    {0x0409}
};

/*******************************************
 *  Manufacturer string descriptor
 *******************************************/
const struct
{
    uint8_t bLength;
    uint8_t bDscType;
    uint16_t string[15];
}sd001=
{
    sizeof(sd001),
    USB_DESCRIPTOR_STRING,
    {
        'V','S','B','-','T','U','O',' ','D','i','p','l','o','m','a'}
};

/*******************************************
 *  Product string descriptor
 *******************************************/
const struct
{
    uint8_t bLength;
    uint8_t bDscType;
    uint16_t string[13];
}sd002=
{
    sizeof(sd002),
    USB_DESCRIPTOR_STRING,
    {
        'D','u','a','l',' ','F','M',' ','T','u','n','e','r'
    }
};

/***************************************
 * Array of configuration descriptors
 ***************************************/
/*const uint8_t *const usbConfigurationDescriptors[]=
{
    conf_desc
};*/

/***************************************
 * Array of string descriptors
 ***************************************/
USB_DEVICE_STRING_DESCRIPTORS_TABLE usbStringDescriptors[3]=
{
    (const uint8_t *const)&sd000,
    (const uint8_t *const)&sd001,
    (const uint8_t *const)&sd002
};

/*******************************************
 * Array of full speed config descriptors
 *******************************************/

USB_DEVICE_CONFIGURATION_DESCRIPTORS_TABLE fullSpeedConfigDescSet[1] =
{
    (const uint8_t*) &conf_desc,
};


/*******************************************
 * USB Device Layer Master Descriptor Table 
 *******************************************/
const USB_DEVICE_MASTER_DESCRIPTOR usbMasterDescriptor =
{
    /* Pointer to standard device descriptor (for low/full speed) */
    .deviceDescriptor = (USB_DEVICE_DESCRIPTOR *) (&dev_desc),


    /* Total number configurations available (for low/full speed)*/
    .configDescriptorCount = 1,

    /* Pointer to array of configurations descriptor pointers
       (for low/full speed)*/
    .configDescriptorTable =  &fullSpeedConfigDescSet[0],

    /* Pointer to array of high speed standard Device descriptor.
       Assign this to NULL if not supported.*/
    .highSpeedDeviceDescriptor = NULL,

    /* Total number of high speed configurations available.
       Set this to zero if not supported*/
    .highSpeedConfigDescriptorCount = 0,

    /* Pointer to array of high speed configurations descriptor pointers.
       Set this to NULL if not supported*/
    .highSpeedConfigDescriptorTable = NULL,

    /* Total number of string descriptors available (common to all speeds)*/
    .stringDescCount = 3,

    /* Pointer to array of string Descriptor pointers (common to all speeds)*/
    .stringDescriptorTable = usbStringDescriptors,

    /* Pointer to full speed device_qualifier descriptor. Device responds
       with this descriptor when it is operating at high speed */
    .fullSpeedDeviceQualifier = NULL,

    /* Pointer to high speed device_qualifier descriptor.
       Device responds with this descriptor when it is
       operating at full speed */
    .highSpeedDeviceQualifier = NULL,

    /* Pointer to BOS descriptor for this Device. Device responds
       with this descriptor when Host sends a GET_DESCRIPTOR request for BOS
       descriptor */
    .bosDescriptor = NULL,
};

/****************************************************
 * Endpoint Table needed by the Device Layer.
 ****************************************************/
uint8_t __attribute__((aligned(512))) endPointTable[USB_DEVICE_ENDPOINT_TABLE_SIZE];

/****************************************************
 * USB Device Layer Initialization Data
 ****************************************************/

const USB_DEVICE_INIT usbDevInitData =
{
    /* System module initialization */
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},

	/* Identifies peripheral (PLIB-level) ID */

    /* Stop in idle */
    .stopInIdle = false,

    /* Suspend in sleep */
    .suspendInSleep = false,
    /* Endpoint table */
    .endpointTable= endPointTable,

    /* Number of function drivers registered to this instance of the
       USB device layer */
    .registeredFuncCount = 1,

    /* Function driver table registered to this instance of the USB device layer*/
    .registeredFunctions = (USB_DEVICE_FUNCTION_REGISTRATION_TABLE*)funcRegistrationTable,

    /* Pointer to USB Descriptor structure */
    .usbMasterDescriptor = (USB_DEVICE_MASTER_DESCRIPTOR*)&usbMasterDescriptor,

    /* USB Device Speed */
    .deviceSpeed = USB_SPEED_FULL,


};

// </editor-fold>


// *****************************************************************************
// *****************************************************************************
// Section: Driver Initialization Data
// *****************************************************************************
// *****************************************************************************

//<editor-fold defaultstate="collapsed" desc="DRV_USART Configuration">

// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: System Data
// *****************************************************************************
// *****************************************************************************

/* Structure to hold the object handles for the modules in the system. */
SYSTEM_OBJECTS sysObj;

// *****************************************************************************
// *****************************************************************************
// Section: Module Initialization Data
// *****************************************************************************
// *****************************************************************************


//<editor-fold defaultstate="collapsed" desc="SYS_DEVCON Configuration">

/*** System Device Control Initialization Data ***/

const SYS_DEVCON_INIT sysDevconInit =
{
    .moduleInit = {0},
};
// </editor-fold>
//<editor-fold defaultstate="collapsed" desc="SYS_DEBUG Configuration">
/*** System Debug Initialization Data ***/

SYS_DEBUG_INIT debugInit =
{
    .moduleInit = {0},
    .errorLevel = SYS_ERROR_DEBUG
};
// </editor-fold>
//<editor-fold defaultstate="collapsed" desc="SYS_CONSOLE Configuration">
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Static Initialization Functions
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: System Initialization
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Initialize ( SYS_INIT_DATA *data )

  Summary:
    Initializes the board, services, drivers, application and other modules.

  Remarks:
    See prototype in system/common/sys_module.h.
 */

void SYS_Initialize ( void* data )
{
    /* Core Processor Initialization */
    SYS_CLK_Initialize( NULL );
    sysObj.sysDevcon = SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)&sysDevconInit);
    SYS_DEVCON_PerformanceConfig(SYS_CLK_SystemFrequencyGet());
    SYS_PORTS_Initialize();

    /* Initialize Drivers */
    DRV_USART0_Initialize();

    /* Initialize System Services */
    SYS_INT_Initialize();  
    sysObj.sysConsole0 = SYS_CONSOLE_Initialize(SYS_CONSOLE_INDEX_0, NULL);

    sysObj.sysDebug = SYS_DEBUG_Initialize(SYS_DEBUG_INDEX_0, (SYS_MODULE_INIT*)&debugInit);

    /* Initialize Middleware */

    /* Initialize the USB device layer */
    sysObj.usbDevObject0 = USB_DEVICE_Initialize (USB_DEVICE_INDEX_0 , ( SYS_MODULE_INIT* ) & usbDevInitData);
    /* Enable Global Interrupts */
    SYS_INT_Enable();

    /* Initialize the Application */
    APP_Initialize();

}

/*******************************************************************************
 End of File
*/

