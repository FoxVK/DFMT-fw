/*******************************************************************************
  MPLAB Harmony Project Main Source File

  Company:
    Microchip Technology Inc.
  
  File Name:
    main.c

  Summary:
    This file contains the "main" function for an MPLAB Harmony project.

  Description:
    This file contains the "main" function for an MPLAB Harmony project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state 
    machines of all MPLAB Harmony modules in the system and it calls the 
    "SYS_Tasks" function from within a system-wide "super" loop to maintain 
    their correct operation. These two functions are implemented in 
    configuration-specific files (usually "system_init.c" and "system_tasks.c")
    in a configuration-specific folder under the "src/system_config" folder 
    within this project's top-level folder.  An MPLAB Harmony project may have
    more than one configuration, each contained within it's own folder under
    the "system_config" folder.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

//Microchip licenses to you the right to use, modify, copy and distribute
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

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes

#include "xc.h"
#include "framework/driver/usart/drv_usart_static.h"
#include "system/debug/sys_debug.h"


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

void _general_exception_handler (unsigned cause, unsigned status)
{
    static unsigned int _excep_addr = 0;
    asm volatile("mfc0 %0,$14" : "=r" (_excep_addr));

    char str[30];
    sprintf(str, "\r\n Exception @0x%x: ", _excep_addr);
    SYS_DEBUG_Message(str);

    switch((cause & 0x7c)>>2)
    {
        case 0 : SYS_DEBUG_Message("interrupt"); break;
        case 4 : SYS_DEBUG_Message("address error exception (load or ifetch)"); break;
        case 5 : SYS_DEBUG_Message("address error exception (store)"); break;
        case 6 : SYS_DEBUG_Message("bus error (ifetch)"); break;
        case 7 : SYS_DEBUG_Message("bus error (load/store)"); break;
        case 8 : SYS_DEBUG_Message("syscall"); break;
        case 9 : SYS_DEBUG_Message("breakpoint"); break;
        case 10: SYS_DEBUG_Message("reserved instruction"); break;
        case 11: SYS_DEBUG_Message("coprocessor unusable"); break;
        case 12: SYS_DEBUG_Message("arithmetic overflow"); break;
        case 13: SYS_DEBUG_Message("trap (possible divide by zero)"); break;
        case 16: SYS_DEBUG_Message("implementation specfic 1"); break;
        case 17: SYS_DEBUG_Message("CorExtend Unuseable"); break;
        case 18: SYS_DEBUG_Message("coprocessor 2"); break;
        default: SYS_DEBUG_Message("Unknown reason"); break;
    }

    _nop();
    while(true);
}

int main ( void )
{
    RCON = 0;
    /* Initialize all MPLAB Harmony modules, including application(s). */
    SYS_Initialize ( NULL );

    //We do not need any analog pin
    ANSELA = 0;
    ANSELB = 0;

    LATBbits.LATB2 = 0;
    TRISBbits.TRISB2 = 0;

    TRISBbits.TRISB5 = 1;   //SDI1
    TRISBbits.TRISB3 = 0;   //SS 1
    TRISBbits.TRISB14 = 0;  //SCK1

    TRISAbits.TRISA4 = 1;   //SDI2
    TRISBbits.TRISB0 = 0;   //SS 2
    TRISBbits.TRISB15 = 0;  //SCK2

    SYSKEY = 0x33333333; //write invalid key to force lock
    SYSKEY = 0xAA996655; //write key1 to SYSKEY
    SYSKEY = 0x556699AA; //write key2 to SYSKEY

    SDI1Rbits.SDI1R = 0b0001;    //SDI1 on RB5
    RPB3Rbits.RPB3R = 0b0011;    //SS 1 on RB3

    SDI2Rbits.SDI2R = 0b0010;    //SDI2 on RA4
    RPB0Rbits.RPB0R = 0b0100;    //SS 2 on RB0

    SYSKEY = 0x33333333; //write invalid key to force lock


    /*TRISAbits.TRISA0 = 0;

    SYSKEY = 0x33333333; //write invalid key to force lock
    SYSKEY = 0xAA996655; //write key1 to SYSKEY
    SYSKEY = 0x556699AA; //write key2 to SYSKEY
    CFGCONbits.IOLOCK = 0;

    RPA0R = 1; //uart tx on RA0;
    U1MODEbits.ON = 1;
    U1BRG = 21; //115 200
    U1MODEbits.RTSMD = 1;

    U1STAbits.UTXEN = 1;

    //U1STAbits.UTXBF //je volno
    U1TXREG = '\r';
    U1TXREG = '\n';
    U1TXREG = '!';*/

    SYS_DEBUG_Message("\r\n");
    int i,j;
    for(i=0; i<300000;i++)
        for(j=0; j<80;j++)
            Nop();
    SYS_DEBUG_Message("!");
    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );

    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

