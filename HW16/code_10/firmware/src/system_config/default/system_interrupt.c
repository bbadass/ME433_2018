/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "system/common/sys_common.h"
#include "app.h"
#include "system_definitions.h"



void __ISR(_TIMER_4_VECTOR, IPL4SOFT) Timer4ISR(void) {
  // code for PI control goes here
    float speed_1=(220/60*700/50)/2 ; //speed of wheel 1: half of maximum speed with 220 rpm max, 700 ticks per rotation, 50 Hz timer
    float speed_2=(220/60*700/50)/2 ; //speed of wheel 2: half of maximum speed with 220 rpm max, 700 ticks per rotation, 50 Hz timer
    float error1 = speed_1 - TMR5; //difference between desired speed and measured speed from encoder, wheel 1
    float error2 = speed_2 - TMR3; //difference between desired speed and measured speed from encoder, wheel 2
    float kp = 50; //weight of error in change of rotation speed
        
    //if there is a difference between the desired speed and the actual speed change the duty cycle 
    if (error1>0 && (error1*kp)<=2399 ){
        OC1RS = kp*error1;
    }
    if (error2>0 && (error2*kp)<=2399 ){
        OC4RS = kp*error2;
    }
    if ((error1*kp)>2399){
        OC1RS = 2399;
    }
    if ((error2*kp)>2399){
        OC4RS = 2399;
    }
    if (error1<0){
        OC1RS = 0;
    }
    if (error2<0){
        OC4RS = 0;
    }
    
    TMR3=0;
    TMR5=0;
    
  IFS0bits.T4IF = 0; // clear interrupt flag, last line
}
// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

 
/*******************************************************************************
 End of File
*/
