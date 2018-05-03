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
#include <ST7735.h> //lcd library
#include<stdio.h> //sprintf
#include<i2c_master_noint.h>
#include <xc.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

uint8_t APP_MAKE_BUFFER_DMA_READY dataOut[APP_READ_BUFFER_SIZE];
uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
int len, i = 0;
int startTime = 0;

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

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void writeCharacter(unsigned short x, unsigned short y, char letter, unsigned short color_on, unsigned short color_off) {

    unsigned short xi = 0;
    unsigned short yi = 0;

    for (xi = 0; xi < 5; xi++) { //loop over the five pixels (in x) per character

        unsigned char pixels = ASCII[letter - 0x20][xi];

        for (yi = 0; yi < 8; yi++) { //loop over bits of each character (8 pixels in y)
            if ((pixels >> yi) & 1) { // if the yith bit of the ascii (shift back by yi) is on, turn the pixel on
                LCD_drawPixel(x + xi, y + yi, color_on);
            } else { // if the yith bit of the ascii (shift back by yi) is off, color the pixel of the off color
                LCD_drawPixel(x + xi, y + yi, color_off);
            }
        }
    }

}

//function to write messages composed of multiple characters on the LCD

void writeString(unsigned short x, unsigned short y, unsigned char* words, unsigned short color_on, unsigned short color_off) {

    unsigned int i = 0;

    while (words[i]) { //run until there is a character, which works because sprintf has a null zero as its last character 
        writeCharacter(x + 5 * i, y, words[i], color_on, color_off); //shift by 5 pixels per letter
        i++;
    }

}


//function to draw bars on the LCD

void drawBarO(unsigned short x, unsigned short y, signed short n1o, unsigned short color_on, unsigned short color_off, unsigned int width, unsigned int length, signed short n1max) {

    unsigned short ni1 = 0;
    unsigned short ni2 = 0;
    unsigned short yi = 0;
    signed short length_max_v = length / 2;
    signed short n1 = n1o / n1max;

    for (yi = 0; yi <= width; yi++) { //loop over length of the bar

        if (n1 > 0) {
            for (ni2 = 0; ni2 <= length_max_v; ni2++) { //loop over n2 values of n (color off of the bar background)
                LCD_drawPixel(x - ni2, y + yi, color_off);
            }
            if (n1 <= length_max_v) {
                for (ni1 = 0; ni1 <= n1; ni1++) { //loop over n1 values of n (color on of the bar)
                    LCD_drawPixel(x + ni1, y + yi, color_on);
                }
                for (ni2 = n1; ni2 <= length_max_v; ni2++) { //loop over n2 values of n (color off of the bar background)
                    LCD_drawPixel(x + ni2, y + yi, color_off);
                }
            }
            else {
                for (ni1 = 0; ni1 <= length_max_v; ni1++) { //loop over n1 values of n (color on of the bar)
                    LCD_drawPixel(x + ni1, y + yi, color_on);
                }
            }

        }


        if (n1 < 0) {
            for (ni2 = 0; ni2 <= length_max_v; ni2++) { //loop over n2 values of n (color off of the bar background)
                LCD_drawPixel(x + ni2, y + yi, color_off);
            }
            if (-n1 <= length_max_v) {
                for (ni1 = 0; ni1 <= -n1; ni1++) { //loop over n1 values of n (color on of the bar)
                    LCD_drawPixel(x - ni1, y + yi, color_on);
                }
                for (ni2 = -n1; ni2 <= length_max_v; ni2++) { //loop over n2 values of n (color off of the bar background)
                    LCD_drawPixel(x - ni2, y + yi, color_off);
                }
            }
            else {
                for (ni1 = 0; ni1 <= length_max_v; ni1++) { //loop over n1 values of n (color on of the bar)
                    LCD_drawPixel(x - ni1, y + yi, color_on);
                }
            }
        }



    }
}

void drawBarV(unsigned short x, unsigned short y, signed short n1o, unsigned short color_on, unsigned short color_off, unsigned int width, unsigned int length, signed short n1max) {

    unsigned short ni1 = 0;
    unsigned short ni2 = 0;
    unsigned short xi = 0;
    signed short length_max_v = length / 2;
    signed short n1 = n1o / n1max;

    for (xi = 0; xi <= width; xi++) { //loop over width of the bar

        if (n1 > 0) {
            for (ni2 = 0; ni2 <= length_max_v; ni2++) { //loop over n2 values of n (color off of the bar background)
                LCD_drawPixel(x + xi, y - ni2, color_off);
            }
            if (n1 < length_max_v) {
                for (ni1 = 0; ni1 <= n1; ni1++) { //loop over n1 values of n (color on of the bar)
                    LCD_drawPixel(x + xi, y + ni1, color_on);
                }
                for (ni2 = n1; ni2 <= length_max_v; ni2++) { //loop over n2 values of n (color off of the bar background)
                    LCD_drawPixel(x + xi, y + ni2, color_off);
                }
            }
            else {
                for (ni1 = 0; ni1 <= length_max_v; ni1++) { //loop over n1 values of n (color on of the bar)
                    LCD_drawPixel(x + xi, y + ni1, color_on);
                }
            }

        }


        if (n1 < 0) {
            for (ni2 = 0; ni2 <= length_max_v; ni2++) { //loop over n2 values of n (color off of the bar background)
                LCD_drawPixel(x + xi, y + ni2, color_off);
            }
            if (-n1 <= length_max_v) {
                for (ni1 = 0; ni1 <= -n1; ni1++) { //loop over n1 values of n (color on of the bar)
                    LCD_drawPixel(x + xi, y - ni1, color_on);
                }
                for (ni2 = -n1; ni2 <= length_max_v; ni2++) { //loop over n2 values of n (color off of the bar background)
                    LCD_drawPixel(x + xi, y - ni2, color_off);
                }
            }
            else {
                for (ni1 = 0; ni1 <= length_max_v; ni1++) { //loop over n1 values of n (color on of the bar)
                    LCD_drawPixel(x + xi, y - ni1, color_on);
                }
            }
        }



    }
}

void write_i2c(unsigned char add, unsigned char reg, unsigned char val) {
    i2c_master_start(); //begin the start sequence
    i2c_master_send(add << 1 | 0); // send the slave address of the register, left shifted by 1, clearing last bit and setting it to 0 indicating to write
    i2c_master_send(reg); //send the adress
    i2c_master_send(val); //send the bits
    i2c_master_stop(); //stop sequence

}

unsigned char read_i2c(unsigned char add, unsigned char reg) {
    i2c_master_start(); //begin the start sequence
    i2c_master_send(add << 1 | 0); // send the slave address, left shifted by 1, clearing last bit and setting it to 0 indicating to write
    i2c_master_send(reg); //send the address of the general purpose i/o register
    i2c_master_restart(); //restart
    i2c_master_send(add << 1 | 1); // send the slave address, left shifted by 1, clearing last bit and setting it to 1 indicating to write
    unsigned char r = i2c_master_recv(); //save the output of the slave
    i2c_master_ack(1); //done talking to the chip (0 if not sone but just recieved))
    i2c_master_stop(); //stop sequence
    return r; //return the value read by the chip
}

unsigned char read_multiple_i2c(unsigned char add, unsigned char reg1, unsigned char* outs, unsigned int length) {
    i2c_master_start(); //begin the start sequence
    i2c_master_send(add << 1 | 0); // send the slave address, left shifted by 1, clearing last bit and setting it to 0 indicating to write
    i2c_master_send(reg1); //send the address of the general purpose i/o register
    i2c_master_restart(); //restart
    i2c_master_send(add << 1 | 1); // send the slave address, left shifted by 1, clearing last bit and setting it to 1 indicating to write
    unsigned int i = 0;
    for (i = 0; i <= length; i++) { //loop over all reading registers
        outs[i] = i2c_master_recv(); //save the output of the slave
        if (i == length) { //when the loop reaches the last value
            i2c_master_ack(1); //not done talking to the chip 
        } else { //for every other value read out
            i2c_master_ack(0); //done talking to the chip  
        }
    }
    i2c_master_stop(); //stop sequence
}

//initialize LSM

void initExp() {
    ANSELBbits.ANSB2 = 0; //make B2 output digital
    ANSELBbits.ANSB3 = 0; //make B3 output digital

    i2c_master_setup();
    write_i2c(0b1101011, 0x12, 0b00000100); //communicate with crl3_c and set ifing (6th bit) to 1, which allows to read multiple registers in a row
    write_i2c(0b1101011, 0x10, 0b10000010); //communicate with ctrl1_xl and set the sample rate to 1.66 kHz (first four bits 1000), with 2g sensitivity (fifth and sixth bits to 00), and 100 Hz filter (last two bits to 10).
    write_i2c(0b1101011, 0x11, 0b10001100); //communicate with ctrl2_g and set the sample rate to 1.66 kHz (first four bits 1000), with with 1000 dps sensitivity (5th and 6th bits to 11)

}








/* TODO:  Add any necessary callback functions.
 */

/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
        USB_DEVICE_CDC_INDEX index,
        USB_DEVICE_CDC_EVENT event,
        void * pData,
        uintptr_t userData
        ) {
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *) userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch (event) {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *) pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *) pData)->breakDuration;

            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch (event) {
        case USB_DEVICE_EVENT_SOF:

            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuratio. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*) eventData;
            if (configuredEventData->configurationValue == 1) {
                /* Update LED to show configured state */

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t) & appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void) {
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if (appData.isConfigured == false) {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    } else {
        retVal = false;
    }

    return (retVal);
}

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

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Reset other flags */
    appData.sofEventHasOccurred = false;
    //appData.isSwitchPressed = false;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];

    appData.state = APP_STATE_INIT;

    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    TRISBbits.TRISB4 = 1; //pin B4 is an input
    TRISAbits.TRISA4 = 0; //pin A4 is an output
    LATAbits.LATA4 = 1; //set the initial output of pin A4 to high (3.3V)

    LCD_init(); //initialize LCD
    initExp(); //initialize LSM
    LCD_clearScreen(BLACK); //turn the whole screen back

    startTime = _CP0_GET_COUNT();
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )
  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    /* Update the application state machine based
     * on the current state */

    unsigned char message [20];
    unsigned char message1 [20]; //array that is going to contain the temperature (remember to save one extra character for the zero in sprint f)
    unsigned char message2 [20];
    unsigned char message3 [20];
    unsigned char message4 [20];
    unsigned char message5 [20];



    unsigned char bar_number_x [4]; //array that is going to contain the number indicating the acc
    signed short n = -10; //number that stores the progress of the progress bar
    unsigned char data [30]; //array that stores the output data from the LSM

    static unsigned int aa = 0;

    /* TODO: Initialize your application's state machine and other
     * parameters.
     * 
     * 
     */

    switch (appData.state) {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;


        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if (appData.isConfigured) {
                /* If the device is configured then lets start reading */
                appData.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:

            if (APP_StateReset()) {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

            appData.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
            if (appData.isReadComplete == true) {
                appData.isReadComplete = false;
                appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

                USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0,
                        &appData.readTransferHandle, appData.readBuffer,
                        APP_READ_BUFFER_SIZE);


                if (appData.readBuffer[0] == 'r') { //if the letter r is read by the computer
                    aa = 1;
                }


                if (appData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID) {
                    appData.state = APP_STATE_ERROR;
                    break;
                }
            }

            break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:
        case APP_STATE_CHECK_TIMER:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was received or a switch was pressed.
             * The isReadComplete flag gets updated in the CDC event handler. */

            if (appData.isReadComplete || _CP0_GET_COUNT() - startTime > (48000000 / 2 / 100)) {
                appData.state = APP_STATE_SCHEDULE_WRITE;
            }

            break;


        case APP_STATE_SCHEDULE_WRITE:


            if (APP_StateReset()) {
                break;
            }

            /* Setup the write */

            appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData.isWriteComplete = false;
            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;

            if (aa == 1) {
                read_multiple_i2c(0b1101011, 0x20, data, 14); //read the seven consecutive registers indicating temperature, acceleration in xyz, and angular velocity in xyz

                //signed short temp = data[1]<< 8 | data [0]; //temperature is given by two 8 bits number, store the second one, then shift on by 8 bits and store the first one
                signed short gyro_x = data[3] << 8 | data [2];
                signed short gyro_y = data[5] << 8 | data [4];
                signed short gyro_z = data[7] << 8 | data [6];
                signed short acc_x = data[9] << 8 | data [8];
                signed short acc_y = data[11] << 8 | data [10];
                signed short acc_z = data[13] << 8 | data [12];

                //sprintf(message1, "%d" , temp);  

                //writeString(10, 10, message1, WHITE, BLACK); //write temp starting from pixel at x=28 y=32

                if (abs(acc_x) < 1000) {
                    sprintf(message2, "Ax %d   ", acc_x);
                } else {
                    sprintf(message2, "Ax %d ", acc_x);
                }
                writeString(10, 20, message2, WHITE, BLACK);

                if (abs(acc_y) < 1000) {
                    sprintf(message3, "Ay %d   ", acc_y);
                } else {
                    sprintf(message3, "Ay %d ", acc_y);
                }

                writeString(10, 30, message3, WHITE, BLACK);

                sprintf(message4, "Gx %d    ", gyro_x);

                writeString(10, 40, message4, WHITE, BLACK);

                sprintf(message5, "Gy %d    ", gyro_y);

                writeString(10, 50, message5, WHITE, BLACK);


                
                
                len = sprintf(dataOut, "%d Ax %d Ay %d Az %d Gx %d Gy %d Gz %d \r\n", i, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
            i++;
            }
            else {
                len = 1;
                dataOut[0] = 0;
            }

            if (i > 100) {
                aa = 0;
                i=0;
            }
            
            if (appData.isReadComplete) {
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle,
                        appData.readBuffer, 1,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
            } else {
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle, dataOut, len,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                startTime = _CP0_GET_COUNT();
            }
            break;

        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if (appData.isWriteComplete == true) {
                appData.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}



/*******************************************************************************
 End of File
 */