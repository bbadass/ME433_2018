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
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <ST7735.h> //lcd library
#include<stdio.h> //sprintf
#include<i2c_master_noint.h>

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

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
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

void writeCharacter(unsigned short x, unsigned short y, char letter, unsigned short color_on, unsigned short color_off){
    
    unsigned short xi=0;   
    unsigned short yi=0;
    
    for (xi=0; xi<5; xi++) { //loop over the five pixels (in x) per character
        
        unsigned char pixels = ASCII[letter - 0x20][xi];
     
        for (yi=0; yi<8 ; yi++){ //loop over bits of each character (8 pixels in y)
            if ((pixels >> yi ) & 1) { // if the yith bit of the ascii (shift back by yi) is on, turn the pixel on
                LCD_drawPixel(x+xi, y+yi, color_on);
            }
            else{ // if the yith bit of the ascii (shift back by yi) is off, color the pixel of the off color
                LCD_drawPixel(x+xi, y+yi, color_off);
            }
        }
    }
    
    }

//function to write messages composed of multiple characters on the LCD
void writeString(unsigned short x, unsigned short y, unsigned char* words, unsigned short color_on, unsigned short color_off){
        
    unsigned int i=0;
    
    while (words[i]) { //run until there is a character, which works because sprintf has a null zero as its last character 
    writeCharacter(x+5*i, y, words[i], color_on, color_off); //shift by 5 pixels per letter
    i++;
    }
    
    }
    
    
    //function to draw bars on the LCD
    void drawBarO(unsigned short x, unsigned short y, signed short n1o, unsigned short color_on, unsigned short color_off, unsigned int width, unsigned int length, signed short n1max){
       
        unsigned short ni1=0;
        unsigned short ni2=0;
        unsigned short yi=0;
        signed short length_max_v = length/2;
        signed short n1 = n1o/n1max;
        
        for(yi=0; yi<=width; yi++){ //loop over length of the bar
            
            if(n1>0){
                 for (ni2=0; ni2<=length_max_v ; ni2++){ //loop over n2 values of n (color off of the bar background)
                     LCD_drawPixel(x-ni2, y+yi, color_off);
                      }  
                if (n1<=length_max_v){
                    for (ni1=0; ni1<=n1 ; ni1++){ //loop over n1 values of n (color on of the bar)
                     LCD_drawPixel(x+ni1, y+yi, color_on);
                    }
                     for (ni2=n1; ni2<=length_max_v ; ni2++){ //loop over n2 values of n (color off of the bar background)
                     LCD_drawPixel(x+ni2, y+yi, color_off);
                      }     
                     }
                 
                else{
                     for (ni1=0; ni1<=length_max_v ; ni1++){ //loop over n1 values of n (color on of the bar)
                     LCD_drawPixel(x+ni1, y+yi, color_on);
                    }
                    }
                
            }
            
           
            if(n1<0){
                for (ni2=0; ni2<=length_max_v ; ni2++){ //loop over n2 values of n (color off of the bar background)
                     LCD_drawPixel(x+ni2, y+yi, color_off);
                      } 
                if (-n1<=length_max_v){
                    for (ni1=0; ni1<=-n1 ; ni1++){ //loop over n1 values of n (color on of the bar)
                     LCD_drawPixel(x-ni1, y+yi, color_on);
                    }
                     for (ni2=-n1; ni2<=length_max_v ; ni2++){ //loop over n2 values of n (color off of the bar background)
                     LCD_drawPixel(x-ni2, y+yi, color_off);
                     }
                     }
                 
                else{
                     for (ni1=0; ni1<=length_max_v ; ni1++){ //loop over n1 values of n (color on of the bar)
                     LCD_drawPixel(x-ni1, y+yi, color_on);
                    }
                 }
            }
            
        
        
        }
    }
    
    void drawBarV(unsigned short x, unsigned short y, signed short n1o, unsigned short color_on, unsigned short color_off, unsigned int width, unsigned int length, signed short n1max){
       
        unsigned short ni1=0;
        unsigned short ni2=0;
        unsigned short xi=0;
        signed short length_max_v = length/2;
        signed short n1 = n1o/n1max;
        
        for(xi=0; xi<=width; xi++){ //loop over width of the bar
            
            if(n1>0){
                 for (ni2=0; ni2<=length_max_v ; ni2++){ //loop over n2 values of n (color off of the bar background)
                     LCD_drawPixel(x+xi, y-ni2, color_off);
                      }  
                if (n1<length_max_v){
                    for (ni1=0; ni1<=n1 ; ni1++){ //loop over n1 values of n (color on of the bar)
                     LCD_drawPixel(x+xi, y+ni1, color_on);
                    }
                     for (ni2=n1; ni2<=length_max_v ; ni2++){ //loop over n2 values of n (color off of the bar background)
                     LCD_drawPixel(x+xi, y+ni2, color_off);
                      }     
                     }
                 
                else{
                     for (ni1=0; ni1<=length_max_v ; ni1++){ //loop over n1 values of n (color on of the bar)
                     LCD_drawPixel(x+xi, y+ni1, color_on);
                    }
                    }
                
            }
            
           
            if(n1<0){
                for (ni2=0; ni2<=length_max_v ; ni2++){ //loop over n2 values of n (color off of the bar background)
                     LCD_drawPixel(x+xi, y+ni2, color_off);
                      } 
                if (-n1<=length_max_v){
                    for (ni1=0; ni1<=-n1 ; ni1++){ //loop over n1 values of n (color on of the bar)
                     LCD_drawPixel(x+xi, y-ni1, color_on);
                    }
                     for (ni2=-n1; ni2<=length_max_v ; ni2++){ //loop over n2 values of n (color off of the bar background)
                     LCD_drawPixel(x+xi, y-ni2, color_off);
                     }
                     }
                 
                else{
                     for (ni1=0; ni1<=length_max_v ; ni1++){ //loop over n1 values of n (color on of the bar)
                     LCD_drawPixel(x+xi, y-ni1, color_on);
                    }
                 }
            }
            
        
        
        }
    }
    

void write_i2c(unsigned char add, unsigned char reg, unsigned char val){
    i2c_master_start();                      //begin the start sequence
    i2c_master_send(add << 1|0);       // send the slave address of the register, left shifted by 1, clearing last bit and setting it to 0 indicating to write
    i2c_master_send(reg);                    //send the adress
    i2c_master_send(val);                    //send the bits
    i2c_master_stop();                       //stop sequence
    
}

unsigned char read_i2c(unsigned char add, unsigned char reg){
    i2c_master_start();                      //begin the start sequence
    i2c_master_send(add << 1|0);       // send the slave address, left shifted by 1, clearing last bit and setting it to 0 indicating to write
    i2c_master_send(reg);                   //send the address of the general purpose i/o register
    i2c_master_restart();                    //restart
    i2c_master_send(add << 1|1);       // send the slave address, left shifted by 1, clearing last bit and setting it to 1 indicating to write
    unsigned char r = i2c_master_recv();     //save the output of the slave
    i2c_master_ack(1);                       //done talking to the chip (0 if not sone but just recieved))
    i2c_master_stop();                      //stop sequence
    return r;                               //return the value read by the chip
}

unsigned char read_multiple_i2c(unsigned char add, unsigned char reg1, unsigned char* outs, unsigned int length){
    i2c_master_start();                      //begin the start sequence
    i2c_master_send(add << 1|0);       // send the slave address, left shifted by 1, clearing last bit and setting it to 0 indicating to write
    i2c_master_send(reg1);                   //send the address of the general purpose i/o register
    i2c_master_restart();                    //restart
    i2c_master_send(add << 1|1);       // send the slave address, left shifted by 1, clearing last bit and setting it to 1 indicating to write
    unsigned int i=0;
    for (i=0; i<=length; i++) {               //loop over all reading registers
    outs[i] = i2c_master_recv();             //save the output of the slave
    if(i==length){                           //when the loop reaches the last value
    i2c_master_ack(1);                       //not done talking to the chip 
    }
    else {                                   //for every other value read out
    i2c_master_ack(0);                       //done talking to the chip  
    }
    }
    i2c_master_stop();                      //stop sequence
}
    
//initialize LSM
void initExp (){
    ANSELBbits.ANSB2 = 0; //make B2 output digital
    ANSELBbits.ANSB3 = 0; //make B3 output digital
    
    i2c_master_setup();
    write_i2c(0b1101011,0x12,0b00000100); //communicate with crl3_c and set ifing (6th bit) to 1, which allows to read multiple registers in a row
    write_i2c(0b1101011,0x10,0b10000010); //communicate with ctrl1_xl and set the sample rate to 1.66 kHz (first four bits 1000), with 2g sensitivity (fifth and sixth bits to 00), and 100 Hz filter (last two bits to 10).
    write_i2c(0b1101011,0x11,0b10001100); //communicate with ctrl2_g and set the sample rate to 1.66 kHz (first four bits 1000), with with 1000 dps sensitivity (5th and 6th bits to 11)
    
}

//The following occurs only once
void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    
    TRISBbits.TRISB4=1; //pin B4 is an input
    TRISAbits.TRISA4=0; //pin A4 is an output
    LATAbits.LATA4=1; //set the initial output of pin A4 to high (3.3V)
    /* TODO: Initialize your application's state machine and other
     * parameters.
     * 
     * 
     */
    
    LCD_init(); //initialize LCD
    initExp(); //initialize LSM
    LCD_clearScreen(BLACK); //turn the whole screen back
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

//the following is inside the infinite while loop
void APP_Tasks ( void )
{
 
    
    
    unsigned char message [20];
    unsigned char message1 [20]; //array that is going to contain the temperature (remember to save one extra character for the zero in sprint f)
    unsigned char message2 [20];
    unsigned char message3 [20];
    unsigned char message4 [20];
    unsigned char message5 [20];
    
    
    
    unsigned char bar_number_x [4]; //array that is going to contain the number indicating the acc
    signed short n = -10; //number that stores the progress of the progress bar
    unsigned char data [30]; //array that stores the output data from the LSM
    
    
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
              _CP0_SET_COUNT(0);
                
        //make LED connected to A4 blink every half second
         while(_CP0_GET_COUNT()<24000000/5){ 
             LATAbits.LATA4=1;
         }
         _CP0_SET_COUNT(0);
                 
        while(_CP0_GET_COUNT()<24000000/5){
             LATAbits.LATA4=0;
         }
         _CP0_SET_COUNT(0);
         
         //sprintf(message, "%d" , read_i2c(0b1101011,0x0F)); //read whoami   
        
         //writeString(28, 32, message, WHITE, BLACK); //write whoami starting from pixel at x=28 y=32
         
        while(_CP0_GET_COUNT()<24000000/20){ //read at a 20 Hz frequency
         
        read_multiple_i2c(0b1101011,0x20,data,14); //read the seven consecutive registers indicating temperature, acceleration in xyz, and angular velocity in xyz
         
        //signed short temp = data[1]<< 8 | data [0]; //temperature is given by two 8 bits number, store the second one, then shift on by 8 bits and store the first one
        signed short gyro_x = data[3]<< 8 | data [2];
        signed short gyro_y = data[5]<< 8 | data [4];
        signed short acc_x = data[9]<< 8 | data [8];
        signed short acc_y = data[11]<< 8 | data [10];
         
         //sprintf(message1, "%d" , temp);  
        
         //writeString(10, 10, message1, WHITE, BLACK); //write temp starting from pixel at x=28 y=32
         
        if(abs(acc_x)<1000){
         sprintf(message2, "Ax %d   " , acc_x);  
        }
        else{
            sprintf(message2, "Ax %d " , acc_x); 
        }
         writeString(10, 20, message2, WHITE, BLACK);  
         
         if(abs(acc_y)<1000){
         sprintf(message3, "Ay %d   " , acc_y);   
         }
         else{
          sprintf(message3, "Ay %d " , acc_y);    
         }
        
         writeString(10, 30, message3, WHITE, BLACK);
         
         sprintf(message4, "Gx %d    " , gyro_x);    
        
         writeString(10, 40, message4, WHITE, BLACK); 
         
         sprintf(message5, "Gy %d    " , gyro_y); 
        
         writeString(10, 50, message5, WHITE, BLACK); 
         
         drawBarV(64, 80, acc_y, YELLOW, BLUE, 5, 140, 228); //draw progress bar starting from pixel at x=13 y=80
         
         drawBarO(64, 80, acc_x, YELLOW, BLUE, 5, 108, 296); //draw progress bar starting from pixel at x=13 y=80
        } 
            break;
        }
        
        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
