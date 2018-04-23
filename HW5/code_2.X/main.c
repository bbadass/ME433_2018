#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h" //ic2 library

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL =  ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 00000000 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

void write_i2c(unsigned char add, unsigned char val){
    i2c_master_start();                      //begin the start sequence
    i2c_master_send(0b0100000 << 1|0);       // send the slave address of the register, left shifted by 1, clearing last bit and setting it to 0 indicating to write
    i2c_master_send(add);                    //send the adress
    i2c_master_send(val);                    //send the bits
    i2c_master_stop();                       //stop sequence
    
}

unsigned char read_i2c(){
    i2c_master_start();                      //begin the start sequence
    i2c_master_send(0b0100000 << 1|0);       // send the slave address, left shifted by 1, clearing last bit and setting it to 0 indicating to write
    i2c_master_send(0x09);                   //send the address of the general purpose i/o register
    i2c_master_restart();                    //restart
    i2c_master_send(0b0100000 << 1|1);       // send the slave address, left shifted by 1, clearing last bit and setting it to 1 indicating to write
    unsigned char r = i2c_master_recv();     //save the output of the slave
    i2c_master_ack(1);                       //done talking to the chip (0 if not sone but just recieved))
    i2c_master_stop();                      //stop sequence
    return r;                               //return the value read by the chip
}
    
void initExp (){
    ANSELBbits.ANSB2 = 0; //make B2 output digital
    ANSELBbits.ANSB3 = 0; //make B3 output digital
    
    i2c_master_setup();
    write_i2c(0x00,0b11110000); //communicate with I/O direction register and tell it to make first four chips input (G7 to G4) and the last four output (G3 to G0))
    write_i2c(0x0A,0b00001111); //communicate with output LATCH register and tell it to turn all the output pins on                     
    
}

int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // set input and output pins
    TRISBbits.TRISB4=1; // B4 i an input
    TRISAbits.TRISA4=0; //A4 is an output
    LATAbits.LATA4=1; //A4 initially set to 3.3V
    
        initExp(); //initialize MCP23
    
    __builtin_enable_interrupts();
   
     _CP0_SET_COUNT(0);
                
        //make LED connected to A4 blink every half second
         while(_CP0_GET_COUNT()<24000000){ 
         }   
    
    while(1) {
        
        _CP0_SET_COUNT(0);
                
        //make LED connected to A4 blink every half second
         while(_CP0_GET_COUNT()<24000000/5){ 
             LATAbits.LATA4=1;
         }
         _CP0_SET_COUNT(0);
                 
        while(_CP0_GET_COUNT()<24000000/5){
             LATAbits.LATA4=0;
         }
         
         if ((read_i2c())>> 7 == 1){// if the output from G7 is high (button not pushed) (got the whole bit number, shift back by 7, so you only get first bit, which is G7)
         write_i2c(0x0A,0b00000001);} //make LED on
         else { //if out from G7 is low (button pushed)
             write_i2c(0x0A,0b00000000);} //make LED off
         
    }
}