#include<xc.h>           // processor SFR definitions
#include<math.h>        //mathematical functions
#include<sys/attribs.h>  // __ISR macro

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


#define CS LATAbits.LATA0 //define channel identifier


// function to send a byte via SPI and return the response
    unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
    }
      
    
    //function to set the voltage (inputs are channel number and voltage in binary: note 3.3 V is 1023 in binary)
    void setVoltage (char a, int v) {
    unsigned short t; //variable for the output
    
    t = a<< 15; //go back to bit 1 to set channel
    
    t = t | 0b0111000000000000; //the output is a 16 bit number, where the first four set identifiers to set properties, and the last two
    t = t | ((v&0b1111111111)<<2); //10 bit number limiting the max voltage to 3.3V, last two bits are inactive, first 8 set voltage
    
    CS=0; //lower the chip select line and enable the MCP
    spi_io(t>>8); //send the first 8 bits: send t, go back by 8 bits
    spi_io(t&0xFF); //send last eight bits: first eight are zeros
    CS = 1; //disable the MCP
    }

   //function to initialize spin communication
    void spi_init() {
  // the chip select pin is used by the sram to indicate when a command is beginning (clear CS to low) and when it is ending (set CS high)
  TRISAbits.TRISA0 = 0; // set up the chip select pin as an output
  CS = 1; //no communication

  // Master - SPI4, pins are: SDI4(F4), SDO4(F5), SCK4(F13).  
  // we manually control SS4 as a digital output (F12)
  // since the pic is just starting, we know that spi is off. We rely on defaults here
 
  // setup spi4
  RPA1Rbits.RPA1R = 0b0011; //set pin A1 as output SDO1
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 03;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 4
                  
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

    // Set pins to input and outputs, and their initial values
    TRISBbits.TRISB4=1; //B4 is input (from push button)
    TRISAbits.TRISA4=0; //A4 is output (to LED)
    
    LATAbits.LATA4=1; //A4 outs 3.3V    
    

    spi_init();
    
    __builtin_enable_interrupts();
        
    double a=0;
    double x=0;
    double dx = 0.0314*2;
    float b=0;
    float db=5.12;
    
    while(1) {
        _CP0_SET_COUNT(0); // PIC timing set to zero  
        
        setVoltage(1,a); //channel A set to 1.65V
        
        setVoltage(0,b); //channel B set to 0.825V     
        
        a = 512/2 + 512/2*sin(x);
        x=x+dx;
        
        b=b+db; //slowly increase b such that half a triangle of 3.3V height is constructed in 100ms (b uodated every 1ms)
        if (b>512){ //when reached top of triangle start going down
            db=-db;
        }
        if (b<1){ //when reached bottom of triangle start going back up
            db=-db;
        }
        
         while(_CP0_GET_COUNT()< 24000){ //while loop with frequency 1kHz
                      }
    }
}