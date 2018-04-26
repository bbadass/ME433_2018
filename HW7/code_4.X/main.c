#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "ST7735.h" //lcd library
#include<stdio.h> //sprintf

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

//function to write single characters on the LCD
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
    void drawBar(unsigned short x, unsigned short y, unsigned short n1, unsigned short color_on, unsigned short color_off, unsigned int length){
       
        unsigned short ni1=0;
        unsigned short ni2=0;
        unsigned short yi=0;
        
        for(yi=0; yi<=length; yi++){ //loop over length of the bar
        for (ni1=0; ni1<=n1 ; ni1++){ //loop over n1 values of n (color on of the bar)
                LCD_drawPixel(x+ni1, y+yi, color_on);
        }
        for (ni2=n1; ni2<100 ; ni2++){ //loop over n2 values of n (color off of the bar background)
                LCD_drawPixel(x+ni2, y+yi, color_off);
        }
        }
    }
    

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
    
    LCD_init(); //initialize LCD
    initExp(); //initialize LSM
    
    __builtin_enable_interrupts(); 
    
    
    unsigned char message [20]; //array that is going to contain the  hello world message (remember to save one extra character for the zero in sprint f)
    unsigned char bar_number [4]; //array that is going to contain the number indicating the progress in the progress bar (remember to save one extra character for the zero in sprint f)
    unsigned short n = 0; //number that stores the progress of the progress bar
    sprintf(message, "Ciao Mamma <3");
    
    LCD_clearScreen(BLACK); //turn the whole screen back
    
    while(1) {
        
        _CP0_SET_COUNT(0);
         
        
         writeString(28, 32, message, WHITE, BLACK); //write hello world starting from pixel at x=28 y=32
         
         
         while(_CP0_GET_COUNT()<24000000/10){ //update bar and progress number at a 10 Hz frequency
             
            sprintf(bar_number, "%d", n); 
             
            writeString(20, 70, bar_number, YELLOW, BLACK); //write progress number starting from pixel at x=85 y=32
         
            drawBar(20, 80, n, YELLOW, BLUE, 20); //draw progress bar starting from pixel at x=13 y=80
            
            n++;
            
            if(n==100){ //reset n to zero when the bar is full
                n=0;
            }
         }
    }
}