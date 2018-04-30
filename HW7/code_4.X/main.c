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
    
    unsigned char message [20];
    unsigned char message1 [20]; //array that is going to contain the temperature (remember to save one extra character for the zero in sprint f)
    unsigned char message2 [20];
    unsigned char message3 [20];
    unsigned char message4 [20];
    unsigned char message5 [20];
    
    
    
    unsigned char bar_number_x [4]; //array that is going to contain the number indicating the acc
    signed short n = -10; //number that stores the progress of the progress bar
    unsigned char data [30]; //array that stores the output data from the LSM
    //sprintf(message, "Ciao Mamma <3");
    
    LCD_clearScreen(BLACK); //turn the whole screen back
    //sprintf(message1, "Ciao Mamma <3");
    //writeString(28, 32, message1, WHITE, BLACK); 
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
    }
}