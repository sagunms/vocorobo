/*****************************************************************************
 VoCoRoBo: Remote Speech Recognition and Tilt-Sensing Multi-Robotic System
------------------------------------------------------------------------------
 Copyright © 2011 Sagun Man Singh Shrestha [Apache License v2. See LICENSE.TXT]
 [sagunms@gmail.com | sagunms.wordpress.com | http://sagunms.com.np]
 [https://github.com/sagunms/vocorobo]
 
 VoCoRoBo Team: Sagun Man Singh Shrestha, Labu Manandhar and Ritesh Bhattarai
 Kathmandu Engineering College, Tribhuvan University, Kathmandu, Nepal
******************************************************************************
 File: vocorobo.c 
 ----------------
* This is the Layer 1 of VoCoRoBo Control Module. Please see documentation.
* This code is the implementation of real-time speech recognition using eight
 4th order digital Chebyshev2 IIR filters in software and creating voice 
 fingerprints of spoken words (front, back, left, right and stop). 
 Euclidean distance is used to compare  the spoken word with the dictionary 
 to recognize the words.
* MMA7260Q 3-axis accelerometer connected to ADC are also interface for tilt
 sensing.
* The two control modes viz. Speech Recognition and Tilt Sensing are selected
 using appropriate pushbutton combinations. When a command is recognized, the
 corresponding bit combinations are sent to the Layer 2 of the Control Module
 for wireless transmission and cryptography.
 ----------------
 Embedded Platform: ATmega32 (F_CLK = 16 MHz)
 Interfacing Components: 
 Mic + 500 gain amp (PA0), 16x2 LCD (PC), MMA7260Q (PA3-5), ATK200(PA0-2),
 MAX233A (PD0,1), Pushbuttons(PB4-7), 16MHz XTAL, etc.
******************************************************************************
 Started: Saturday, May 6, 2010 at 12:30 AM
 Last Updated: Thursday, January 13, 2011 at 1:14 PM
******************************************************************************
 Project References:
 T. Aamodt, 2003. "Speech Recognition Algorithm", University of British Columbia.
 X. Lu, S. Lee, 2006. "Voice Recognition Security System", Cornell University.
 A. Harison, C. Shah, 2006. "Voice Recognition Car", Cornell University.
 B.R. Land.	"Fixed Point mathematical function in GCC and assembler", 
			"Optimized 2nd order IIR code", Cornell University.
 B.R. Land, Sept 2008. "Fast Digital Filtering". Circuit Cellar Issue #218, p.40.
 AVR201: Using the AVR® Hardware Multiplier, Atmel Corporation.
 IIR Design: nauticom.net/www/jdtaft/iir.htm
 T. Igoe, 2006 “MMA7260Q 3-Axis Accelerometer Report for PIC 18F252 using PicBasic 
	Pro”, Sensor Workshop at ITP. 
 Application Note AN3447: “Implementing Auto-zero calibration technique for 
	accelerometers”, Freescale Semiconductors.
*****************************************************************************/

#include <mega32.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <delay.h>
#include "dict.h"  // dictonary file created using template code

//--- LCD definitions ---
#asm(".equ __lcd_port=0x15")
#include <lcd.h>
#define LCDcolumns 16 //characters
char lcd_buffer[17];    //LCD display buffer

char mindis_itoa[7]; //to convert integer to string to display threshold value in LCD
char ocr1a_itoa[7];
char adc_itoa[7];
//-----------------------

#define begin 	{
#define end 	}
#define NSAMPLES_PER_INTERVAL 250
#define NSAMPLES 4000
#define NDIM     160
#define NWORDS   5 
#define NCOP     3
// Functionality of the Robot
#define go     	0
#define left   	1
#define right   2
#define stop    3
#define back   	4 

#pragma regalloc-              // Register Allocation
int x11_1, x11_2, x12_1, x12_2; // Filter1 Inputs
int y11_1, y11_2, y12_1, y12_2; // Filter1 Outputs 
int x21_1, x21_2, x22_1, x22_2; // Filter2 Inputs
int y21_1, y21_2, y22_1, y22_2; // Filter2 Outputs
int x31_1, x31_2, x32_1, x32_2; // Filter3 Inputs
int y31_1, y31_2, y32_1, y32_2; // Filter3 Outputs
int x41_1, x41_2, x42_1, x42_2; // Filter4 Inputs
int y41_1, y41_2, y42_1, y42_2; // Filter4 Outputs
int x51_1, x51_2, x52_1, x52_2; // Filter5 Inputs
int y51_1, y51_2, y52_1, y52_2; // Filter5 Outputs
int x61_1, x61_2, x62_1, x62_2; // Filter6 Inputs
int y61_1, y61_2, y62_1, y62_2; // Filter6 Outputs
int x71_1, x71_2, x72_1, x72_2; // Filter7 Inputs
int y71_1, y71_2, y72_1, y72_2; // Filter7 Outputs
int x81_1, x81_2, x82_1, x82_2; // Filter8 Inputs
int y81_1, y81_2, y82_1, y82_2; // Filter8 Outputs
int x91_1, x91_2, x92_1, x92_2; // Filter9 Inputs
int y91_1, y91_2, y92_1, y92_2; // Filter9 Outputs
int x101_1, x101_2, x102_1, x102_2;  // Filter10 Inputs
int y101_1, y101_2, y102_1, y102_2;  // Filter10 Outputs
// Stores Each filters output.
unsigned int y1=0, y2=0, y3=0, y4=0, y5=0, y6=0,y7=0,y8=0,y9=0,y10=0; 
// vairable to check filterout before squaring.
unsigned int gainchk;
#pragma regalloc+        // Register allcation finish.

int word[16][10];                                                 
char nms_d; // keeps track of if noise measurement is done
char it; // iteration counter
unsigned int noise1, noise2, noise3; // sample noise measurement values
char searchindx;   // in main|initialization statement set to zero
char WORDFOUND;       // word has been found
char Ain;          // value of ADCH (read in ADC interrupt
unsigned int threshold ;   // threshold for word vs. noise (3*avg noise ampltiude
char i, j,k, l, m;
char new_sample;   // new value in from ADC   
int fingerprint[NDIM];   
int y; 
char z; // misc temp var
int input;
double rtop, rxbot, rybot, r, dicmean, fingmean, sqval; // variables for correlation calculation 
double xdiff, ydiff;
double res, mindix;

//--- Speed Control definitions ----
#define MAXSPEED 19500
#define MINSPEED 9500
unsigned int speedpercentage;
#pragma warn-
        eeprom int speedeeprom;
#pragma warn+
void speedcontrolmode(void);
//----------------------------------

//--- Accelerometer defintions ---
unsigned short oldADCpin, ADCpin; 
unsigned int ADCtemp;
unsigned int ADCarray[3];
unsigned int xyzOrigin[3];
char xyzSpeed[3];

unsigned char xyzsampledflag;
unsigned char avgOrgflag;
unsigned char xyzResult;
unsigned char accelerometerflag; 

void accelerometermode(void); 
void displayOrientation(char val, unsigned char axis);
//---------------------------------

unsigned long int tmp1;
long int tmp;   // storing difference values from euclidean distance 
unsigned long int result;
unsigned long int mindis;
char minword;

void initialize();
void meas_noise();      // Measure noise analyses                                                    
void get_sample();
void lookup(); 
int multfix(int a,int b);   //Fixed point multiplication
int macfix(int a, int b, int c);   // Fixed point multiplier accumulator

unsigned int cycle;     
char segcnt, samcnt; // 250 samcnt & 32 segcnt
//
#define int2fix(a)   (((int)(a))<<8)            //Convert char to fix. a is a char
#define fix2int(a)   ((signed char)((a)>>8))    //Convert fix to char. a is an int 
#define fix2uint(a)   ((unsigned char)((a)>>8))    //Convert fix to char. a is an int 
#define float2fix(a) ((int)((a)*256.0)) 
#define fix2float(a) ((float)(a)/256.0)

#pragma regalloc-    
//We are not using filter1 since it has higher noise level.
 int A112 = 435; int A113 = -186; int B111 = 10; int B112 = -14; int B113 = 10;
 int A122 = 480; int A123 = -230; int B121 = 2459; int B122 = -4637; int B123 = 2459;
 int G1 = 140;   //7
 
 // Filter2: 1st 2nd order coefficient   
 int A212 = 451; int A213 = -248; int B211 = 21; int B212 = -32; int B213 = 21;
// Filter2: 2nd 2nd order coefficient  
 int A222 = 458; int A223 = -248; int B221 = 2225; int B222 = -4285; int B223 = 2225;
 int G2 =   80;   //4   // Filter2 gain
 
//Filter3: 1st 2nd order coefficient 
 int A312 = 355; int A313 = -248; int B311 = 27; int B312 = -29; int B313 = 27;
//Filter3: 2nd 2nd order coefficient 
 int A322 = 366; int A323 = -248; int B321 = 1090; int B322 = -1826; int B323 = 1090;
 int G3 = 120;    //6   //Filter3 gain
 
 //Filter4: 1st 2nd order coefficient 
 int A412 = 224; int A413 = -248; int B411 = 31; int B412 = -15; int B413 = 31;
 //Filter4: 2nd 2nd order coefficient  
 int A422 = 239; int A423 = -248; int B421 = 762; int B422 = -965; int B423 = 762;
 int G4 = 140;   //7  //Filter4 gain

//Filter5: 1st 2nd order coefficient  
 int A512 = 72; int A513 = -248; int B511 = 34; int B512 = 4; int B513 = 34;
//Filter5: 2nd 2nd order coefficient  
 int A522 = 88; int A523 = -248; int B521 = 633; int B522 = -464; int B523 = 633;
 int G5 = 160;  //8  //Filter5 gain
 
//Filter6: 1st 2nd order coefficient 
 int A612 = -72; int A613 = -248; int B611 = 34; int B612 = -4; int B613 = 34;
//Filter6: 2nd 2nd order coefficient  
 int A622 = -88; int A623 = -248; int B621 = 633; int B622 = 464; int B623 = 633;
 int G6 = 160;   //8 //Filter6 gain
 
//Filter7: 1st 2nd order coefficient 
 int A712 = -224; int A713 = -248; int B711 = 31; int B712 = 15; int B713 = 31;
//Filter7: 2nd 2nd order coefficient   
 int A722 = -239; int A723 = -248; int B721 = 762; int B722 = 965; int B723 = 762;
 int G7 = 140;  //7  //Filter7 gain
 
//Filter8: 1st 2nd order coefficient  
 int A812 = -355; int A813 = -248; int B811 = 27; int B812 = 29; int B813 = 27;
 //Filter8: 2nd 2nd order coefficient 
 int A822 = -366; int A823 = -248; int B821 = 1090; int B822 = 1826; int B823 = 1090;
 int G8 = 120;       //6   //Filter8 gain
 
//Filter9: 1st 2nd order coefficient  
 int A912 = -451; int A913 = -248; int B911 = 21; int B912 = 32; int B913 = 21;
//Filter9: 2nd 2nd order coefficient 
 int A922 = -458; int A923 = -248; int B921 = 2225; int B922 = 4285; int B923 = 2225;
 int G9 = 80;      //4 
 
//Filter10: 1st 2nd order coefficient 
 int A1012 = -435; int A1013 = -186; int B1011 = 10; int B1012 = 14; int B1013 = 10;
 //Filter10: 2nd 2nd order coefficient
 int A1022 = -480; int A1023 = -230; int B1021 = 2459; int B1022 = 4637; int B1023 = 2459;
 int G10 = 140;      //7 
#pragma regalloc+
//

//int filter1(int x);   // 0 - 200Hz Low pass Filter
int filter2(int x);     // 200Hz - 400Hz
int filter3(int x);     // 400Hz - 600Hz
int filter4(int x);     // 600Hz - 800Hz
int filter5(int x);     // 800Hz - 1000Hz
int filter6(int x);     // 1000Hz - 1200Hz
int filter7(int x);     // 1200Hz - 1400Hz
int filter8(int x);     // 1400Hz - 1600Hz
int filter9(int x);     // 1600Hz - 1800Hz
int macfix(int a, int b, int c);  // multiplier and accum
   
//////////////////////////////////////////////////////// 
// Timer0 interrupt used to measure noise and to take ADC data
interrupt [TIM0_COMP] void noiseanaly(void)
begin
     // To start ADC again
    Ain = ADCH;
    new_sample = 1;
    ADCSR.6=1; 
    if (z == 2) {
        TCCR0 = 0b00001011;// Prescale 64,clear on compare match WGM01 = 1,WGM00 = 0
        TCNT0 = 0;
        OCR0 = 62;// This is sampling about at 4300Hz
        z=0;
    }
end

//////////////////////////////////////////////////////////////////////////////
void initialize() { 

    searchindx = 0;
    WORDFOUND = 0;   // word found  
    
    DDRD = 0xFF;
	TCCR1A = 0b11000010;    //COMP1B 1:0 are disabled 'cause we need to use that signalling atk200 board
    TCCR1B = 0b00010010; 	// prescale /8 and CTC     
    OCR1A = speedeeprom;	//Load OCR1A value from EEPROM
    ICR1H  = 0X4E;	ICR1L =  0X20;      
    
    TIMSK = 0b00000010;   // Setup the interrupt for counter0 .
    Ain = 0;        //Adc data variable
    new_sample = 0;   // flag to indicate sample from adc
    nms_d = 0;      
    it =0;
    // setup timer interrupt to trigger every 125us 
    segcnt = 0;	samcnt = 0; 

    DDRB = 0b00001111;		//4 pin (Bit 0 to 3) are outputs, others (Bit 4 to 7) are inputs
    PORTB = 0b11110000;		//4 outputs (motors) OFF, rest have their internal pull-up resistors enabled 
    
    DDRC.7 = 1;     PORTD.7 = 1; 

	//ATK200 Communication Protocol Code
	//STOP = 000
	PORTD.2 = 0;	PORTD.3 = 0;	PORTD.4 = 0;

    //setup for Tranm    
    UCSRB = 0x18 ;    // UART to setup TX and Rx
    UBRRL = 103 ;     // Baud Rate for mega32.      
    TCCR0 = 0b00001011; //  prescale divide by 64, 122Hz pwm signal.  
    OCR0 = 62;

    //----BOOTSCREEN-----
    printf("\r\n\n----------------\r\n");
    printf("\r\nVoCoRoBo-10\r\n");
    printf("\r\n------------------\r\n");
    printf("\r\nVOICE CONTROL MODE\r\n");  
    printf("\r\nStarting...\r\n");
    
    //sagun lcd initialize---------------
    lcd_init(LCDcolumns);       //initialize the display
	lcd_clear();         //clear the display
    lcd_gotoxy(2,0);      //position to upper left on display
    lcd_putsf("VoCoRoBo-10");   //another string from flash
   	delay_ms(500);
	lcd_gotoxy(0,1);      //position to upper left on display
	lcd_putsf("- CONTROL MODE -");   //another string from flash
    // ----------------------------------------

    ///////////////////////////
    accelerometerflag = 0;
	if(PINB.5 == 0)
        speedcontrolmode();                 
    if(PINB.6 == 0)
        accelerometermode();
	// ----------------------------------------

    //Reinitialize ADC registers for voice sampling after accelerometer sampling SAGUN
    ADMUX = 0b00100000;  // Initialize ADC mux value 
    ADCSR = 0b11000111;  // Watch out Bit 4.
    
    #asm("sei")     
    meas_noise();     // To measureNoise 
    PORTD.7 = 0;
}

//////////////////////////////////////////////////////////////////////////////
// to measure the noise in the room at prgram start up.
void meas_noise(void) {
    i = 0;
    while (nms_d == 0) { // loop until noise measurement is done
        if (it == 0){
            if (new_sample == 1) {
                new_sample =0;
                noise1 += Ain;
                i++;                    
            }
        }
        else if (it == 1) {
            if (new_sample == 1) {
                new_sample = 0;
                noise2 += Ain;
                i++;               
            }
        }
        if (i == 255) {
            i = 0;
            it++;       //   iterate
            if (it > 2) nms_d =1;
                   
            TCNT0 = 0;            
            TCCR0 = 0b00001101; // in timer1 interrupt when match occurs turn off timer1 and start timer0 again.
            OCR0  = 255;
            z = 2;
        }
        else if (it == 2) {
            if (new_sample == 1) {
                new_sample = 0;
                noise3 += Ain;
                i++;
            } 
        }
    }
    if (noise1 > noise2) { 
        if (noise2 > noise3)
            threshold = noise2 <<2;
        else if (noise1 > noise3 )
            threshold = noise3 <<2;
        else threshold = noise1 <<2;
    }
    else if ( noise1 > noise3)
        threshold = noise1 << 2;
    else if (noise2 > noise3)
        threshold = noise3 <<2;
    else threshold = noise2 <<2;
} 

void get_sample(void) {
    if (WORDFOUND == 0) {    // word is still not found    
        if (((unsigned) int2fix(Ain) > threshold) ) {
            WORDFOUND = 1; 
            i = 0;
        }                                                                          
        
    }
}
////////////////////////////////////////////////////////////////////////////////

void lookup()
{	// by default assume the first word... 
	result = 0;
	mindis = 0;
	minword = 0; 
	m=0;  
	// The first for loop assings mindis to be initial distance from first word.
	for(j = 16; j < NDIM; j++){
	    tmp = dic[0][j] - fingerprint[j];
		tmp1 = abs(tmp);	//sagun
        mindis = mindis + (long int)tmp1;        
	}
	for(j = 0; j < NWORDS; j++){
	    result = 0;
	    for( i=16; i < NDIM; i++ ) {
	      
	        tmp = dic[j][i] - fingerprint[i];
	        tmp1 = abs(tmp);
	        result = result + (long int) tmp1;
	    } 
	    // To check the result is the minimum distance.
		if( result < mindis ) {
			mindis = result;
			minword = j;
		}
        	printf("\r\nWord %d: %d\r\n",m++, result); //%ld
	}
    printf("\r\n-----------------------\r\n");	//%ld                                             
	printf("\r\nMinimum distance: %d\r\n",mindis);	//%ld
}

/*
This is Robot control function which sends different bit 
combinations to L293D H-bridge depending on values of minword 
set from lookup() function.
*/
void roboControl(void){
	//lcd_clear();
	if(accelerometerflag == 0) {
		lcd_gotoxy(0,0);
		lcd_putsf("Min Dist: ");
	}
	lcd_gotoxy(0,1);
	lcd_putsf("You said: ");
    if(minword == go){
		//printf("\nFront");
        lcd_gotoxy(10,0);
        itoa(abs(mindis),mindis_itoa);
        lcd_puts(mindis_itoa); 
        lcd_gotoxy(10,1);  
        lcd_putsf("FRONT ");
        PORTB = 0b11110110;		//the 0s are the active pins (active low).
        PORTD.7 = 0;        
        //Rest (Bit 4 to 7) are HIGH for enabling internal pull-up resistors for switches
		//ATK200 Communication Protocol Code
		//FRONT = 001
		PORTD.2 = 1;	PORTD.3 = 0;	PORTD.4 = 0;
    }
    else if(minword == left){
        //printf("\nLeft");
        lcd_gotoxy(10,0);
        itoa(abs(mindis),mindis_itoa);
        lcd_puts(mindis_itoa); 
        lcd_gotoxy(10,1);  
        lcd_putsf("LEFT  "); 
        PORTB = 0b11111010;
        PORTD.7 = 0;  
		//ATK200 Communication Protocol Code
		//LEFT = 011
		PORTD.2 = 1;	PORTD.3 = 1;	PORTD.4 = 0;		
    }
    else if(minword == right){
		//printf("\nRight");
        lcd_gotoxy(10,0);
        itoa(abs(mindis),mindis_itoa);
        lcd_puts(mindis_itoa); 
        lcd_gotoxy(10,1);  
        lcd_putsf("RIGHT ");
        PORTB = 0b11110101;
        PORTD.7 = 0;
		//ATK200 Communication Protocol Code
		//RIGHT = 100
		PORTD.2 = 0;	PORTD.3 = 0;	PORTD.4 = 1;	
    } 
    else if(minword == stop){
		//printf("\nStop");
        lcd_gotoxy(10,0);
        itoa(abs(mindis),mindis_itoa);
        lcd_puts(mindis_itoa); 
        lcd_gotoxy(10,1);  
        lcd_putsf("STOP  ");
        PORTB = 0b11111111;
        PORTB = 0b11110000;
        PORTD.7 = 1;
		//ATK200 Communication Protocol Code
		//STOP = 000
		PORTD.2 = 0;	PORTD.3 = 0;	PORTD.4 = 0;	
    }
    else if(minword == back){
	//printf("\nBack");
        lcd_gotoxy(10,0);
        itoa(abs(mindis),mindis_itoa);
        lcd_puts(mindis_itoa); 
        lcd_gotoxy(10,1);    
        lcd_putsf("BACK  ");
        PORTB = 0b11111001;
        PORTD.7 = 0;
		//ATK200 Communication Protocol Code
		//BACK = 010
		PORTD.2 = 0;	PORTD.3 = 1;	PORTD.4 = 0;	
    }  
    else{
	//printf("\nStop");
        lcd_gotoxy(10,0);
        itoa(abs(mindis),mindis_itoa);
        lcd_puts(mindis_itoa); 
        lcd_gotoxy(10,1);   
        lcd_putsf("STOP  ");
        PORTB = 0b11110000;
        PORTD.7 = 1;
		//SAGUN's ATK200 Communication Protocol Code
		//STOP = 000
		PORTD.2 = 0;	PORTD.3 = 0;	PORTD.4 = 0;	
    }
} 

void speedcontrolmode(void) {
        lcd_clear(); 
        lcd_gotoxy(1,0);
        lcd_putsf("SPEED CONTROL");
        
        while(1) {
                speedpercentage = (OCR1A - MINSPEED) / 100;
                itoa(speedpercentage, ocr1a_itoa);
				
                if(speedpercentage < 100) {
                        if(speedpercentage <10) {     //When speed percentage changes from 3 digit to 2 digit, clear the previous residual digit
                                lcd_gotoxy(1,1);
                                lcd_putsf("%  ");
                        }
                        else {
                                lcd_gotoxy(2,1);
                                lcd_putsf("% ");
                        }                                        
                }
                else {
                        lcd_gotoxy(3,1);
                        lcd_putsf("%");        
                }
                
                lcd_gotoxy(0,1);
                lcd_puts(ocr1a_itoa);                                              
                
                if(OCR1A < 10000) {                //When OCR1A changes from 5 digit to 4 digit, clear the previous residual digit
                        lcd_gotoxy(11,1);
                        lcd_putsf(" "); 
                        lcd_gotoxy(12,1);
                }
                else
                        lcd_gotoxy(11,1);       
                itoa(OCR1A, ocr1a_itoa);
                lcd_puts(ocr1a_itoa);

					if(PINB.4 == 0) {       //SPEED UP
							delay_ms(500);          //Debounce
							if(PINB.4 == 0) {
									//oldocr1a = OCR1A;
									if(OCR1A < MAXSPEED)
											OCR1A +=  500;          //800;
									else
											OCR1A = MAXSPEED;
							}                        
					}
					else if(PINB.7 == 0) {  //SPEED DOWN
							delay_ms(500);          //Debounce
							if(PINB.7 == 0) {
									//oldocr1a = OCR1A;
									if(OCR1A > MINSPEED)
											OCR1A -= 500;            //800;
									else
											OCR1A = MINSPEED;
							}
					}
					else if(PINB.6 == 0) {          //Exit Speed Control Menu
							speedeeprom = OCR1A;    //Save EEPROM speed value when speed menu exits
							break;
					}
        }                    
}

void accelerometermode(void) {
        accelerometerflag = 1;  //To prevent the display of Min Dist. text during execution of roboControl() function
        
        ADCpin = 0;  // X, Y, Z:  0, 1, 2
        oldADCpin = 0;
        xyzsampledflag = 0;
        avgOrgflag = 0;
        xyzOrigin[0] = 0;
        xyzOrigin[1] = 0;
        xyzOrigin[2] = 0;
        
        xyzSpeed[0] = 0;
        xyzSpeed[1] = 0;
        xyzSpeed[2] = 0;

        //ADC pin 3-5
        ADMUX = (1<<5)|5;		// Channel 5 only
        ADCSR = 0b11000111;
        
        lcd_clear(); 
        lcd_gotoxy(0,0);
        lcd_putsf("MMA7260Q CONTROL");
        
        for (;;)  {     // main loop
                if ((ADCSRA & (1<<6)) == 0) {	// If ADC conversion has finished
                        ADCtemp = ADCH;
        
                        if (++ADCpin > 2) {      //if all x,y,z values are sampled, return to the first x value
                                ADCpin = 0;
                                xyzsampledflag = 1;
                        }
                                
                        ADMUX = (1<<5)|(5 - ADCpin); 
                        ADCSRA |= (1<<6);	// Start new ADC conversion
                        ADCarray[oldADCpin] =  ADCtemp;
                	oldADCpin = ADCpin;
                        if(xyzsampledflag == 1) {       //Print only after sampling of all 3 values x,y,z are complete and put in array
                	        if(avgOrgflag < 3) {
                                        xyzOrigin[0] += ADCarray[0];
                                        xyzOrigin[1] += ADCarray[1];               
                                        xyzOrigin[2] += ADCarray[2];
                                        avgOrgflag++;
                                }
                                else if(avgOrgflag == 3) {
                                        xyzOrigin[0] /= 3;
                                        xyzOrigin[1] /= 3;
                                        xyzOrigin[2] /= 3;
                                        printf("\r\nOrigin [%d,%d,%d]",xyzOrigin[0],xyzOrigin[1],xyzOrigin[2]);
                                        avgOrgflag++;
                                }
                                else {
                        	        //
                        	        xyzResult =  ADCarray[0];
                                        if(xyzResult > xyzOrigin[0]) {
                                	        xyzSpeed[0] = xyzResult - xyzOrigin[0] ;
                                        }
                                        else {
                                	        xyzSpeed[0] = xyzOrigin[0] - xyzResult ;
                                		xyzSpeed[0] |= 0x80; // set to negative number
                                        }
                                        //printf("\nX=%d",xyzSpeed[0]);
                                        //
                                        xyzResult =  ADCarray[1]; // read Y
                                        if(xyzResult > xyzOrigin[1]) {
                                		xyzSpeed[1] = xyzResult - xyzOrigin[1];
                                        }
                                        
                                        else{
                                		xyzSpeed[1] = xyzOrigin[1] - xyzResult;
                                		xyzSpeed[1] |= 0x80;
                                        }
                                        //printf("\nY=%d",xyzSpeed[1]);
                                        //
                                        xyzResult =  ADCarray[2]; // read Z
                                        if(xyzResult > xyzOrigin[2]){
                                		xyzSpeed[2] = xyzResult - xyzOrigin[2];
                                        }
                                        
                                        else{
                                		xyzSpeed[2] = xyzOrigin[2] - xyzResult;
                                		xyzSpeed[2] |= 0x80;
                                        }
                                        //printf("\nZ=%d",xyzSpeed[2]);
                                        printf("\tSpeed [%d,%d,%d]",xyzSpeed[0],xyzSpeed[1],xyzSpeed[2]);
                                }
                                
                                printf("\r\n[%d,%d,%d]",ADCarray[0],ADCarray[1],ADCarray[2]);
                	        xyzsampledflag = 0;
                	        
                	        displayOrientation(xyzSpeed[0], 0);
                	        displayOrientation(xyzSpeed[1], 1); 
                	}
                }
        }
}
        

void displayOrientation(char val, unsigned char axis) {		//Axis: x=0, y=1, z=2

	if( !(val & 0x80) && val ) {    // if positive
		// print on the right
		if( val > 10 ) {
			if(axis == 1) {
				//lcd_gotoxy(0,1); 
				//lcd_putsf("FRONT");
				minword = go;
			}
			if(axis == 0) {
				//lcd_gotoxy(0,1); 
				//lcd_putsf("LEFT ");
				minword = left;
			} 
        }
	}
	else if(val) {            // if negative
		val = val & 0x7F;
		// print on le left
		if( val > 10 ) {
			if(axis == 1) {
				//lcd_gotoxy(0,1); 
				//lcd_putsf("BACK ");
				minword = back;
			}
			if(axis == 0) {
				//lcd_gotoxy(0,1); 
				//lcd_putsf("RIGHT");
				minword = right;
			} 
		}
		else {
			//lcd_gotoxy(0,1); 
			//lcd_putsf("STOP ");
			minword = stop;
		}
                 
	}
	//else PORTB = 0;            // if zeros
	
	else { 
                //lcd_gotoxy(0,1); 
                //lcd_putsf("STOP ");
                minword = stop;
        }
        
        roboControl(); 
}
      
 /////////////////////////////////////////////////////////////////////
//========================================================   
//second order IIR -- "Direct Form II Transposed"
//  y(n) = b(1)*x(n) + b(2)*x(n-1) +  b(3)*x(n-2)
//                   - a(2)*y(n-1) -  a(3)*y(n-2) 
//assumes a(1)=1
// a's and b's need to be global, in RAM and set to fixed point values  
// also input and output history values
//example: 
//      #pragma regalloc-
//      int b1,b2,b3,a2,a3, xn_1, xn_2, yn_1, yn_2 ;
//      #pragma regalloc+ 
//      b1=0x0010;
//      a1=float2fix(-(value from matlab))   
//
// The following ASM code is equivalent to:
//     yy=0; yy = macfix(b1,xx,yy); 
//     yy = macfix(b2,xn_1,yy);
//     yy = macfix(b3,xn_2,yy);
//     yy = macfix(-a2,yn_1,yy);
//     yy = macfix(-a3,yn_2,yy); 
//     //update the state variables
//     xn_2 = xn_1;
//     xn_1 = xx;
//     yn_2 = yn_1;
//     yn_1 = yy;
//     return yy;


int filter1(int xx)
// xx is the current input signal sample
// returns the current filtered output sample
begin
    #asm
    ;.macro mult_acc         ;r31:r30:r24 += r23:r22 * r21:r20
    ;    muls r23, r21		; (signed)ah * (signed)bh
    ;	add	r31, r0
    ;	mul	r22, r20		; al * bl
    ;	add	r24, r0
    ;	adc	r30, r1
    ;	adc	r31, r27
    ;	mulsu   r23, r20        ; (signed)ah * bl
    ;	add	r30, r0
    ;	adc	r31, r1
    ;	mulsu	r21, r22	; (signed)bh * al
    ;	add	r30, r0
    ;	adc	r31, r1 
    	
    ; .endm 
    
    push r20    ;save parameter regs
    push r21
    
    clr r27             ;permanent zero
    clr r24             ;clear 24 bit result reg; msb to lsb => r31:r30:r24
    clr r30      
    clr r31
    
    lds  R22, _B111       ;load b1 from RAM
    lds  R23, _B111+1 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    mult_acc            ; b1*xx  
    
    lds  R22, _B112       ;load b2 from RAM
    lds  R23, _B112+1 
    lds  R20, _x11_1     ;load x(n-1) from  RAM
    lds  R21, _x11_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B113       ;load b3 from RAM
    lds  R23, _B113+1 
    lds  R20, _x11_2     ;load x(n-2) from  RAM
    lds  R21, _x11_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A112       ;load -a2 from RAM
    lds  R23, _A112+1 
    lds  R20, _y11_1     ;load y(n-1) from  RAM
    lds  R21, _y11_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A113       ;load -a3 from RAM
    lds  R23, _A113+1 
    lds  R20, _y11_2     ;load y(n-2) from  RAM
    lds  R21, _y11_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x11_1     ;load x(n-1) from  RAM
    lds  R21, _x11_1+1
    sts  _x11_2, r20      ;store x(n-2) to  RAM
    sts  _x11_2+1, R21 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    sts  _x11_1, r20     ;store x(n-1) to  RAM
    sts  _x11_1+1, R21 
    lds  R20, _y11_1     ;load y(n-1) from  RAM
    lds  R21, _y11_1+1
    sts  _y11_2, R20     ;store y(n-2) to  RAM
    sts  _y11_2+1, R21 
    sts  _y11_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y11_1+1, r31  
     
  ;*************************Second 2nd order IIR filter.**************
    lds  R22, _B121       ;load b1 from RAM
    lds  R23, _B121+1 
    lds   R20, _y11_1         ;load input parameter xx from stack
    lds  R21, _y11_1+1 
    clr  r30
    clr  r31
    mult_acc            ; b1*xx  
    
    lds  R22, _B122       ;load b2 from RAM
    lds  R23, _B122+1 
    lds  R20, _x12_1     ;load x(n-1) from  RAM
    lds  R21, _x12_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B123       ;load b3 from RAM
    lds  R23, _B123+1 
    lds  R20, _x12_2     ;load x(n-2) from  RAM
    lds  R21, _x12_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A122       ;load -a2 from RAM
    lds  R23, _A122+1 
    lds  R20, _y12_1     ;load y(n-1) from  RAM
    lds  R21, _y12_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A123       ;load -a3 from RAM
    lds  R23, _A123+1 
    lds  R20, _y12_2     ;load y(n-2) from  RAM
    lds  R21, _y12_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x12_1     ;load x(n-1) from  RAM
    lds  R21, _x12_1+1
    sts  _x12_2, r20      ;store x(n-2) to  RAM
    sts  _x12_2+1, R21 
    lds   R20, _y11_1         ;load input parameter xx from stack
    lds  R21, _y11_1+1
    sts  _x12_1, r20     ;store x(n-1) to  RAM
    sts  _x12_1+1, R21 
    lds  R20, _y12_1     ;load y(n-1) from  RAM
    lds  R21, _y12_1+1
    sts  _y12_2, R20     ;store y(n-2) to  RAM
    sts  _y12_2+1, R21 
    sts  _y12_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y12_1+1, r31  

;*************************End of Second 2nd order IIR filter.************** 
    ; Adding new stuff
    ; multipling filter output w/ gain then 
    ; squaring the output of multiplier 
    mov r23,r31
    mov r22,r30
    clr r30
    clr r31
    lds r20, _G1
    lds r21,_G1+1
    
     muls r23, r21		   ; (signed)ah * (signed)bh
	add	r31, r0
	mul	r22, r20		   ; al * bl
	add	r24, r0
	adc	r30, r1
	adc	r31, r27
	mulsu r23, r20		; (signed)ah * bl
	add	r30, r0
	adc	r31, r1
	mulsu	r21, r22		; (signed)bh * al
	add	r30, r0
	adc	r31, r1 	
      mov r23,r31
      mov r22,r30
    ;Here multiplier with gain finishes
    ;Now start squaring y variable 
    
    lds r30,_y1
    lds r31,_y1+1
    clr r24
    
    muls r23,r23
    add  r31,r0
    mul  r22,r22
    add  r24,r0
    adc  r30,r1
    adc  r31,r27
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1    
    
	
    pop r21             ;restore parameter regs
    pop r20
	
    #endasm 
	
end
//*********************FILTER 2*******************

int filter2(int xx)
// xx is the current input signal sample
// returns the current filtered output sample
begin
    #asm
        .macro mult_acc         ;r31:r30:r24 += r23:r22 * r21:r20
        muls r23, r21		; (signed)ah * (signed)bh
    	add	r31, r0
    	mul	r22, r20		; al * bl
    	add	r24, r0
    	adc	r30, r1
    	adc	r31, r27
    	mulsu   r23, r20        ; (signed)ah * bl
    	add	r30, r0
    	adc	r31, r1
    	mulsu	r21, r22	; (signed)bh * al
    	add	r30, r0
    	adc	r31, r1 
    	
    .endm 
    
    push r20    ;save parameter regs
    push r21
    
    clr r27             ;permanent zero
    clr r24             ;clear 24 bit result reg; msb to lsb => r31:r30:r24
    clr r30      
    clr r31
    
    lds  R22, _B211       ;load b1 from RAM
    lds  R23, _B211+1 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    mult_acc            ; b1*xx  
    
    lds  R22, _B212       ;load b2 from RAM
    lds  R23, _B212+1 
    lds  R20, _x21_1     ;load x(n-1) from  RAM
    lds  R21, _x21_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B213       ;load b3 from RAM
    lds  R23, _B213+1 
    lds  R20, _x21_2     ;load x(n-2) from  RAM
    lds  R21, _x21_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A212       ;load -a2 from RAM
    lds  R23, _A212+1 
    lds  R20, _y21_1     ;load y(n-1) from  RAM
    lds  R21, _y21_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A213       ;load -a3 from RAM
    lds  R23, _A213+1 
    lds  R20, _y21_2     ;load y(n-2) from  RAM
    lds  R21, _y21_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x21_1     ;load x(n-1) from  RAM
    lds  R21, _x21_1+1
    sts  _x21_2, r20      ;store x(n-2) to  RAM
    sts  _x21_2+1, R21 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    sts  _x21_1, r20     ;store x(n-1) to  RAM
    sts  _x21_1+1, R21 
    lds  R20, _y21_1     ;load y(n-1) from  RAM
    lds  R21, _y21_1+1
    sts  _y21_2, R20     ;store y(n-2) to  RAM
    sts  _y21_2+1, R21 
    sts  _y21_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y21_1+1, r31  
   
    
  ;*************************Second 2nd order IIR filter.**************
    lds  R22, _B221       ;load b1 from RAM
    lds  R23, _B221+1 
    lds   R20, _y21_1         ;load input parameter xx from stack
    lds  R21, _y21_1+1
    clr r30
    clr r31
    mult_acc            ; b1*xx  
    
    lds  R22, _B222       ;load b2 from RAM
    lds  R23, _B222+1 
    lds  R20, _x22_1     ;load x(n-1) from  RAM
    lds  R21, _x22_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B223       ;load b3 from RAM
    lds  R23, _B223+1 
    lds  R20, _x22_2     ;load x(n-2) from  RAM
    lds  R21, _x22_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A222       ;load -a2 from RAM
    lds  R23, _A222+1 
    lds  R20, _y22_1     ;load y(n-1) from  RAM
    lds  R21, _y22_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A223       ;load -a3 from RAM
    lds  R23, _A223+1 
    lds  R20, _y22_2     ;load y(n-2) from  RAM
    lds  R21, _y22_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x22_1     ;load x(n-1) from  RAM
    lds  R21, _x22_1+1
    sts  _x22_2, r20      ;store x(n-2) to  RAM
    sts  _x22_2+1, R21 
    lds   R20, _y21_1         ;load input parameter xx from stack
    lds  R21, _y21_1+1
    sts  _x22_1, r20     ;store x(n-1) to  RAM
    sts  _x22_1+1, R21 
    lds  R20, _y22_1     ;load y(n-1) from  RAM
    lds  R21, _y22_1+1
    sts  _y22_2, R20     ;store y(n-2) to  RAM
    sts  _y22_2+1, R21 
    sts  _y22_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y22_1+1, r31  
;*************************End of Second 2nd order IIR filter.************** 
       ; Adding new stuff
    ; multipling filter output w/ gain then 
    ; squaring the output of multiplier 
    mov r23,r31
    mov r22,r30
    clr r30
    clr r31
    lds r20, _G2
    lds r21,_G2+1
    
     muls r23, r21		   ; (signed)ah * (signed)bh
	add	r31, r0
	mul	r22, r20		   ; al * bl
	add	r24, r0
	adc	r30, r1
	adc	r31, r27
	mulsu r23, r20		; (signed)ah * bl
	add	r30, r0
	adc	r31, r1
	mulsu	r21, r22		; (signed)bh * al
	add	r30, r0
	adc	r31, r1 
	sts _gainchk,r30 
	sts _gainchk+1,r31
      mov r23,r31
      mov r22,r30
    ;Here multiplier with gain finishes
    ;Now start squaring y variable 
    
    lds r30,_y2
    lds r31,_y2+1
    clr r24
    
    muls r23,r23
    add  r31,r0
    mul  r22,r22
    add  r24,r0
    adc  r30,r1
    adc  r31,r27
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1    
    
	
    pop r21             ;restore parameter regs
    pop r20
	
    #endasm 
	
end  
//**************************FILTER 3**************************
int filter3(int xx)
// xx is the current input signal sample
// returns the current filtered output sample
begin
    #asm

    push r20    ;save parameter regs
    push r21
    
    clr r27             ;permanent zero
    clr r24             ;clear 24 bit result reg; msb to lsb => r31:r30:r24
    clr r30      
    clr r31
    
    lds  R22, _B311       ;load b1 from RAM
    lds  R23, _B311+1 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    mult_acc            ; b1*xx  
    
    lds  R22, _B312       ;load b2 from RAM
    lds  R23, _B312+1 
    lds  R20, _x31_1     ;load x(n-1) from  RAM
    lds  R21, _x31_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B313       ;load b3 from RAM
    lds  R23, _B313+1 
    lds  R20, _x31_2     ;load x(n-2) from  RAM
    lds  R21, _x31_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A312       ;load -a2 from RAM
    lds  R23, _A312+1 
    lds  R20, _y31_1     ;load y(n-1) from  RAM
    lds  R21, _y31_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A313       ;load -a3 from RAM
    lds  R23, _A313+1 
    lds  R20, _y31_2     ;load y(n-2) from  RAM
    lds  R21, _y31_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x31_1     ;load x(n-1) from  RAM
    lds  R21, _x31_1+1
    sts  _x31_2, r20      ;store x(n-2) to  RAM
    sts  _x31_2+1, R21 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    sts  _x31_1, r20     ;store x(n-1) to  RAM
    sts  _x31_1+1, R21 
    lds  R20, _y31_1     ;load y(n-1) from  RAM
    lds  R21, _y31_1+1
    sts  _y31_2, R20     ;store y(n-2) to  RAM
    sts  _y31_2+1, R21 
    sts  _y31_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y31_1+1, r31  
    
    
  ;*************************Second 2nd order IIR filter.**************
    lds  R22, _B321       ;load b1 from RAM
    lds  R23, _B321+1 
    lds   R20, _y31_1         ;load input parameter xx from stack
    lds  R21, _y31_1+1 
    clr r30
    clr r31
    mult_acc            ; b1*xx  
    
    lds  R22, _B322       ;load b2 from RAM
    lds  R23, _B322+1 
    lds  R20, _x32_1     ;load x(n-1) from  RAM
    lds  R21, _x32_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B323       ;load b3 from RAM
    lds  R23, _B323+1 
    lds  R20, _x32_2     ;load x(n-2) from  RAM
    lds  R21, _x32_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A122       ;load -a2 from RAM
    lds  R23, _A122+1 
    lds  R20, _y32_1     ;load y(n-1) from  RAM
    lds  R21, _y32_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A323       ;load -a3 from RAM
    lds  R23, _A323+1 
    lds  R20, _y32_2     ;load y(n-2) from  RAM
    lds  R21, _y32_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x32_1     ;load x(n-1) from  RAM
    lds  R21, _x32_1+1
    sts  _x32_2, r20      ;store x(n-2) to  RAM
    sts  _x32_2+1, R21 
    lds   R20, _y31_1         ;load input parameter xx from stack
    lds  R21, _y31_1+1
    sts  _x32_1, r20     ;store x(n-1) to  RAM
    sts  _x32_1+1, R21 
    lds  R20, _y32_1     ;load y(n-1) from  RAM
    lds  R21, _y32_1+1
    sts  _y32_2, R20     ;store y(n-2) to  RAM
    sts  _y32_2+1, R21 
    sts  _y32_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y32_1+1, r31  
;*************************End of Second 2nd order IIR filter.************** 
    ; Adding new stuff
    ; multipling filter output w/ gain then 
    ; squaring the output of multiplier 
    mov r23,r31
    mov r22,r30
    clr r30
    clr r31
    lds r20, _G3
    lds r21,_G3+1
    
     muls r23, r21		   ; (signed)ah * (signed)bh
	add	r31, r0
	mul	r22, r20		   ; al * bl
	add	r24, r0
	adc	r30, r1
	adc	r31, r27
	mulsu r23, r20		; (signed)ah * bl
	add	r30, r0
	adc	r31, r1
	mulsu	r21, r22		; (signed)bh * al
	add	r30, r0
	adc	r31, r1 
      mov r23,r31
      mov r22,r30
    ;Here multiplier with gain finishes
    ;Now start squaring y variable 
    
    lds r30,_y3
    lds r31,_y3+1
    clr r24
    
    muls r23,r23
    add  r31,r0
    mul  r22,r22
    add  r24,r0
    adc  r30,r1
    adc  r31,r27
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1    
    
    pop r21             ;restore parameter regs
    pop r20
	
    #endasm 
	
end 
//**********************FILTER 4***************************
int filter4(int xx)
// xx is the current input signal sample
// returns the current filtered output sample
begin
    #asm
    push r20    ;save parameter regs
    push r21
    
    clr r27             ;permanent zero
    clr r24             ;clear 24 bit result reg; msb to lsb => r31:r30:r24
    clr r30      
    clr r31
    
    lds  R22, _B411       ;load b1 from RAM
    lds  R23, _B411+1 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    mult_acc            ; b1*xx  
    
    lds  R22, _B412       ;load b2 from RAM
    lds  R23, _B412+1 
    lds  R20, _x41_1     ;load x(n-1) from  RAM
    lds  R21, _x41_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B413       ;load b3 from RAM
    lds  R23, _B413+1 
    lds  R20, _x41_2     ;load x(n-2) from  RAM
    lds  R21, _x41_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A412       ;load -a2 from RAM
    lds  R23, _A412+1 
    lds  R20, _y41_1     ;load y(n-1) from  RAM
    lds  R21, _y41_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A413       ;load -a3 from RAM
    lds  R23, _A413+1 
    lds  R20, _y41_2     ;load y(n-2) from  RAM
    lds  R21, _y41_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x41_1     ;load x(n-1) from  RAM
    lds  R21, _x41_1+1
    sts  _x41_2, r20      ;store x(n-2) to  RAM
    sts  _x41_2+1, R21 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    sts  _x41_1, r20     ;store x(n-1) to  RAM
    sts  _x41_1+1, R21 
    lds  R20, _y41_1     ;load y(n-1) from  RAM
    lds  R21, _y41_1+1
    sts  _y41_2, R20     ;store y(n-2) to  RAM
    sts  _y41_2+1, R21 
    sts  _y41_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y41_1+1, r31  
    
      
  ;*************************Second 2nd order IIR filter.**************
    lds  R22, _B421       ;load b1 from RAM
    lds  R23, _B421+1 
    lds   R20, _y41_1         ;load input parameter xx from stack
    lds  R21, _y41_1+1
    clr r30
    clr r31
    mult_acc            ; b1*xx  
    
    lds  R22, _B422       ;load b2 from RAM
    lds  R23, _B422+1 
    lds  R20, _x42_1     ;load x(n-1) from  RAM
    lds  R21, _x42_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B423       ;load b3 from RAM
    lds  R23, _B423+1 
    lds  R20, _x42_2     ;load x(n-2) from  RAM
    lds  R21, _x42_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A422       ;load -a2 from RAM
    lds  R23, _A422+1 
    lds  R20, _y42_1     ;load y(n-1) from  RAM
    lds  R21, _y42_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A423       ;load -a3 from RAM
    lds  R23, _A423+1 
    lds  R20, _y42_2     ;load y(n-2) from  RAM
    lds  R21, _y42_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x42_1     ;load x(n-1) from  RAM
    lds  R21, _x42_1+1
    sts  _x42_2, r20      ;store x(n-2) to  RAM
    sts  _x42_2+1, R21 
    lds   R20, _y41_1         ;load input parameter xx from stack
    lds  R21, _y41_1+1
    sts  _x42_1, r20     ;store x(n-1) to  RAM
    sts  _x42_1+1, R21 
    lds  R20, _y42_1     ;load y(n-1) from  RAM
    lds  R21, _y42_1+1
    sts  _y42_2, R20     ;store y(n-2) to  RAM
    sts  _y42_2+1, R21 
    sts  _y42_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y42_1+1, r31  
;*************************End of Second 2nd order IIR filter.************** 
       ; Adding new stuff
    ; multipling filter output w/ gain then 
    ; squaring the output of multiplier 
    mov r23,r31
    mov r22,r30 
    clr r30
    clr r31
    lds r20, _G4
    lds r21,_G4+1
    
     muls r23, r21		   ; (signed)ah * (signed)bh
	add	r31, r0
	mul	r22, r20		   ; al * bl
	add	r24, r0
	adc	r30, r1
	adc	r31, r27
	mulsu r23, r20		; (signed)ah * bl
	add	r30, r0
	adc	r31, r1
	mulsu	r21, r22		; (signed)bh * al
	add	r30, r0
	adc	r31, r1 
      mov r23,r31
      mov r22,r30
    ;Here multiplier with gain finishes
    ;Now start squaring y variable 
    
    lds r30,_y4
    lds r31,_y4+1
    clr r24
    
    muls r23,r23
    add  r31,r0
    mul  r22,r22
    add  r24,r0
    adc  r30,r1
    adc  r31,r27
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1    
    
    pop r21             ;restore parameter regs
    pop r20
	
    #endasm 
	
end  

int filter5(int xx)
// xx is the current input signal sample
// returns the current filtered output sample
begin
    #asm
   
    push r20    ;save parameter regs
    push r21
    
    clr r27             ;permanent zero
    clr r24             ;clear 24 bit result reg; msb to lsb => r31:r30:r24
    clr r30      
    clr r31
    
    lds  R22, _B511       ;load b1 from RAM
    lds  R23, _B511+1 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    mult_acc            ; b1*xx  
    
    lds  R22, _B512       ;load b2 from RAM
    lds  R23, _B512+1 
    lds  R20, _x51_1     ;load x(n-1) from  RAM
    lds  R21, _x51_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B513       ;load b3 from RAM
    lds  R23, _B513+1 
    lds  R20, _x51_2     ;load x(n-2) from  RAM
    lds  R21, _x51_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A512       ;load -a2 from RAM
    lds  R23, _A512+1 
    lds  R20, _y51_1     ;load y(n-1) from  RAM
    lds  R21, _y51_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A513       ;load -a3 from RAM
    lds  R23, _A513+1 
    lds  R20, _y51_2     ;load y(n-2) from  RAM
    lds  R21, _y51_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x51_1     ;load x(n-1) from  RAM
    lds  R21, _x51_1+1
    sts  _x51_2, r20      ;store x(n-2) to  RAM
    sts  _x51_2+1, R21 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    sts  _x51_1, r20     ;store x(n-1) to  RAM
    sts  _x51_1+1, R21 
    lds  R20, _y51_1     ;load y(n-1) from  RAM
    lds  R21, _y51_1+1
    sts  _y51_2, R20     ;store y(n-2) to  RAM
    sts  _y51_2+1, R21 
    sts  _y51_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y51_1+1, r31  
     
  ;*************************Second 2nd order IIR filter.**************
    lds  R22, _B521       ;load b1 from RAM
    lds  R23, _B521+1 
    lds   R20, _y51_1         ;load input parameter xx from stack
    lds  R21, _y51_1+1
    clr r30
    clr r31
    mult_acc            ; b1*xx  
    
    lds  R22, _B522       ;load b2 from RAM
    lds  R23, _B522+1 
    lds  R20, _x52_1     ;load x(n-1) from  RAM
    lds  R21, _x52_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B523       ;load b3 from RAM
    lds  R23, _B523+1 
    lds  R20, _x52_2     ;load x(n-2) from  RAM
    lds  R21, _x52_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A522       ;load -a2 from RAM
    lds  R23, _A522+1 
    lds  R20, _y52_1     ;load y(n-1) from  RAM
    lds  R21, _y52_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A523       ;load -a3 from RAM
    lds  R23, _A523+1 
    lds  R20, _y52_2     ;load y(n-2) from  RAM
    lds  R21, _y52_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x52_1     ;load x(n-1) from  RAM
    lds  R21, _x52_1+1
    sts  _x52_2, r20      ;store x(n-2) to  RAM
    sts  _x52_2+1, R21 
    lds   R20, _y51_1         ;load input parameter xx from stack
    lds  R21, _y51_1+1
    sts  _x52_1, r20     ;store x(n-1) to  RAM
    sts  _x52_1+1, R21 
    lds  R20, _y52_1     ;load y(n-1) from  RAM
    lds  R21, _y52_1+1
    sts  _y52_2, R20     ;store y(n-2) to  RAM
    sts  _y52_2+1, R21 
    sts  _y52_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y52_1+1, r31 
    ; Adding new stuff
    ; multipling filter output w/ gain then 
    ; squaring the output of multiplier 
    mov r23,r31
    mov r22,r30
    clr r30
    clr r31
    lds r20, _G5
    lds r21,_G5+1
    
     muls r23, r21		   ; (signed)ah * (signed)bh
	add	r31, r0
	mul	r22, r20		   ; al * bl
	add	r24, r0
	adc	r30, r1
	adc	r31, r27
	mulsu r23, r20		; (signed)ah * bl
	add	r30, r0
	adc	r31, r1
	mulsu	r21, r22		; (signed)bh * al
	add	r30, r0
	adc	r31, r1 
      mov r23,r31
      mov r22,r30
    ;Here multiplier with gain finishes
    ;Now start squaring y variable 
    
    lds r30,_y5
    lds r31,_y5+1
    clr r24
    
    muls r23,r23
    add  r31,r0
    mul  r22,r22
    add  r24,r0
    adc  r30,r1
    adc  r31,r27
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1  
    
     
;*************************End of Second 2nd order IIR filter.************** 
      
  
	
    pop r21             ;restore parameter regs
    pop r20
	
    #endasm 
	
end

int filter6(int xx)
// xx is the current input signal sample
// returns the current filtered output sample
begin
    #asm
   
    push r20    ;save parameter regs
    push r21
    
    clr r27             ;permanent zero
    clr r24             ;clear 24 bit result reg; msb to lsb => r31:r30:r24
    clr r30      
    clr r31
    
    lds  R22, _B611       ;load b1 from RAM
    lds  R23, _B611+1 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    mult_acc            ; b1*xx  
    
    lds  R22, _B612       ;load b2 from RAM
    lds  R23, _B612+1 
    lds  R20, _x61_1     ;load x(n-1) from  RAM
    lds  R21, _x61_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B613       ;load b3 from RAM
    lds  R23, _B613+1 
    lds  R20, _x61_2     ;load x(n-2) from  RAM
    lds  R21, _x61_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A612       ;load -a2 from RAM
    lds  R23, _A612+1 
    lds  R20, _y61_1     ;load y(n-1) from  RAM
    lds  R21, _y61_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A613       ;load -a3 from RAM
    lds  R23, _A613+1 
    lds  R20, _y61_2     ;load y(n-2) from  RAM
    lds  R21, _y61_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x61_1     ;load x(n-1) from  RAM
    lds  R21, _x61_1+1
    sts  _x61_2, r20      ;store x(n-2) to  RAM
    sts  _x61_2+1, R21 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    sts  _x61_1, r20     ;store x(n-1) to  RAM
    sts  _x61_1+1, R21 
    lds  R20, _y61_1     ;load y(n-1) from  RAM
    lds  R21, _y61_1+1
    sts  _y61_2, R20     ;store y(n-2) to  RAM
    sts  _y61_2+1, R21 
    sts  _y61_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y61_1+1, r31  
     
  ;*************************Second 2nd order IIR filter.**************
    lds  R22, _B621       ;load b1 from RAM
    lds  R23, _B621+1 
    lds   R20, _y61_1         ;load input parameter xx from stack
    lds  R21, _y61_1+1
    clr r30
    clr r31
    mult_acc            ; b1*xx  
    
    lds  R22, _B622       ;load b2 from RAM
    lds  R23, _B622+1 
    lds  R20, _x62_1     ;load x(n-1) from  RAM
    lds  R21, _x62_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B623       ;load b3 from RAM
    lds  R23, _B623+1 
    lds  R20, _x62_2     ;load x(n-2) from  RAM
    lds  R21, _x62_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A622       ;load -a2 from RAM
    lds  R23, _A622+1 
    lds  R20, _y62_1     ;load y(n-1) from  RAM
    lds  R21, _y62_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A623       ;load -a3 from RAM
    lds  R23, _A623+1 
    lds  R20, _y62_2     ;load y(n-2) from  RAM
    lds  R21, _y62_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x62_1     ;load x(n-1) from  RAM
    lds  R21, _x62_1+1
    sts  _x62_2, r20      ;store x(n-2) to  RAM
    sts  _x62_2+1, R21 
    lds   R20, _y61_1         ;load input parameter xx from stack
    lds  R21, _y61_1+1
    sts  _x62_1, r20     ;store x(n-1) to  RAM
    sts  _x62_1+1, R21 
    lds  R20, _y62_1     ;load y(n-1) from  RAM
    lds  R21, _y62_1+1
    sts  _y62_2, R20     ;store y(n-2) to  RAM
    sts  _y62_2+1, R21 
    sts  _y62_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y62_1+1, r31 
    ; Adding new stuff
    ; multipling filter output w/ gain then 
    ; squaring the output of multiplier 
    mov r23,r31
    mov r22,r30
    clr r30
    clr r31
    lds r20, _G6
    lds r21,_G6+1
    
     muls r23, r21		   ; (signed)ah * (signed)bh
	add	r31, r0
	mul	r22, r20		   ; al * bl
	add	r24, r0
	adc	r30, r1
	adc	r31, r27
	mulsu r23, r20		; (signed)ah * bl
	add	r30, r0
	adc	r31, r1
	mulsu	r21, r22		; (signed)bh * al
	add	r30, r0
	adc	r31, r1 
	
	
      mov r23,r31
      mov r22,r30
    ;Here multiplier with gain finishes
    ;Now start squaring y variable 
    
    lds r30,_y6
    lds r31,_y6+1
    clr r24
    
    muls r23,r23
    add  r31,r0
    mul  r22,r22
    add  r24,r0
    adc  r30,r1
    adc  r31,r27
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1  
    
     
;*************************End of Second 2nd order IIR filter.************** 
      
  
	
    pop r21             ;restore parameter regs
    pop r20
	
    #endasm 
	
end


int filter7(int xx)
// xx is the current input signal sample
// returns the current filtered output sample
begin
    #asm
   
    push r20    ;save parameter regs
    push r21
    
    clr r27             ;permanent zero
    clr r24             ;clear 24 bit result reg; msb to lsb => r31:r30:r24
    clr r30      
    clr r31
    
    lds  R22, _B711       ;load b1 from RAM
    lds  R23, _B711+1 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    mult_acc            ; b1*xx  
    
    lds  R22, _B712       ;load b2 from RAM
    lds  R23, _B712+1 
    lds  R20, _x71_1     ;load x(n-1) from  RAM
    lds  R21, _x71_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B713       ;load b3 from RAM
    lds  R23, _B713+1 
    lds  R20, _x71_2     ;load x(n-2) from  RAM
    lds  R21, _x71_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A712       ;load -a2 from RAM
    lds  R23, _A712+1 
    lds  R20, _y71_1     ;load y(n-1) from  RAM
    lds  R21, _y71_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A713       ;load -a3 from RAM
    lds  R23, _A713+1 
    lds  R20, _y71_2     ;load y(n-2) from  RAM
    lds  R21, _y71_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x71_1     ;load x(n-1) from  RAM
    lds  R21, _x71_1+1
    sts  _x71_2, r20      ;store x(n-2) to  RAM
    sts  _x71_2+1, R21 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    sts  _x71_1, r20     ;store x(n-1) to  RAM
    sts  _x71_1+1, R21 
    lds  R20, _y71_1     ;load y(n-1) from  RAM
    lds  R21, _y71_1+1
    sts  _y71_2, R20     ;store y(n-2) to  RAM
    sts  _y71_2+1, R21 
    sts  _y71_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y71_1+1, r31  
     
  ;*************************Second 2nd order IIR filter.**************
    lds  R22, _B721       ;load b1 from RAM
    lds  R23, _B721+1 
    lds   R20, _y71_1         ;load input parameter xx from stack
    lds  R21, _y71_1+1
    clr r30
    clr r31
    mult_acc            ; b1*xx  
    
    lds  R22, _B722       ;load b2 from RAM
    lds  R23, _B722+1 
    lds  R20, _x72_1     ;load x(n-1) from  RAM
    lds  R21, _x72_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B723       ;load b3 from RAM
    lds  R23, _B723+1 
    lds  R20, _x72_2     ;load x(n-2) from  RAM
    lds  R21, _x72_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A722       ;load -a2 from RAM
    lds  R23, _A722+1 
    lds  R20, _y72_1     ;load y(n-1) from  RAM
    lds  R21, _y72_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A723       ;load -a3 from RAM
    lds  R23, _A723+1 
    lds  R20, _y72_2     ;load y(n-2) from  RAM
    lds  R21, _y72_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x72_1     ;load x(n-1) from  RAM
    lds  R21, _x72_1+1
    sts  _x72_2, r20      ;store x(n-2) to  RAM
    sts  _x72_2+1, R21 
    lds   R20, _y71_1         ;load input parameter xx from stack
    lds  R21, _y71_1+1
    sts  _x72_1, r20     ;store x(n-1) to  RAM
    sts  _x72_1+1, R21 
    lds  R20, _y72_1     ;load y(n-1) from  RAM
    lds  R21, _y72_1+1
    sts  _y72_2, R20     ;store y(n-2) to  RAM
    sts  _y72_2+1, R21 
    sts  _y72_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y72_1+1, r31 
    ; Adding new stuff
    ; multipling filter output w/ gain then 
    ; squaring the output of multiplier 
    mov r23,r31
    mov r22,r30
    clr r30
    clr r31
    lds r20, _G5
    lds r21,_G5+1
    
     muls r23, r21		   ; (signed)ah * (signed)bh
	add	r31, r0
	mul	r22, r20		   ; al * bl
	add	r24, r0
	adc	r30, r1
	adc	r31, r27
	mulsu r23, r20		; (signed)ah * bl
	add	r30, r0
	adc	r31, r1
	mulsu	r21, r22		; (signed)bh * al
	add	r30, r0
	adc	r31, r1 
	sts _gainchk,r30 
	sts _gainchk+1,r31     ; this is just to test the output of second order filter.
	
      mov r23,r31
      mov r22,r30
    ;Here multiplier with gain finishes
    ;Now start squaring y variable 
    
    lds r30,_y7
    lds r31,_y7+1
    clr r24
    
    muls r23,r23
    add  r31,r0
    mul  r22,r22
    add  r24,r0
    adc  r30,r1
    adc  r31,r27
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1  
    
     
;*************************End of Second 2nd order IIR filter.************** 
      
  
	
    pop r21             ;restore parameter regs
    pop r20
	
    #endasm 
	
end 

int filter8(int xx)
// xx is the current input signal sample
// returns the current filtered output sample
begin
    #asm
   
    push r20    ;save parameter regs
    push r21
    
    clr r27             ;permanent zero
    clr r24             ;clear 24 bit result reg; msb to lsb => r31:r30:r24
    clr r30      
    clr r31
    
    lds  R22, _B811       ;load b1 from RAM
    lds  R23, _B811+1 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    mult_acc            ; b1*xx  
    
    lds  R22, _B812       ;load b2 from RAM
    lds  R23, _B812+1 
    lds  R20, _x81_1     ;load x(n-1) from  RAM
    lds  R21, _x81_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B813       ;load b3 from RAM
    lds  R23, _B813+1 
    lds  R20, _x81_2     ;load x(n-2) from  RAM
    lds  R21, _x81_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A812       ;load -a2 from RAM
    lds  R23, _A812+1 
    lds  R20, _y81_1     ;load y(n-1) from  RAM
    lds  R21, _y81_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A813       ;load -a3 from RAM
    lds  R23, _A813+1 
    lds  R20, _y81_2     ;load y(n-2) from  RAM
    lds  R21, _y81_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x81_1     ;load x(n-1) from  RAM
    lds  R21, _x81_1+1
    sts  _x81_2, r20      ;store x(n-2) to  RAM
    sts  _x81_2+1, R21 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    sts  _x81_1, r20     ;store x(n-1) to  RAM
    sts  _x81_1+1, R21 
    lds  R20, _y81_1     ;load y(n-1) from  RAM
    lds  R21, _y81_1+1
    sts  _y81_2, R20     ;store y(n-2) to  RAM
    sts  _y81_2+1, R21 
    sts  _y81_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y81_1+1, r31  
     
  ;*************************Second 2nd order IIR filter.**************
    lds  R22, _B821       ;load b1 from RAM
    lds  R23, _B821+1 
    lds   R20, _y81_1         ;load input parameter xx from stack
    lds  R21, _y81_1+1
    clr r30
    clr r31
    mult_acc            ; b1*xx  
    
    lds  R22, _B822       ;load b2 from RAM
    lds  R23, _B822+1 
    lds  R20, _x82_1     ;load x(n-1) from  RAM
    lds  R21, _x82_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B823       ;load b3 from RAM
    lds  R23, _B823+1 
    lds  R20, _x82_2     ;load x(n-2) from  RAM
    lds  R21, _x82_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A822       ;load -a2 from RAM
    lds  R23, _A822+1 
    lds  R20, _y82_1     ;load y(n-1) from  RAM
    lds  R21, _y82_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A823       ;load -a3 from RAM
    lds  R23, _A823+1 
    lds  R20, _y82_2     ;load y(n-2) from  RAM
    lds  R21, _y82_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x82_1     ;load x(n-1) from  RAM
    lds  R21, _x82_1+1
    sts  _x82_2, r20      ;store x(n-2) to  RAM
    sts  _x82_2+1, R21 
    lds   R20, _y81_1         ;load input parameter xx from stack
    lds  R21, _y81_1+1
    sts  _x82_1, r20     ;store x(n-1) to  RAM
    sts  _x82_1+1, R21 
    lds  R20, _y82_1     ;load y(n-1) from  RAM
    lds  R21, _y82_1+1
    sts  _y82_2, R20     ;store y(n-2) to  RAM
    sts  _y82_2+1, R21 
    sts  _y82_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y82_1+1, r31 
    ; Adding new stuff
    ; multipling filter output w/ gain then 
    ; squaring the output of multiplier 
    mov r23,r31
    mov r22,r30
    clr r30
    clr r31
    lds r20, _G8
    lds r21,_G8+1
    
     muls r23, r21		   ; (signed)ah * (signed)bh
	add	r31, r0
	mul	r22, r20		   ; al * bl
	add	r24, r0
	adc	r30, r1
	adc	r31, r27
	mulsu r23, r20		; (signed)ah * bl
	add	r30, r0
	adc	r31, r1
	mulsu	r21, r22		; (signed)bh * al
	add	r30, r0
	adc	r31, r1 
	
      mov r23,r31
      mov r22,r30
    ;Here multiplier with gain finishes
    ;Now start squaring y variable 
    
    lds r30,_y8
    lds r31,_y8+1
    clr r24
    
    muls r23,r23
    add  r31,r0
    mul  r22,r22
    add  r24,r0
    adc  r30,r1
    adc  r31,r27
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1  
    
     
;*************************End of Second 2nd order IIR filter.************** 
      
  
	
    pop r21             ;restore parameter regs
    pop r20
	
    #endasm 
	
end     
////
int filter9(int xx)
// xx is the current input signal sample
// returns the current filtered output sample
begin
    #asm
   
    push r20    ;save parameter regs
    push r21
    
    clr r27             ;permanent zero
    clr r24             ;clear 24 bit result reg; msb to lsb => r31:r30:r24
    clr r30      
    clr r31
    
    lds  R22, _B911       ;load b1 from RAM
    lds  R23, _B911+1 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    mult_acc            ; b1*xx  
    
    lds  R22, _B912       ;load b2 from RAM
    lds  R23, _B912+1 
    lds  R20, _x91_1     ;load x(n-1) from  RAM
    lds  R21, _x91_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B913       ;load b3 from RAM
    lds  R23, _B913+1 
    lds  R20, _x91_2     ;load x(n-2) from  RAM
    lds  R21, _x91_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A912       ;load -a2 from RAM
    lds  R23, _A912+1 
    lds  R20, _y91_1     ;load y(n-1) from  RAM
    lds  R21, _y91_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A913       ;load -a3 from RAM
    lds  R23, _A913+1 
    lds  R20, _y91_2     ;load y(n-2) from  RAM
    lds  R21, _y91_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x91_1     ;load x(n-1) from  RAM
    lds  R21, _x91_1+1
    sts  _x91_2, r20      ;store x(n-2) to  RAM
    sts  _x91_2+1, R21 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    sts  _x91_1, r20     ;store x(n-1) to  RAM
    sts  _x91_1+1, R21 
    lds  R20, _y91_1     ;load y(n-1) from  RAM
    lds  R21, _y91_1+1
    sts  _y91_2, R20     ;store y(n-2) to  RAM
    sts  _y91_2+1, R21 
    sts  _y91_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y91_1+1, r31  
     
  ;*************************Second 2nd order IIR filter.**************
    lds  R22, _B921       ;load b1 from RAM
    lds  R23, _B921+1 
    lds   R20, _y91_1         ;load input parameter xx from stack
    lds  R21, _y91_1+1
    clr r30
    clr r31
    mult_acc            ; b1*xx  
    
    lds  R22, _B922       ;load b2 from RAM
    lds  R23, _B922+1 
    lds  R20, _x92_1     ;load x(n-1) from  RAM
    lds  R21, _x92_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B923       ;load b3 from RAM
    lds  R23, _B923+1 
    lds  R20, _x92_2     ;load x(n-2) from  RAM
    lds  R21, _x92_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A922       ;load -a2 from RAM
    lds  R23, _A922+1 
    lds  R20, _y92_1     ;load y(n-1) from  RAM
    lds  R21, _y92_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A923       ;load -a3 from RAM
    lds  R23, _A923+1 
    lds  R20, _y92_2     ;load y(n-2) from  RAM
    lds  R21, _y92_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x92_1     ;load x(n-1) from  RAM
    lds  R21, _x92_1+1
    sts  _x92_2, r20      ;store x(n-2) to  RAM
    sts  _x92_2+1, R21 
    lds   R20, _y91_1         ;load input parameter xx from stack
    lds  R21, _y91_1+1
    sts  _x92_1, r20     ;store x(n-1) to  RAM
    sts  _x92_1+1, R21 
    lds  R20, _y92_1     ;load y(n-1) from  RAM
    lds  R21, _y92_1+1
    sts  _y92_2, R20     ;store y(n-2) to  RAM
    sts  _y92_2+1, R21 
    sts  _y92_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y92_1+1, r31 
    ; Adding new stuff
    ; multipling filter output w/ gain then 
    ; squaring the output of multiplier 
    mov r23,r31
    mov r22,r30
    clr r30
    clr r31
    lds r20, _G9
    lds r21,_G9+1
    
     muls r23, r21		   ; (signed)ah * (signed)bh
	add	r31, r0
	mul	r22, r20		   ; al * bl
	add	r24, r0
	adc	r30, r1
	adc	r31, r27
	mulsu r23, r20		; (signed)ah * bl
	add	r30, r0
	adc	r31, r1
	mulsu	r21, r22		; (signed)bh * al
	add	r30, r0
	adc	r31, r1 
	
      mov r23,r31
      mov r22,r30
    ;Here multiplier with gain finishes
    ;Now start squaring y variable 
    
    lds r30,_y9
    lds r31,_y9+1
    clr r24
    
    muls r23,r23
    add  r31,r0
    mul  r22,r22
    add  r24,r0
    adc  r30,r1
    adc  r31,r27
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1  
    
     
;*************************End of Second 2nd order IIR filter.************** 
      
  
	
    pop r21             ;restore parameter regs
    pop r20
	
    #endasm 
	
end  
///
int filter10(int xx)
// xx is the current input signal sample
// returns the current filtered output sample
begin
    #asm
   
    push r20    ;save parameter regs
    push r21
    
    clr r27             ;permanent zero
    clr r24             ;clear 24 bit result reg; msb to lsb => r31:r30:r24
    clr r30      
    clr r31
    
    lds  R22, _B1011       ;load b1 from RAM
    lds  R23, _B1011+1 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    mult_acc            ; b1*xx  
    
    lds  R22, _B1012       ;load b2 from RAM
    lds  R23, _B1012+1 
    lds  R20, _x101_1     ;load x(n-1) from  RAM
    lds  R21, _x101_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B1013       ;load b3 from RAM
    lds  R23, _B1013+1 
    lds  R20, _x101_2     ;load x(n-2) from  RAM
    lds  R21, _x101_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A1012       ;load -a2 from RAM
    lds  R23, _A1012+1 
    lds  R20, _y101_1     ;load y(n-1) from  RAM
    lds  R21, _y101_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A1013       ;load -a3 from RAM
    lds  R23, _A1013+1 
    lds  R20, _y101_2     ;load y(n-2) from  RAM
    lds  R21, _y101_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x101_1     ;load x(n-1) from  RAM
    lds  R21, _x101_1+1
    sts  _x101_2, r20      ;store x(n-2) to  RAM
    sts  _x101_2+1, R21 
    ld   R20, Y         ;load input parameter xx from stack
    ldd  R21, Y+1
    sts  _x101_1, r20     ;store x(n-1) to  RAM
    sts  _x101_1+1, R21 
    lds  R20, _y101_1     ;load y(n-1) from  RAM
    lds  R21, _y101_1+1
    sts  _y101_2, R20     ;store y(n-2) to  RAM
    sts  _y101_2+1, R21 
    sts  _y101_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y101_1+1, r31  
     
  ;*************************Second 2nd order IIR filter.**************
    lds  R22, _B1021       ;load b1 from RAM
    lds  R23, _B1021+1 
    lds   R20, _y101_1         ;load input parameter xx from stack
    lds  R21, _y101_1+1
    clr r30
    clr r31
    mult_acc            ; b1*xx  
    
    lds  R22, _B1022       ;load b2 from RAM
    lds  R23, _B1022+1 
    lds  R20, _x102_1     ;load x(n-1) from  RAM
    lds  R21, _x102_1+1
    mult_acc            ; b2*x(n-1) 
    
    lds  R22, _B1023       ;load b3 from RAM
    lds  R23, _B1023+1 
    lds  R20, _x102_2     ;load x(n-2) from  RAM
    lds  R21, _x102_2+1
    mult_acc            ; b3*x(n-2)
    
    lds  R22, _A1022       ;load -a2 from RAM
    lds  R23, _A1022+1 
    lds  R20, _y102_1     ;load y(n-1) from  RAM
    lds  R21, _y102_1+1
    mult_acc            ; -a2*y(n-1)  
    
    lds  R22, _A1023       ;load -a3 from RAM
    lds  R23, _A1023+1 
    lds  R20, _y102_2     ;load y(n-2) from  RAM
    lds  R21, _y102_2+1
    mult_acc            ; -a3*y(n-2)
    
    lds  R20, _x102_1     ;load x(n-1) from  RAM
    lds  R21, _x102_1+1
    sts  _x102_2, r20      ;store x(n-2) to  RAM
    sts  _x102_2+1, R21 
    lds   R20, _y101_1         ;load input parameter xx from stack
    lds  R21, _y101_1+1
    sts  _x102_1, r20     ;store x(n-1) to  RAM
    sts  _x102_1+1, R21 
    lds  R20, _y102_1     ;load y(n-1) from  RAM
    lds  R21, _y102_1+1
    sts  _y102_2, R20     ;store y(n-2) to  RAM
    sts  _y102_2+1, R21 
    sts  _y102_1, r30     ;store new output as y(n-1) to  RAM
    sts  _y102_1+1, r31 
    ; Adding new stuff
    ; multipling filter output w/ gain then 
    ; squaring the output of multiplier 
    mov r23,r31
    mov r22,r30
    clr r30
    clr r31
    lds r20, _G10
    lds r21,_G10+1
    
     muls r23, r21		   ; (signed)ah * (signed)bh
	add	r31, r0
	mul	r22, r20		   ; al * bl
	add	r24, r0
	adc	r30, r1
	adc	r31, r27
	mulsu r23, r20		; (signed)ah * bl
	add	r30, r0
	adc	r31, r1
	mulsu	r21, r22		; (signed)bh * al
	add	r30, r0
	adc	r31, r1 
	
	
      mov r23,r31
      mov r22,r30
    ;Here multiplier with gain finishes
    ;Now start squaring y variable 
    
    lds r30,_y10
    lds r31,_y10+1
    clr r24
    
    muls r23,r23
    add  r31,r0
    mul  r22,r22
    add  r24,r0
    adc  r30,r1
    adc  r31,r27
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1
    mulsu r23,r22
    add  r30,r0
    adc  r31,r1                
     
;*************************End of Second 2nd order IIR filter.**************     
      pop r21             ;restore parameter regs
    pop r20
	
    #endasm 
	
end




//~~~~~~~~~~~~~~~~~~~~~~~note:  need to include pragma warn
int multfix(int a,int b)
begin
    #asm
    push r20
    push r21
   
    LDD  R22,Y+2  ;load lsb a
	LDD  R23,Y+3  ; load msb a	
	LD   R20,Y    ;load lsb b
	LDD  R21,Y+1  ;load msb b
	
	muls	r23, r21		; (signed)ah * (signed)bh
	mov		r31, r0         ;
	mul		r22, r20		; al * bl
	mov     r30, r1         ;
	                        ;mov     r16, r0
	mulsu	r23, r20		; (signed)ah * bl
	add		r30, r0         ;
	adc		r31, r1         ;
	mulsu	r21, r22		; (signed)bh * al
	add		r30, r0         ;
	adc		r31, r1         ;
	
	pop r21
	pop r20
    #endasm
end
//==Fixed Mult and Accumulate============================// return = a*b + c ;;  fixed format
int macfix(int a, int b, int c)
begin
    //r31:r30:r24 += r23:r22 * r21:r20 
    //
    #asm      
    push r20
    push r21
    
        LDD  R22,Y+4  ;load a
	LDD  R23,Y+5 
	LDD  R20,Y+2  ;load b
	LDD  R21,Y+3
	LD   R30,Y    ;load c to lsb result
	LDD  R31,Y+1  ;and msb result
	clr  r24      ;low order byte
	clr  r27      ;permanent zero
	
    ;mac operation 
        muls r23, r21		   ; (signed)ah * (signed)bh
	add	r31, r0
	mul	r22, r20		   ; al * bl
	add	r24, r0
	adc	r30, r1
	adc	r31, r27
	mulsu r23, r20		; (signed)ah * bl
	add	r30, r0
	adc	r31, r1
	mulsu	r21, r22		; (signed)bh * al
	add	r30, r0
	adc	r31, r1
	;end mac operation
	
	pop r21
	pop r20
    #endasm
 end
 
 
 void main(void){

    initialize(); // open files (if testing on workstation), 
    while(1)
    begin 
        if (PINB.4 == 0){
                minword = go;  //go
                //printf("\nGo");        
        } 
        else if (PINB.5 == 0){
                minword = left; // left
				//printf("\nLeft");
        }
        else if (PINB.6 == 0){
                PORTD.7 = 0;
                 minword = right; //right
                 //printf("\nRight");
        }
        else if (PINB.7 == 0){ 
                 minword = back;   //back
                 //printf("\nBack");
        }
        
        if((PINB.4 == 0) || (PINB.5 == 0) || (PINB.6 == 0) || (PINB.7 == 0)){
                roboControl();
        }
        
        if (new_sample == 1) {
            get_sample(); // read in a word sample (assume framing is done by low level routine)
                            //analyze();  
            if(WORDFOUND == 1)
            begin 
                input = (((int)Ain) -102);
                //filter1(input);
                y2 = filter2(input);
                y3 = filter3(input);
                y4 = filter4(input);
                y5 = filter5(input);
                y6 = filter6(input);
                y7 = filter7(input);
                y8 = filter8(input);
                y9 = filter9(input);
                // y10 = filter10((int)Ain);     
  	            samcnt++; 
  	            if(samcnt == 125){
    	            segcnt++;
    	            samcnt = 0; // Reset sample counter.
    	            j = 0;
    	            word[i][j++] = y1;
    	            word[i][j++] = y2;
    	            word[i][j++] = y3;
    	            word[i][j++] = y4;
    	            word[i][j++] = y5;
    	            word[i][j++] = y6;
    	            word[i][j++] = y7;
    	            word[i][j++] = y8;
    	            word[i][j++] = y9;
    	            word[i][j++] = y10; 	            
    	            y1 = 0; y2 = 0; y3 = 0; y4 = 0; y5 = 0;
					y6 = 0; y7 = 0; y8 = 0; y9 = 0; y10 = 0;    
    	            i++;
  	            } // samcnt
  	            if(segcnt == 16){  // only makes ADC 1/2 long instead full second (ifsegcnt 32)
    	             segcnt = 0;
    	             samcnt = 0;
    	             WORDFOUND = 0; 
    	             i = 0;
    	             j = 0;
    	             for(m = 0; m < 10; m++){
	                  for(l = 0; l < 16; l++){
      	             fingerprint[k] = word[l][m];
    	                  k++;
    	                }
    	             }
					 /*
					 /////////////////EXPERIMENT: ANALYSIS ---------------------------
						if(PINC & (1<<0)) { //(PINC == 0b10111111) {
			                printf("\r\nOLD and NEW Fingerprints: \r\n");
			                for(j=0; j <160; j++){
			                    printf(" %u - %u, \n\r",dic[minword][j],fingerprint[j]);                   
			                }
							j=0;
			            }
					 */
    	             k = 0;
    	             lookup();     	             
    	             roboControl();   // controls car
    	             printf("Result: Word %d\r\n\n",minword);   // To test which word is that
  	            }
	        end  // if statement

			new_sample = 0;   // Re initialize to 0 for next ADC conversion.
			
			
			/*
				// ---- START OF DICTIONARY TEMPLATE GENERATION CODE ----
				// Steps:
				// uncomment this code, choose a switch such as PINB.5 below
				// compile, program ATmega32 with the hex code, connect the RS-232 connector to UART,
				// run hyperterminal @ 9600 bps, no flow control. 
				// Pushing the chosen button (PINB.5), speak one of the 5 voice commands (say, front!)
				// you should see the 160 datapoints printed in HyperTerminal. Release the pushbutton
				// copy-paste the datapoints into <dict.h> file in the section commented '\\front'
				// the similar process is repeated for the remaining 4 voice commands.
				// now dict.h file will be ready. uncomment this section back to original again
				// recompile this layer1-speechrecog-mma7260q project
				// it should now respond to the new voice commands
				
				// this is just for outputting to hyperterm when data has been stored in above fashion. 
				if(PINB.5 == 0){		//<--- edit the desired pin to activate logging in hyperterminal
					z = 1;
				}         
				if (z == 1) {
					printf("\r\nFINGERPRINT: \r\n");
		 
					z = 0;
					for(j=0; j <160; j++){
						printf(" %u, \n\r",fingerprint[j]);                   
					}
					lcd_clear();
					lcd_gotoxy(0,0);
					lcd_putsf("SEE FINGERPRINT");
					lcd_gotoxy(0,1);
					lcd_putsf("in hyperterminal");
				}
				// ---- END OF DICTIONARY TEMPLATE GENERATION CODE ----
				
			*/
			
		}  //new_sample
    end // end while(1)
}
