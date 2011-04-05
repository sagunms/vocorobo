/*****************************************************************************
 VoCoRoBo: Remote Speech Recognition and Tilt-Sensing Multi-Robotic System
------------------------------------------------------------------------------
 Copyright © 2011 Sagun Man Singh Shrestha [Apache License v2. See LICENSE.TXT]
 [sagunms@gmail.com | http://sagunms.wordpress.com | http://sagunms.com.np]
 
 VoCoRoBo Team: Sagun Man Singh Shrestha, Labu Manandhar and Ritesh Bhattarai
 Kathmandu Engineering College, Tribhuvan University, Kathmandu, Nepal
******************************************************************************
 File: main.c 
 [ROBOT 1 | RX address: C2:C2:C2:C2:C3 (Pipe 2)]
 ------------
 This is the VoCoRoBo Robot Module 2. Please see documentation.
 Establishes wireless link between itself and the Control Module using nRF24L01+ 
 2.4 GHz transceiver via SPI port. Also uses ARC4 cryptography  to encrypt robot
 control commands as well as decrypt remote sensor data.
 Receives control bytes from Control Module to its Pipe 2 (C2:C2:C2:C2:C3) addr.
 and transmits bytes to Pipe 0 (E7:E7:E7:E7:E7) addr. of Control Module.
 ------------
 Embedded Platform: ATmega16 (F_CLK = 8 MHz)
 Interfacing Components: 
 nRF24L01+ (PD2-7), LEDs (PC0-7), L293D + two DC motors (PD0-3), etc...
******************************************************************************
 Started: Thu, Aug 12, 2010 @ 3:55 PM
 Last Updated: Sat, Jan 1, 2011 @ 10:30 AM
******************************************************************************
 Project References: 
 S. Brennen Ball, 2007. “Specializing in the NXP LPC2148 and Microchip PIC18F452 
	microcontrollers and the Nordic Semiconductor nRF24L01 2.4 GHz RF link”; 
	http://www.diyembedded.com  [Code Modified for ATmega]
 “Interfacing nRF2401 with SPI” (White Paper), Nordic Semiconductor. 
*****************************************************************************/

#include <mega32.h>
#include <stdio.h>
#include <delay.h>
#include "nrf24l01.h" 
#include "arc4.h"

//USART Definitions
#define RXC 7
//Reserved pins:
#define selLeft 0
#define selRight 1

void Initialize(void); 
void InitializeIO(void); 

void delay_s(unsigned int n);		//delay n seconds [Tue, Aug 31, 2010 @ 1:03 AM]
void ToggleLED(void); //toggle the current state of the on-board LED

//void initialize(void);
void roboControl(unsigned char);

#define PAYLOAD_SIZE 	2
#define keylen	5 //length of the key
unsigned char key[keylen] = {'S', 'a', 'G', 'u', 'N'}; //bytes of the key

/////////////////////////////////////////////////////////////////////
 
void main(void) {
 	unsigned int i; //general counter variable
	
	//Declaration of Receiver
	unsigned char curseq = 0; //keeps the current expected packet number
	unsigned char data_rx[PAYLOAD_SIZE]; //array to hold the encrypted byte and packet count
	
	//Declaration of Transmitter
	//to do: sensor update function is yet to be added
	//unsigned char cur_letter = 100;		//temperature sensor reading to be transmitted
	unsigned char tx_addr[5]; //temporary variable to hold TX address for current pipe
	
	Initialize(); //initialize IO, UART, SPI, set up nRF24L01 as TX, execute KSA
	
	while(1)
	{
		//---------- FOR RECEPTION ---------------
		//========================================
		nrf_set_as_rx(true); //change the device to an RX to get the character back from the other 24L01
		printf("\n--- RECEPTION ----");
		//wait until a packet has been received		
		while(!(nrf_irq_pin_active() && nrf_irq_rx_dr_active()));
		printf("\n\tpacket received");
		nrf_irq_clear_all(); //clear all interrupts in the 24L01
		nrf_read_rx_payload(data_rx, PAYLOAD_SIZE); //get the payload into data_rx
		printf("\n\tNormal data_rx[0]=%c data_rx[1]=%d\n", data_rx[0], data_rx[1]);
		//catch up to the correct location in the S array by executing the PRGA
		if(data_rx[1] != curseq)
		{
			for(i = 0; i < data_rx[1] - curseq; i++)
				arc4_get_prga_byte();
			printf("\n\tNo. of Prga update = %d times", i);
		}
		
		arc4_decrypt(data_rx, 1); //decrypt the data_rx received
		printf("\n\tDecryp data_rx[0]=%c data_rx[1]=%d\n", data_rx[0], data_rx[1]);

		curseq = data_rx[1] + 1; //increase the value of the expected next packet
		printf("\ncurseq = %d", curseq);
		//delay_us(130); //wait for receiver to come from standby to RX
		delay_us(200);
		//---------- FOR TRANSMISSION ---------------
		//===========================================
		//setup TX address as DFLT RX address for pipe 0 (E7:E7:E7:E7:E7)
		tx_addr[0] = tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = nrf_RX_ADDR_P0_B0_DFLT_VAL;
		nrf_set_tx_addr(tx_addr, 5);
		
		nrf_set_as_tx(); //resume normal operation as a TX
		printf("\n--- TRANSMISSION ----");

		//data_rx[0] = cur_letter; //set up data_rx to be sent
		printf("\n\tNormal data_rx[0]=%c, data_rx[1]=%d", data_rx[0], data_rx[1]);
		arc4_encrypt(data_rx, 1); //encrypt the data_rx
		printf("\n\tEncryp data_rx[0]=%c, data_rx[1]=%d", data_rx[0], data_rx[1]);
		nrf_write_tx_payload(data_rx, PAYLOAD_SIZE, true); //transmit encrypted byte and counter
	
		//wait for the packet to be sent
		while(!(nrf_irq_pin_active() && nrf_irq_tx_ds_active()));
		printf("\n\tpacket sent");
		nrf_irq_clear_all(); //clear all interrupts in the 24L01

		data_rx[1]++; //increment the packet count
		
		delay_us(200); //wait for receiver to come from standby to RX
		///////////////////////////////////////
		ToggleLED(); //toggle the on-board LED as visual indication that the loop has completed
	}
}

//initialize routine
void Initialize(void) 
{ 
	InitializeIO(); //set up IO (directions and functions)
	//Initialize USART
	UCSRB = 0x18 ;    // UART to setup TX and Rx
	UBRRL = 51 ;     // 9600 Baud Rate for internal 8mhz F_CLK Mega32	|5:45; Sun, Aug 29, 2010| SAGUN
	spi1_masterinit(); //SAGUN Initialize SPI port
	nrf_initialize(nrf_CONFIG_DFLT_VAL | nrf_CONFIG_PWR_UP | nrf_CONFIG_PRIM_RX, //1 byte CRC, powered up, RX
				true,								//enable CE
				nrf_EN_AA_ENAA_NONE, 			//disable auto-ack on all pipes
				nrf_EN_RXADDR_ERX_P2, 			//enable receive on pipes 2		//SAGUN Sat, Jan 1, 2011 @ 10:30 AM
				nrf_SETUP_AW_DFLT_VAL, 		//5-byte addressing
				nrf_SETUP_RETR_DFLT_VAL, 	//not using auto-ack, so use DFLT
				nrf_RF_CH_DFLT_VAL, 		//RF channel 3
				nrf_RF_SETUP_DFLT_VAL,  	//2 Mbps, 0 dBm
				NULL, 								//DFLT receive addresses on all 6 pipes
				NULL, 								//""
				nrf_RX_ADDR_P2_DFLT_VAL, 	//""
				nrf_RX_ADDR_P3_DFLT_VAL, 	//""
				nrf_RX_ADDR_P4_DFLT_VAL, 	//""
				nrf_RX_ADDR_P5_DFLT_VAL, 	//""
				NULL, 								//DFLT TX address
				nrf_RX_PW_P0_DFLT_VAL, 						//18 byte payload width on pipe 0	//SAGUN Sat, Jan 1, 2011 @ 10:30 AM
				nrf_RX_PW_P1_DFLT_VAL, 		//""
				PAYLOAD_SIZE,		 		//""PIPE2 WIDTH
				nrf_RX_PW_P3_DFLT_VAL,  	//""
				nrf_RX_PW_P4_DFLT_VAL,  	//""
				nrf_RX_PW_P5_DFLT_VAL); 	//""
	
	arc4_initialize_ksa(key, keylen); //execute the KSA
	//printf("\nROBOT AGENT Initialized!\n");   //test
  //////////////////////////////FOR ROBOT CONTROL///////////////////
    DDRA = 0x00;
	PORTA = 0xff;
} 

//initialize IO pins
void InitializeIO(void) 
{ 	
   	int i;
	DDRC = 0xFF;    //blinky test SAGUN
	PORTC = 0xAA;
	for(i=0;i<10;i++){
	        PORTC = ~PORTC;
	        delay_ms(200);
	}
	PORTC = 0x0F;
}    

void delay_s(unsigned int n)		//SAGUN delay n seconds [Tue, Aug 31, 2010 @ 1:03 AM]
{
	int i;
	for(i=0;i<n;i++) {
		delay_ms(100);		//SAGUN 100 ms = 1 s
	}
}

//toggles on-board LED
void ToggleLED(void)
{
	PORTC = ~PORTC; //invert the bit that controls the LED
}

///////////////////////////////////////////  
void roboControl(unsigned char cmd2receive) {
	//Bit Manipulation		//Sun, Sept 12, 2010 @ 2:02 AM | KUCC Day 3
    if(cmd2receive == 'F'){ 
         //printf("\nfront");
         PORTD = 0b00000110;
    }
    else if(cmd2receive == 'L'){
        //printf("\nleft");
        PORTD = 0b00000011;	
    }
    else if(cmd2receive == 'R'){//else if(((~PINA & controlbyte_mask)) && right){
        //printf("\nright");
        PORTD = 0b00001100;
    } 
    else if(cmd2receive == 'S'){
        //printf("\nstop");
        PORTD = 0b00000000;
    }
    else if(cmd2receive == 'B'){
        //printf("\nback");
        PORTD = 0b00001001;
    } 
    else if(cmd2receive == 'U'){
        //printf("\nspeedup");
		PORTD = 0b00000000;			// STOP STATE FOR NOW
    }
    else if(cmd2receive == 'D'){
        //printf("\nspeeddown");
		PORTD = 0b00000000;			// STOP STATE FOR NOW 	//SAGUN | Wed, Sept. 15, 2010 @ 11:15 PM | LOCUS Exhibition Preparation (1 day left)
    }
    else {
        //printf("\nstop");
        PORTD = 0b00000000;
	}
}
