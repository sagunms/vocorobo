/*****************************************************************************
 VoCoRoBo: Remote Speech Recognition and Tilt-Sensing Multi-Robotic System
------------------------------------------------------------------------------
 Copyright © 2011 Sagun Man Singh Shrestha [Apache License v2. See LICENSE.TXT]
 [sagunms@gmail.com | http://sagunms.wordpress.com | http://sagunms.com.np]
 
 VoCoRoBo Team: Sagun Man Singh Shrestha, Labu Manandhar and Ritesh Bhattarai
 Kathmandu Engineering College, Tribhuvan University, Kathmandu, Nepal
******************************************************************************
 File: main.c
 ------------
 This is the Layer 2 of VoCoRoBo Control Module. Please see documentation.
 Establishes wireless link between Control Module and the two Robot Modules
 using nRF24L01+ 2.4 GHz transceiver via SPI port. Also uses ARC4 cryptography
 to encrypt robot control commands as well as decrypt remote sensor data.
 Tranmits control bytes to Pipe 1 (C2:C2:C2:C2:C2) for remote Robot 1 and 
 Pipe 2 (C2:C2:C2:C2:C3) for Robot 2. The remote sensor data are received at
 Pipe 0 (E7:E7:E7:E7:E7) at the local nRF24L01.
 ------------
 Embedded Platform: ATmega16 (F_CLK = 8 MHz)
 Interfacing Components: 
 nRF24L01+ (PD2-7), LEDs (PC0-7), ATmega32 (PA0-2), MAX232 (PD0,1), etc...
******************************************************************************
 Started: Thu, Aug 12, 2010 at 3:55 PM
 Last Updated: Thursday, January 13, 2011 at 1:14 PM
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
#include "arc4multi.h"

//UART Definition
#define RXC 7
//Reserved pins:
#define selLeft 0
#define selRight 1

void Initialize(void); 
void InitializeIO(void); 

void delay_s(unsigned int n);		//delay n seconds

int toggleflag = 0;
void ToggleLED(void); //toggle the current state of the on-board LED

unsigned char roboControl(void);

#define PAYLOAD_SIZE 	2
#define keylen	5 //length of the key
unsigned char key[keylen] = {'S', 'a', 'G', 'u', 'N'}; //bytes of the key
unsigned char tx_addr[5]; //temporary variable to hold TX address for current pipe

// Functionality of the Robot
#define stop	0x00
#define front	0x01
#define back	0x02
#define left	0x03
#define right	0x04
#define spd_dn	0x06
#define spd_up	0x07

 /////////////////////////////////////////////////////////////////////

 void main(void) {
	unsigned int i, count;
	unsigned int uart_itr = 0;		//just to track number of loops made in the uart
	
	//Declaration of Transmitter
	unsigned char cur_letter1 = 'S'; //the letter we're going to send initially is STOP
	unsigned char cur_letter2 = 'S';
	unsigned char data_tx1[2], data_tx2[2];; //array to hold the encrypted byte and packet count
	
	//Declaration of Receiver
	unsigned char curseq1 = 0;
	unsigned char curseq2 = 0; 		//keeps the current expected packet number
	unsigned char txrx_match = 0;	//to indicate that the transmitted and the received character after ping test MATCHES
	
	//Initialize Transmitter
	data_tx1[1] = 0; //initialize packet count
	data_tx2[1] = 0;
	
	Initialize(); //initialize IO, UART, SPI, set up nRF24L01 as RX, execute KSA

	while(1)
	{	
	    //delay_s(1); //delay 1 second between each letter
		printf("\nLOOP");
		if(PIND.4 == 0 && PIND.5 == 1) {	//ROBOT AGENT1
			//---------- FOR TRANSMISSION ---------------
			//===========================================
			
			//setup TX address as DFLT RX address for pipe 1 (C2:C2:C2:C2:C2)
			tx_addr[0] = tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = nrf_RX_ADDR_P1_B0_DFLT_VAL;
			nrf_set_tx_addr(tx_addr, 5);
			
			nrf_set_as_tx(); //resume normal operation as a TX
			printf("\n--- TRANSMISSION - 1----");
			data_tx1[0] = cur_letter1; //set up data_tx to be sent
			printf("\n\tNormal data_tx1[0]=%c, data_tx1[1]=%d", data_tx1[0], data_tx1[1]);
			txrx_match = data_tx1[0]; //saturday, january 8, 2011
			arc4_encrypt(data_tx1, 1, 0); //encrypt the data_tx
			printf("\n\tEncryp data_tx1[0]=%c, data_tx1[1]=%d", data_tx1[0], data_tx1[1]);

			nrf_write_tx_payload(data_tx1, PAYLOAD_SIZE, true); //transmit encrypted byte and counter
			
			//wait for the packet to be sent
			while(!(nrf_irq_pin_active() && nrf_irq_tx_ds_active()));
			printf("\n\tpacket sent");
			nrf_irq_clear_all(); //clear all interrupts in the 24L01
			
			//go to the next character in the alphabet unless we're at z
			cur_letter1 = roboControl();
			printf("\n\tcur_letter1 = roboControl() = %c", cur_letter1);
			data_tx1[1]++; //increment the packet count
			//printf("\n\tcount = %c \n", data_tx1[1]);
			delay_us(130); //wait for receiver to come from standby to RX
			
			//---------- FOR RECEPTION ---------------
			//========================================
			nrf_set_as_rx(true); //change the device to an RX to get the character back from the other 24L01
			printf("\n--- RECEPTION - 1----");
			//wait a while to see if we get the data back (change the loop maximum and the lower if
			//  argument (should be loop maximum - 1) to lengthen or shorten this time frame
			for(count = 0; count < 20000; count++)
			{
				//check to see if the data has been received.  if so, get the data and exit the loop.
				//  if the loop is at its last count, assume the packet has been lost and set the data
				//  to go to the UART to "?".  If neither of these is true, keep looping.
				//printf("\tloopcount = %d", count);
				if((nrf_irq_pin_active() && nrf_irq_rx_dr_active()))
				{
					//get the payload of COUNT (1 byte) | SPEED (1 byte) | PADDING BITS (12 bytes) | LIGHT (2 bytes) | TEMPERATURE (2 bytes) = 18 bytes into data_tx1 for ROBOT 1 using nrf_read_rx_payload()
					//now check from which pipe the payload came from using nrf_get_rx_pipe_from_status();

					nrf_read_rx_payload(data_tx1, PAYLOAD_SIZE);
					printf("\n\tReceived data_tx1[0]=%c, data_tx1[1]=%d", data_tx1[0], data_tx1[1]);
					break;
				}
				//if loop is on its last iteration, assume packet has been lost.
				if(count == 19999) 
					data_tx1[0] = 0;
					//for(i = 0; i < PAYLOAD_SIZE; i++)       data_tx1[i] = 0;   //clear entire payload //get prga for ROBOT 1
					
			}
			printf("\tloopcount ENDED = %d", count);
			nrf_irq_clear_all(); //clear interrupts again
				
			//catch up to the correct location in the S array by executing the PRGA
			if(data_tx1[1] != curseq1)	
			{	
				for(i = 0; i < data_tx1[1] - curseq1; i++) {
					arc4_get_prga_byte(0);  //get prga for ROBOT 1
					
				}
				printf("\n\tNo. of Prga update = %d times", data_tx1[1] - curseq1);
			}
			
			arc4_decrypt(data_tx1, 1, 0); //decrypt the data received
			printf("\n\tDecryp data_tx1[0]=%c, data_tx1[1]=%d", data_tx1[0], data_tx1[1]);			
			if(data_tx1[0] == txrx_match) //saturday, january 8, 2011
				printf("\n\t(PING SUCCESS)");
			else
				printf("\n\t(PING FAILED)");
				
			curseq1 = data_tx1[1] + 1;
			printf("\n\tcurseq1 = %d", curseq1);
			delay_us(130); //wait for receiver to come from standby to RX
		}
		
		else if(PIND.4 == 1 && PIND.5 == 0) {	//ROBOT AGENT2
			//---------- FOR TRANSMISSION ---------------
			//===========================================
			//setup TX address as DFLT RX address for pipe 2 (C2:C2:C2:C2:C3)
			tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = nrf_RX_ADDR_P1_B0_DFLT_VAL;
			tx_addr[0] = nrf_RX_ADDR_P2_DFLT_VAL;
			nrf_set_tx_addr(tx_addr, 5);
			
			nrf_set_as_tx(); //resume normal operation as a TX
			printf("\n--- TRANSMISSION - 2----");

			data_tx2[0] = cur_letter2; //set up data_tx to be sent
			printf("\n\tNormal data_tx2[0]=%c, data_tx2[1]=%d", data_tx2[0], data_tx2[1]);
			txrx_match = data_tx2[0]; //saturday, january 8, 2011
			arc4_encrypt(data_tx2, 1, 1); //encrypt the data_tx
			printf("\n\tEncryp data_tx2[0]=%c, data_tx2[1]=%d", data_tx2[0], data_tx2[1]);
			nrf_write_tx_payload(data_tx2, PAYLOAD_SIZE, true); //transmit encrypted byte and counter
			
			//wait for the packet to be sent
			while(!(nrf_irq_pin_active() && nrf_irq_tx_ds_active()));
			printf("\n\tpacket sent");
			nrf_irq_clear_all(); //clear all interrupts in the 24L01
			
			//go to the next character in the alphabet unless we're at z
			cur_letter2 = roboControl();
			printf("\n\tcur_letter2 = roboControl() = %c", cur_letter1);
			data_tx2[1]++; //increment the packet count
			//printf("\n\tcount = %d \n", data_tx1[1]);
			delay_us(130);
			
			//---------- FOR RECEPTION ---------------
			//========================================		
			nrf_set_as_rx(true); //change the device to an RX to get the character back from the other 24L01
			printf("\n--- RECEPTION - 2----");

			//wait a while to see if we get the data back (change the loop maximum and the lower if
			//  argument (should be loop maximum - 1) to lengthen or shorten this time frame
			for(count = 0; count < 20000; count++)
			{
				//check to see if the data has been received.  if so, get the data and exit the loop.
				//  if the loop is at its last count, assume the packet has been lost. If neither of these is true, keep looping.
				//printf("\tloopcount = %d", count);
				if((nrf_irq_pin_active() && nrf_irq_rx_dr_active()))
				{
					nrf_read_rx_payload(data_tx2, PAYLOAD_SIZE);
					printf("\n\tReceived data_rx2[0]=%d data_rx2[1]=%d\n", data_tx2[0], data_tx2[1]);
					break;
				}
				//if loop is on its last iteration, assume packet has been lost.
				if(count == 19999) 
					data_tx2[0] = 0;
			}
			printf("\tloopcount ENDED = %d", count);
			nrf_irq_clear_all(); //clear interrupts again

			//catch up to the correct location in the S array by executing the PRGA
			if(data_tx2[1] != curseq2)	
			{
				for(i = 0; i < data_tx2[1] - curseq2; i++)
					arc4_get_prga_byte(1);  //get prga for ROBOT 1 //SAGUN Sun, Jan 2, 2011 @ 12:25 AM [1 DAY after NEW YEAR 2011]
			}
			
			arc4_decrypt(data_tx2, 1, 1); //decrypt the data received
			printf("\n\tDecryptd data_tx2[0]=%c, data_tx2[1]=%d", data_tx2[0], data_tx2[1]);
			if(data_tx2[0] == txrx_match) //saturday, january 8, 2011
				printf("\n\t(PING SUCCESS)");
			else
				printf("\n\t(PING FAILED)");
			curseq1 = data_tx2[1] + 1;
			printf("\n\tcurseq2 = %d", curseq2);
			delay_us(130); //wait for receiver to come from standby to RX			
		}
		
		if(data_tx1[0] == 100) {
			PORTC &= 0x0F;
			PORTC |= 0x90;	//1oo1 
		}
		else if(data_tx2[0] == 200) {
			PORTC &= 0x0F;		
			PORTC |= 0x60;	//o11o 
		}
		else {
			PORTC &= 0x0F;		
			PORTC |= 0x50;	//o101 
		}
		
		ToggleLED(); //toggle the on-board LED as visual indication that the loop has completed
	}
}


//initialize routine
void Initialize(void) 
{ 
	int i; 	//loop counter
	InitializeIO(); //set up IO (directions and functions)
	
	//Initialize USART
	UCSRB = 0x18 ;    // UART to setup TX and Rx
	UBRRL = 51 ;     // 9600 Baud Rate for internal 8mhz F_CLK Mega32	|5:45; Sun, Aug 29, 2010| SAGUN
	
	spi1_masterinit(); //SAGUN Initialize SPI port		//OpenSPI(SPI_FOSC_16, MODE_00, SMPMID); //open SPI1
	
	nrf_initialize(nrf_CONFIG_DFLT_VAL | nrf_CONFIG_PWR_UP | nrf_CONFIG_PRIM_RX, //1 byte CRC, powered up, RX
				true,						//enable CE
				nrf_EN_AA_ENAA_NONE, 		//disable auto-ack on all pipes
				nrf_EN_RXADDR_ERX_P0, 		//enable receive on pipes 0		//SAGUN Sat, Jan 1, 2011 @ 10:30 AM
				nrf_SETUP_AW_DFLT_VAL, 		//5-byte addressing
				nrf_SETUP_RETR_DFLT_VAL, 	//not using auto-ack, so use DFLT
				nrf_RF_CH_DFLT_VAL, 		//RF channel 3
				nrf_RF_SETUP_DFLT_VAL,  	//2 Mbps, 0 dBm
				NULL, 						//DFLT receive addresses on all 6 pipes
				NULL, 						//""
				nrf_RX_ADDR_P2_DFLT_VAL, 	//""
				nrf_RX_ADDR_P3_DFLT_VAL, 	//""
				nrf_RX_ADDR_P4_DFLT_VAL, 	//""
				nrf_RX_ADDR_P5_DFLT_VAL, 	//""
				NULL, 						//DFLT TX address
				PAYLOAD_SIZE, 				//18 byte payload width on pipe 0	//SAGUN Sat, Jan 1, 2011 @ 10:30 AM
				nrf_RX_PW_P1_DFLT_VAL, 		//""
				nrf_RX_PW_P2_DFLT_VAL, 		//""
				nrf_RX_PW_P3_DFLT_VAL,  	//""
				nrf_RX_PW_P4_DFLT_VAL,  	//""
				nrf_RX_PW_P5_DFLT_VAL); 	//""
				
	arc4_initialize_ksa(key, keylen, 0); 	//execute the KSA for ROBOT 1
	arc4_initialize_ksa(key, keylen, 1); 	//execute the KSA for ROBOT 2

    ////////////////////////////FOR ROBOT CONTROL///////////////////
	DDRA = 0x00;
	PORTA = 0xff; 
	DDRD &= 0b11001111;		//Making only PIND.4 and PIND.5 --> INPUTS
	PORTD |= 0b00110000;
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

void delay_s(unsigned int n)		//delay n seconds
{
	int i;
	for(i=0;i<n;i++) {
		delay_ms(100);		//100 ms = 1 s
	}
}

//toggles on-board LED
void ToggleLED(void)
{
	if(toggleflag == 0) {
		toggleflag = 1;
		PORTC &= 0xF0;
		PORTC |= 0x03;
	}
	else {
		toggleflag = 0;
		PORTC &= 0xF0;
		PORTC |= 0x0C;
	}
}

///////////////////////////////////////////  
unsigned char roboControl(void) {
	unsigned char cmd2transmit;	

//Bit Manipulation
    if(~PINA == front){
         //printf("\nfront");
		 cmd2transmit = 'F';	//to send 'F' which stands for FRONT over wireless link
    }
    else if(~PINA == left){
        //printf("\nleft");
		cmd2transmit = 'L';
    }
    else if(~PINA == right){
        //printf("\nright");
		cmd2transmit = 'R';
    } 
    else if(~PINA == stop){
        //printf("\nstop");
		cmd2transmit = 'S';
	}
    else if(~PINA == back){
        //printf("\nback");
		cmd2transmit = 'B';
    } 
    
    else if(~PINA == spd_up){
        //printf("\nspeedup");
		cmd2transmit = 'U';	
    }
    else if(~PINA == spd_dn){
        //printf("\nspeeddown");
		cmd2transmit = 'D';
    }
    else {
        //printf("\nstop");
		cmd2transmit = 'S';
	}
	return cmd2transmit;
}                                     
