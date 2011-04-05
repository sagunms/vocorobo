/*****************************************************************************
*
* File: spi1.c
* 
* VoCoRoBo-10
*
* Sagun Man Singh, 2010
*
* Reference S. Brennen Ball, 2007
* 
* 
*****************************************************************************/

#include <spi.h>
#include "spi1.h"
#include <mega32.h>

//===========================================================
//SPI Functions (SAGUN)
void spi1_masterinit(void)
{
	// Set MOSI and SCK output, all others input

	/*	
	//correction: output [4:40 pm; Sun, Aug 29, 2010]    //input
	//Brennen the asshole!, he'd been misguiding me. CE pin should have been an OUTPUT, not INPUT (according to Tutorial 1)
	DD_SPI |= (1<<DD_SCK)|(1<<DD_MOSI)|(1<<DD_CSN);	//Configure Output Pins
	DD_SPI &=~ ((1<<DD_MISO)|(1<<DD_IRQ)|(1<<DD_CE));		//Configure Input Pins
	*/
	DD_SPI |= (1<<DD_SCK)|(1<<DD_MOSI)|(1<<DD_CSN)|(1<<DD_CE);	//Configure Output Pins
	DD_SPI &=~ ((1<<DD_MISO)|(1<<DD_IRQ));		//Configure Input Pins	
	PORT_SPI |= (1<<DD_CSN);	//set CSN Bit
	
	/*
	// Enable SPI, Master, set clock rate fck/16
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	
	// Enable SPI, Master, set clock rate fck/2 (maximum)
	SPCR = (1<<SPE)|(1<<MSTR);
	SPSR = (1<<SPI2X);	//double speed
	*/
	// Enable SPI, Master, set clock rate fck/8 (SPI2X = 1, SPR1 = 0, SPR0 = 1) SAGUN
	//Note: if FCLK = 16 MHz; Data rate = 16 mhz/8 = 2 Mbps	SAGUN
	/*
		SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
		SPSR = (1<<SPI2X);	//double speed
	*/
	//Change: Sunday, August 29, 2010; int 8 mhz/8 = 1 Mbps SAGUN
		SPCR = (1<<SPE)|(1<<MSTR);
		//SPI2X SPR1 SPR0 = 0 0 0 ==> fclk/4 = 8 mhz/4 = 2 mhz ~ 2 mbps
		SPCR &= ~((1<<SPR1)|(1<<SPR0));
		SPSR &= ~(1<<SPI2X);	
	
}

/*
void spi1_slaveinit(void)
{
	// Set MISO output, all others input
	DDR_SPI = (1<<DD_MISO);
	// Enable SPI
	SPCR = (1<<SPE);
}
*/

unsigned char spi1_send_read_byte(unsigned char byte)	//Dual function tx + rx in same function SAGUN
{
	/* Start transmission */
	SPDR = byte;            //cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
	return SPDR;	//SAGUN
}

/*
char SPI_SlaveReceive(void)
{
	// Wait for reception complete
	while(!(SPSR & (1<<SPIF)))
	;
	//Return data register
	return SPDR;
}
*/