/*****************************************************************************
*
* File: spi1.h
* 
* VoCoRoBo-10
*
* Sagun Man Singh, 2010
*
* Reference S. Brennen Ball, 2007
* 
* 
*****************************************************************************/

#ifndef _SPI1_H_
#define _SPI1_H_

//SPI definitions (SAGUN)
//==========================================
//- SPI PORT and PIN definitions - 
#define DD_SPI DDRB
#define PORT_SPI PORTB  //debug SAGUN
//#define DD_SS 4  fr
//#define DD_MOSI 5
//#define DD_MISO 6
#define DD_SCK 7	//output
#define DD_MISO 6	//input
#define DD_MOSI 5	//output
#define DD_CSN 4	//output
#define DD_IRQ 3	//input
#define DD_CE 2		//correction: output [4:40 pm; Sun, Aug 29, 2010]    //input

//- SPI SPSR definitions -
#define SPIF 7
//#define WCOL 6
#define SPI2X 0
//SPCR
//#define SPIE 7
#define SPE 6
//#define DORD 5
#define MSTR 4
//#define CPOL 3
//#define CPHA 2
#define SPR1 1
#define SPR0 0
//===========================================

#include <spi.h>

void spi1_masterinit(void);		//SAGUN
//void spi1_slaveinit(void);	//SAGUN
unsigned char spi1_send_read_byte(unsigned char byte);

#endif //_SPI_H_
