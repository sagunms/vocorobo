/******************************************************************************
*
* File: arc4multi.c
* 
* VoCoRoBo-10
*
* Sagun Man Singh, 2010
* Thouroughly edited on Thu, Sep 2, 2010 | 4:32 PM
*
* Reference S. Brennen Ball, 2007
* 
* 
*****************************************************************************/

#include "arc4multi.h"		//SAGUN Sat, Jan 1, 2011 @ 11:50 PM [HAPPY NEW YEAR 2011]

unsigned char arc4_s1[256];	//S array	robot 1
unsigned char arc4_s2[256];	//S array	robot 2

unsigned int arc4_a1 = 0;		//a array index		robot 1
unsigned int arc4_a2 = 0;		//a array index		robot 2

unsigned int arc4_b1 = 0;		//b array index		robot 1
unsigned int arc4_b2 = 0;		//b array index		robot 2


///////////////////////////////////////////////////////////////////////////////
// arc4_initialize_ksa(unsigned char * key, unsigned int keylen, unsigned char robot)
//
// Description:
//  Execute the KSA on the S array.
//
// Parameters:
//  unsigned char * key - Array of bytes that holds the desired user key.
//  unsigned int keylen - Length of the key (should be <= 256).
// 
// Return value:
//  None
///////////////////////////////////////////////////////////////////////////////

void arc4_initialize_ksa(unsigned char * key, unsigned int keylen, unsigned char robot)
{
	unsigned int i, j = 0;
	unsigned char temp;
	
    arc4_a1 = 0;	arc4_a2 = 0;
    arc4_b1 = 0;	arc4_b2 = 0;
	
	if(robot == 0) {		//first robot chosen
		for (i = 0; i < 256; i++)
			arc4_s1[i] = (unsigned char)i;

		for (i = 0; i < 256; i++)
		{
			j = (j + arc4_s1[i] + key[i % keylen]) % 256;
			temp = arc4_s1[i];
			arc4_s1[i] = arc4_s1[j];
			arc4_s1[j] = temp;
		}
	}
	else if(robot == 1) {
		for (i = 0; i < 256; i++)
			arc4_s2[i] = (unsigned char)i;

		for (i = 0; i < 256; i++)
		{
			j = (j + arc4_s2[i] + key[i % keylen]) % 256;
			temp = arc4_s2[i];
			arc4_s2[i] = arc4_s2[j];
			arc4_s2[j] = temp;
		}
	}
}


///////////////////////////////////////////////////////////////////////////////
// unsigned char arc4_get_prga_byte(unsigned char robot)
//
// Description:
//  Get the next byte from the PRGA and update S.
//
// Parameters:
//  unsigned char * key - Array of bytes that holds the desired user key.
//  unsigned int keylen - Length of the key (should be <= 256).
// 
// Return value:
//  The next byte from the PRGA
///////////////////////////////////////////////////////////////////////////////

unsigned char arc4_get_prga_byte(unsigned char robot)
{
	unsigned char temp;
	
	if(robot == 0) {
		arc4_a1 = (arc4_a1 + 1) % 256;
		arc4_b1 = (arc4_b1 + arc4_s1[arc4_a1]) % 256;

		temp = arc4_s1[arc4_a1];
		arc4_s1[arc4_a1] = arc4_s1[arc4_b1];
		arc4_s1[arc4_b1] = temp;

		return arc4_s1[(arc4_s1[arc4_a1] + arc4_s1[arc4_b1]) % 256];
	}
	else if(robot == 1) {
		arc4_a2 = (arc4_a2 + 1) % 256;
		arc4_b2 = (arc4_b2 + arc4_s2[arc4_a2]) % 256;

		temp = arc4_s2[arc4_a2];
		arc4_s2[arc4_a2] = arc4_s2[arc4_b2];
		arc4_s2[arc4_b2] = temp;

		return arc4_s2[(arc4_s2[arc4_a2] + arc4_s2[arc4_b2]) % 256];	
	}
}


///////////////////////////////////////////////////////////////////////////////
// arc4_encrypt(unsigned char * buffer, unsigned int bufflen, unsigned char robot)
//
// Description:
//  Encrypt an array of bytes.
//
// Parameters:
//  unsigned char * buffer - Array of bytes that holds the data to be encrypted.
//   This data is overwritten in the algorithm with encrypted data.
//  unsigned int bufflen - Length of the buffer.
// 
// Return value:
//  None
///////////////////////////////////////////////////////////////////////////////

void arc4_encrypt(unsigned char * buffer, unsigned int bufflen, unsigned char robot)
{
	unsigned int c;
	
    for (c = 0; c < bufflen; c++)
        buffer[c] = (unsigned char)(buffer[c] ^ arc4_get_prga_byte(robot));
}


///////////////////////////////////////////////////////////////////////////////
// arc4_decrypt(unsigned char * buffer, unsigned int bufflen, unsigned char robot)
//
// Description:
//  Decrypt an array of bytes.
//
// Parameters:
//  unsigned char * buffer - Array of bytes that holds the data to be decrypted.
//   This data is overwritten in the algorithm with decrypted data.
//  unsigned int bufflen - Length of the buffer.
// 
// Return value:
//  None
///////////////////////////////////////////////////////////////////////////////

void arc4_decrypt(unsigned char * buffer, unsigned int bufflen, unsigned char robot)
{
    arc4_encrypt(buffer, bufflen, robot);
}
