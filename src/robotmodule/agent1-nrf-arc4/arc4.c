/******************************************************************************
*
* File: arc4.c
* 
* 
* VoCoRoBo-10
*
* Sagun Man Singh, 2010
*
* Reference S. Brennen Ball, 2007
* 
* 
*****************************************************************************/

#include "arc4.h"

unsigned char arc4_s[256];	//S array
unsigned int arc4_a = 0;		//a array index
unsigned int arc4_b = 0;		//b array index


///////////////////////////////////////////////////////////////////////////////
// arc4_initialize_ksa(unsigned char * key, unsigned int keylen)
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

void arc4_initialize_ksa(unsigned char * key, unsigned int keylen)
{
	unsigned int i, j = 0;
	unsigned char temp;
	
    arc4_a = 0;
    arc4_b = 0;

    for (i = 0; i < 256; i++)
        arc4_s[i] = (unsigned char)i;

    for (i = 0; i < 256; i++)
    {
        j = (j + arc4_s[i] + key[i % keylen]) % 256;
        temp = arc4_s[i];
        arc4_s[i] = arc4_s[j];
        arc4_s[j] = temp;
    }
}


///////////////////////////////////////////////////////////////////////////////
// unsigned char arc4_get_prga_byte(void)
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

unsigned char arc4_get_prga_byte(void)
{
	unsigned char temp;
	
    arc4_a = (arc4_a + 1) % 256;
    arc4_b = (arc4_b + arc4_s[arc4_a]) % 256;

    temp = arc4_s[arc4_a];
    arc4_s[arc4_a] = arc4_s[arc4_b];
    arc4_s[arc4_b] = temp;

    return arc4_s[(arc4_s[arc4_a] + arc4_s[arc4_b]) % 256];
}


///////////////////////////////////////////////////////////////////////////////
// arc4_encrypt(unsigned char * buffer, unsigned int bufflen)
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

void arc4_encrypt(unsigned char * buffer, unsigned int bufflen)
{
	unsigned int c;
	
    for (c = 0; c < bufflen; c++)
        buffer[c] = (unsigned char)(buffer[c] ^ arc4_get_prga_byte());
}


///////////////////////////////////////////////////////////////////////////////
// void arc4_decrypt(unsigned char * buffer, unsigned int bufflen)
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

void arc4_decrypt(unsigned char * buffer, unsigned int bufflen)
{
    arc4_encrypt(buffer, bufflen);
}
