/*****************************************************************************
*
* File: arc4.h
* 
* VoCoRoBo-10
*
* Sagun Man Singh, 2010
*
* Reference S. Brennen Ball, 2007
* 
* 
*****************************************************************************/

#ifndef _ARC4_H_
#define _ARC4_H_

////////////////////////////////////////////////////////////////////////////////////
// Function declarations
//
// Below are all function definitions contained in the library.  Please see
//  arc4.c for comments regarding the usage of each function.
////////////////////////////////////////////////////////////////////////////////////

void arc4_initialize_ksa(unsigned char * key, unsigned int keylen);
unsigned char arc4_get_prga_byte(void);
void arc4_encrypt(unsigned char * buffer, unsigned int bufflen);
void arc4_decrypt(unsigned char * buffer, unsigned int bufflen);

#endif //_ARC4_H_
