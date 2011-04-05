/*****************************************************************************
*
* File: arc4multi.h
* 
* VoCoRoBo-10
*
* Sagun Man Singh, 2010
*
* Reference S. Brennen Ball, 2007
* 
* 
*****************************************************************************/

#ifndef _ARC4MULTI_H_		//SAGUN Sat, Jan 1, 2011 @ 11:50 PM [HAPPY NEW YEAR 2011]
#define _ARC4MULTI_H_

////////////////////////////////////////////////////////////////////////////////////
// Function declarations
//
// Below are all function definitions contained in the library.  Please see
//  arc4.c for comments regarding the usage of each function.
////////////////////////////////////////////////////////////////////////////////////

void arc4_initialize_ksa(unsigned char * key, unsigned int keylen, unsigned char robot);
unsigned char arc4_get_prga_byte(unsigned char robot);
void arc4_encrypt(unsigned char * buffer, unsigned int bufflen, unsigned char robot);
void arc4_decrypt(unsigned char * buffer, unsigned int bufflen, unsigned char robot);

#endif //_ARC4MULTI_H_
