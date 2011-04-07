VoCoRoBo: Remote Speech Recognition and Tilt-Sensing Multi-Robotic System
=========================================================================

VoCoRoBo stands for Voice Controlled RoBot. This work  is  based  on  the  implementation  of  real-time  speech  recognition  using DSP algorithms such as Chebyshev  IIR  filters, accelerometer for tilt-sensing and establishment of short-range  wireless  secure  link with ARC4  cipher,  all  using  low-cost  8-bit ATmega microcontrollers. The  robot  implements  a  simple  but  effective  algorithm  for  comparing  the spoken word  with a dictionary of fingerprints using a modified Euclidean distance calculation. It also includes the ability to securely control the navigation of multiple robots located at remote locations wirelessly from the Control Module and also gather the various environmental data collected by the Robot Modules and display them in the back to Control. Considering the time-critical algorithms actually requiring large computations as well as a variety of sensors interfaced in the system, this project can demonstrate how one can build an expansible multi-robotic system from cheap and ubiquitous electronics. 

The system is divided into two broad sub-subsystems: Control Module and Multi-Robot Module.  The Control Module is further divided into two layers: the topmost layer and the second layer.

# Control Module

The topmost layer of the control module consists of ATMega32, where speech recognition, MMA7260Q accelerometer sensing, output to 16x2 text LCD are handled. The 2nd layer consists of ATMega16 where the nRF24L01 wireless routine as well as encryption and decryption with ARC4 cipher are implemented. 

## Layer 1 [Folder: \src\controlmodule\layer1-speechrecog-mma7260q]

The Control Module is further divided into two layers: the topmost layer (Layer 1) and the second layer (Layer 2).

### File: vocorobo.c

* This is the Layer 1 of VoCoRoBo Control Module. Please see documentation and code comments.
* This code is the implementation of real-time speech recognition using eight 4th order digital Chebyshev2 IIR filters in software and creating voice fingerprints of spoken words (front, back, left, right and stop). Euclidean distance is used to compare  the spoken word with the dictionary to recognize the words.
* MMA7260Q 3-axis accelerometer connected to ADC are also interface for tilt-sensing.
* The two control modes viz. Speech Recognition and Tilt Sensing are selected using appropriate pushbutton combinations. When a command is recognized, the corresponding bit combinations are sent to the Layer 2 of the Control Module for wireless transmission and cryptography.

### File: dict.h

This is the dictionary template file to store the 160 data points (16 for each 10 filters) for each 5 voice commands (front, back, left, right and stop). Therefore, total datapoints stored is 160 x 5 = 800.
For recognizing the voice commands given by one person, the person should first uncomment the dictionary template generation code found at the last few lines of vocorobo.c file. Please follow the following steps to initialize your voice for the robot to recognize:
* Uncomment the commented dictionary template generation code, choose an input pin say, PINB.5 needed for triggering hyperterminal logging of datapoints for the spoken word
* Compile, program ATmega32 with the hex code, connect the RS-232 connector to UART,
* Run hyperterminal @ 9600 bps, no flow control (defaults)
* Pushing the chosen button (PINB.5), speak one of the 5 voice commands (say, front!)
* You should now see the 160 datapoints printed in HyperTerminal. Release the pushbutton
* Copy-paste the datapoints into <dict.h> file in the section commented '\\front'
* The similar process is repeated for the remaining 4 voice commands (back, left, right and stop).
* Now dict.h file will be ready so uncomment this section back to original code again.
* Recompile this layer1-speechrecog-mma7260q project and load the .hex file to ATMega32 again.
* It should now respond to the voice commands for that person.

## Layer 2 [Folder: \src\robotmodule\agent1-nrf-arc4 | \src\robotmodule\agent1-nrf-arc4]

### File: main.c and main.h

* This is the code for VoCoRoBo Robot Module 1 and 2. Please see documentation and code comments in the specific file.
* The bit combinations inputted to three bits of PINA are converted to the corresponding function control bytes to be sent out via wireless via SPI port are defined in this code.
* Establishes wireless link between itself and the Control Module using nRF24L01+ 2.4 GHz transceiver via SPI port. Also uses ARC4 cryptography to encrypt robot control commands as well as decrypt remote sensor data according to the private key stored in this code.
* Robot 1 receives control bytes from Control Module to RX address: C2:C2:C2:C2:C2 (Pipe1) and transmits bytes to Pipe 0 (E7:E7:E7:E7:E7) addr. of Control Module.
* Similarly, Robot 2 receives control bytes from Control Module to RX address: C2:C2:C2:C2:C3 (Pipe2) and transmits bytes to Pipe 0 (E7:E7:E7:E7:E7) addr. of Control Module.

### File: arc4multi.c & arc4multi.h

* These files form the library for the ARC4 cryptography routine. Please see documentation and code comments in the specific file.
* It initializes the key scheduling algorithm to initialize S variable with the specified private key, generates pseudorandom byte for both encryption and decryption.
* Separate variables are initialized for Robot 1 and Robot 2.

### File: nrf24l01.c & nrf24l01.h

* This is the nRF24L01 library containing the defines for all of the nRF24L01+ registers, fields within the registers, register default values, opcodes and functions to get the wireless link up and running. Please see documentation and code comments in the specific file.


# Robot Modules

It consists of two identical robots (A and B) which can be positioned at different locations, provided they are within the signal range of the Control Module.

### File: main.c and main.h

* This is similar to main file described in Layer 2. Additionally, it received function control byte received via wireless (SPI port) will produce corresponding bit combinations that are outputted to the H-bridge (PORTD) of ATMega16.

### File: arc4.c & arc4.h | nrf24l01.c & nrf24l01.h

* These files are similar as that described in Layer 2. the ARC4 cryptography routine has variables for only a single robot unlike in Layer2.


# Embedded Platform 

* Control Module Layer 1: ATmega32 (F_CLK = 16 MHz)
* Control Module Layer 2 and Robot Modules 1 and 2: ATmega16 (F_CLK = 8 MHz)

## Interfacing Components

* Control Module Layer 1: Mic + 500 gain amp (PA0), 16x2 LCD (PC), MMA7260Q (PA3-5), ATK200(PA0-2), MAX233A (PD0,1), Pushbuttons(PB4-7), 16MHz XTAL, etc.
* Control Module Layer 2: nRF24L01+ (PD2-7), LEDs (PC0-7), ATmega32 (PA0-2), MAX232 (PD0,1)
* Robot Modules 1 and 2: nRF24L01+ (PD2-7), LEDs (PC0-7), ATmega32 (PA0-2), MAX232 (PD0,1), L293D (PD0-3) + DC Geared Motors

## Software used

* HP InfoTech CodeVisionAVR C Compiler 1.2.4.5
* AVRDude 5.6
* RealTerm Serial Capture Program 2.0.0.57


# License

VoCoRoBo is intended to be used in both open-source and commercial environments. It is licensed under Apache License v2.0 license. Please review LICENSE.txt for more details.

[Sagun Man Singh Shrestha](http://sagunms.wordpress.com) | http://sagunms.com.np

Follow me on twitter [@sagunms](http://www.twitter.com/sagunms)