VoCoRoBo: Remote Speech Recognition and Tilt-Sensing Multi-Robotic System
=========================================================================
This work  is  based  on  the  implementation  of  real-time  speech  recognition  using DSP 
algorithms such as Chebyshev  IIR  filters, accelerometer for tilt-sensing and establishment of short-
range  wireless  secure  link with ARC4  cipher,  all  using  low-cost  8-bit ATmega microcontrollers.  
The  robot  implements  a  simple  but  effective  algorithm  for  comparing  the  spoken  word  with  a 
dictionary of fingerprints using a modified Euclidean distance calculation. It also includes the ability 
to securely control the navigation of multiple robots located at remote locations wirelessly from the 
Control Module and also gather the various environmental data collected by the Robot Modules and 
display them in the back to Control. Considering the time-critical algorithms actually requiring large 
computations as well as a variety of sensors interfaced   in  the system, this project can demonstrate  
how one can build an expansible multi-robotic system from cheap and ubiquitous electronics. 

## Introduction

## Control Module

### Layer 1

File: vocorobo.c 
Folder: \src\controlmodule\layer1-speechrecog-mma7260q\
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

### Layer 2

File: dict.h
Folder: \src\controlmodule\layer1-speechrecog-mma7260q\
----------------
This is the dictionary template file to store the 160 data points 
(16 for each 10 filters) again each for 5 voice commands (front, back,
left, right and stop).

## License

VoCoRoBo is intended to be used in both open-source and commercial environments. It is licensed under Apache License v2.0 license. Please review LICENSE.txt for more details.
