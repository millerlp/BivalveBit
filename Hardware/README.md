# BivalveBit hardware
 Heart rate and valve gape sensor wearable for oysters and mussels. 
 
 * Based around a ATmega4808 microcontroller
 * Designed to fit inside a nominal 3/4" schedule 40 PVC plumbing pipe (<18.5mm
 inside diameter) along with an 18650 size Lithium-Ion battery cell. 
 * Provides one I2C 4-pin port for communicating with a IR heart rate sensor
 * Provides one analog 4-pin port for communicating with an analog sensor such
 as the A1395 Hall effect sensor (with SLEEP pin on the 4th wire). 
 * Writes data to a standard microSD card
 
 Hardware design files for the main board were originally created in KiCad 5.1, 
 in the directory BivalveBit_RevE. The files have been updated to KiCad 6.0.1. 
 
 Hardware design files for the heart rate (and temperature) sensor board
 were created in Autodesk EAGLE 9.6, and are available in the 
 Mussel_heartrate_RevC_EAGLE directory. A version imported into KiCad 6.0.1 is available
 in the directory Mussel_heartrate_RevC_KiCad. 
 
 Hardware design files in KiCad 6.0.1 for the Hall effect sensor board using an Allegro A1395
 Hall effect sensor can be found in the directory Hall_effect_sensor_board_A139x. 
 
 
