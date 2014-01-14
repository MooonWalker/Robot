/*####################################################################
 FILE: motorshield.h
 VERSION: 1.0
 PURPOSE: Dual DC Motor Shield library for Arduino

 HISTORY:
 Mirko Prosseda - Original version (23/06/2013)
#######################################################################*/


#ifndef motorshield_h
#define motorshield_h

#include "Arduino.h"

#define IB 9
#define IA 10
#define E1 3
#define IIB 2
#define IIA 6
#define E2 5


class motorshield
{
public:
    void initialize();                             // initialize the Motor Shield hardware
    void setMotorDir(uint8_t ch, uint8_t dir);     // set direction for a specific motor channel
    void setMotorSpeed(uint8_t ch, uint8_t speed); // set speed percentage for a specific motor channel
    float getCurrent(uint8_t ch);                  // read current absorption on a specific channel
    float getTemperature();                        // read board temperature
};
#endif
