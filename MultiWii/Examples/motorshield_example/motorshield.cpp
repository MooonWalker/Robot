/*####################################################################
 FILE: motorshield.cpp
 VERSION: 1.0
 PURPOSE: Dual DC Motor Shield library for Arduino

 HISTORY:
 Mirko Prosseda - Original version (23/06/2013)
#######################################################################*/


#include "motorshield.h"



void motorshield::initialize()
{
    // set digital pins direction
    pinMode(IA, OUTPUT);
    pinMode(IB, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(IIA, OUTPUT);
    pinMode(IIB, OUTPUT);
    pinMode(E2, OUTPUT);
    // reset digital pins and disables both channels
    digitalWrite(IA, LOW);
    digitalWrite(IB, LOW);
    digitalWrite(IIA, LOW);
    digitalWrite(IIB, LOW);
    digitalWrite(E1, LOW);
    digitalWrite(E2, LOW);

}

// Set direction for a specific motor channel
void motorshield::setMotorDir(uint8_t ch, uint8_t dir)
{
    if((dir == 0) || (dir == 1)) // dir parameter must be 0 or 1
    {
        switch(ch)               // ch parameter must be 1 or 2
        {
            case 1:
                digitalWrite(IA, dir);
                digitalWrite(IB, !dir);
                break;
            case 2:
                digitalWrite(IIA, dir);
                digitalWrite(IIB, !dir);
                break;
        }
    }
}

// Set speed percentage for a specific motor channel
void motorshield::setMotorSpeed(uint8_t ch, uint8_t speed)
{
    
        switch(ch)       // ch parameter must be 1 or 2
        {
            case 1:
                analogWrite(E1, speed);
                break;
            case 2:
                analogWrite(E2, speed);
                break;
        }
    
}
