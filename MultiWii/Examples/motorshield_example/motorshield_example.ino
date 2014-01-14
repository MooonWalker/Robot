/*####################################################################
 FILE: motorshield_example.ino
 VERSION: 1.0
 PURPOSE: Dual DC Motor Shield test sketch

 Description:
 * Shield temperature are monitored on Serial Monitor
 * Motor speed and direction are controlled by a potentiometer.

 Connections:
 * this sketch needs two potentiometers attached on 3-pin strips labeled
 * "Analog Input"; they are used to change motor speed and direction

 HISTORY:
 Mirko Prosseda - Original version (23/06/2013)
#######################################################################*/


#include "motorshield.h"
#include <Wire.h>

motorshield MotorShield;

//const int analogPin1 = A2;
//const int analogPin2 = A3;

//int analogValue1, analogValue2;
uint8_t pwmValue1, pwmValue2;

//float temperature;

void setup()
{
    MotorShield.initialize(); // initialize the Motor Shield hardware
    Serial.begin(9600);       // initialize Serial Port
//    Serial.println("Dual DC Motor Shield test sketch");
}

void loop()
{
 
    MotorShield.setMotorDir(1, 0);
    MotorShield.setMotorDir(2, 1);
  
  // apply speed adjustment
  MotorShield.setMotorSpeed(1, 150);
  MotorShield.setMotorSpeed(2, 150);

//  temperature = MotorShield.getTemperature();
//  Serial.println(temperature,1);        // temperature is printed with 1 decimal
}
