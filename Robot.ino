/*********************

LCD

Sárga drót INT1!!
Billentyûmegszakítás

SCL zöld drót
SDA fehér drót.

Note: Beware the mismatch in interrupt numbers used on MEGA2560 versus the Arduino
Attach/DetachInterrupt commands, as follows:

Arduino             MEGA2560
int.0	pin D2   interrupt/counter 4
int.1	pin D3   interrupt/counter 5
int.2	pin D21  interrupt/counter 0
int.3	pin D20  interrupt/counter 1
int.4	pin D19  interrupt/counter 2
int.5	pin D18  interrupt/counter 3 (THIS IS WHAT IS USED BELOW) 


**********************/
#define DEBUG 1
#define LED_PIN 13
#define SERVOPIN 12 //MISO
#define TRIG_PIN 4	//obsolete
#define ECHO_PIN 11	//MOSI
//pin D3 INT1
#define MAX_DISTANCE 200 //Max distance to ping in cm

#define  HALF 133  //motor speed slow
#define  FULL 255  //motor speed full

#define BRAKE 1		//motors brake
#define NOBRAKE 0	//motors free run
#define RED 0x1

#include "Wire.h"
#include "Kalman.h"
#include "Servo.h"
#include "Adafruit_MCP23017.h"
#include "Adafruit_RGBLCDShield.h"
#include "Adafruit_INA219.h" 
#include "Adafruit_Sensor.h"
#include "Adafruit_HMC5883_U.h" //Magnetometer
#include "NewPing.h"
#include "IncFile1.h"

Servo myservo;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
Adafruit_INA219 ina219; 
//Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);

float busvoltage = 0;

uint32_t timer;

const int loopPeriod = 55;          // a period of 55ms = a frequency of 25Hz
const int slowPeriod = 2000;		// 2sec
unsigned long timeLoopDelay = 0, slowLoopDelay=0;

long duration, distanceCm, avgDistance, runningAverage=0;
const int  numberOfPings=10;

boolean blinkState = false;
boolean moving = false;
boolean object = false;
boolean start = false;
char direction =0;		//where to turn after lookaround

volatile boolean isTriggered=false;
uint8_t buttons;

enum STATE {STDBY, FWD, LOOKAROUND, TURNRIGHT, TURNLEFT, BACK};
STATE actualState = STDBY;
STATE prevState = STDBY;

aping angularPings[6];
boolean singlePing(int angle, aping *aPs);

void setup() 
{	
	//--------------------------------	
	ina219.begin();
	//This signal is active low, so HIGH-to-LOW when interrupt	
	
	//delay(20);
	//compass.begin();		//init compass    
    Serial.begin(9600);
	Wire.begin();
	lcd.begin(16, 2);
	lcd.enableButtonInterrupt();
	lcd.setBacklight(RED);
	lcd.setCursor(0, 0);
	lcd.print("Initsonar...");
	initSonar();
    // initialize device
    delay(200);
   
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("Calibrate sensors...");
	
	timer = micros();
	
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("Motorinit...");
	myservo.attach(SERVOPIN);
	delay(1000);
	myservo.write(0);
	delay(1000);
	myservo.write(180);
	delay(1000);
	myservo.write(90);
	delay(500);
	lcd.clear();
	attachInterrupt(4,ISR_Button,FALLING);
}

void loop() 
{	  
	
	switch(actualState)	//state machine
	{
		case STDBY:					//initial state		
			myservo.detach();	
			if(millis() - slowLoopDelay >= slowPeriod) //	.5Hz loop for pinging
			{
				//object=singlePing(90, angularPings);	
				//busvoltage = ina219.getBusVoltage_V();					
				slowLoopDelay = millis();
			}	
			
			if (isTriggered) //button interrupt1 fired
			{	
				Serial.println("trigg");
				handleKeypress();
				if (buttons)
				{
					lcd.setCursor(0,1);
					if (buttons & BUTTON_UP)
					{
						lcd.print("UP        ");						
					}
					if (buttons & BUTTON_DOWN)
					{
						lcd.print("DOWN      ");
					}
					if (buttons & BUTTON_LEFT)
					{
						lcd.print("LEFT      ");
					}
					if (buttons & BUTTON_RIGHT)
					{
						lcd.print("RIGHT ");
					}
					if (buttons & BUTTON_SELECT)  //start the journey
					{
						lcd.print("START    ");
						start=true;
						actualState = LOOKAROUND;  //next state
						lcd.clear();
					}
					delay(100);
				}		//if (buttons)	
			}		//if (isTriggered)
			else
			{
				Serial.println("else");
				lcdPrintsSTBY();
			}		
		break;
		
		case LOOKAROUND:
			Serial.println("LOOKAROUND");
			
			myservo.attach(SERVOPIN);
			myservo.write(0);
			singlePing(0,angularPings);
			delay(500);
			myservo.write(35);
			singlePing(35,angularPings);
			delay(500);			
			myservo.write(180);
			singlePing(180, angularPings);
			delay(500);
			myservo.write(145);
			singlePing(145, angularPings);
			delay(500);
			myservo.write(90);
			object=singlePing(90, angularPings);
			Serial.println(object);
			delay(500);
			
			lcdPrintsLook();

			direction=whereToTurn(angularPings);
			
			//--> turn to freeway and continue traveling			
			if (direction==90)
			{
				actualState=FWD;
			} 
			else if (direction<90)
			{
				actualState=TURNRIGHT;
			}
			else
			{
				actualState=TURNLEFT;
			}
		break;
		
		case FWD:
			if(millis() - timeLoopDelay >= loopPeriod) //	25Hz loop for pinging
			{
				object=doSonarping(); // read the measured distances
				busvoltage = ina219.getBusVoltage_V();
				timeLoopDelay = millis();
			}
			Serial.println("FWD");
			//motorsFwd(HALF);
			if(object) 
			{
				//motorsStop(BRAKE);
				actualState = LOOKAROUND;
			}
			start=false;
		break;
		
		case TURNLEFT:
			Serial.println("TURNLEFT");
			//-->FWD
		break;
		
		case TURNRIGHT:
			Serial.println("TURNRIGHT");
			for(int i=0; i<5;i++)
			{
				Serial.print(angularPings[i].a);
				Serial.print(" ; ");
				Serial.println(angularPings[i].dist);
			}
			//turnRight(90);
			//-->FWD
		break;
		
		case BACK:
			Serial.println("BACK");
		break;		
	}
	
	
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
	
}


void ISR_Button()
{
	
	isTriggered = true; //Also flag we triggered
	
}

void handleKeypress()
{
	isTriggered=false;
	buttons = lcd.readButtons();
}
