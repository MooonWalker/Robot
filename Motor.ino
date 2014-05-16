int ENA = 5;
int ENB = 6;
int IN_A1 = 4;
int IN_B1 = 2;
int IN_A2 = 7;
int IN_B2 = 8;

void motorInit()
{
	pinMode(ENA, OUTPUT);
	pinMode(ENB,OUTPUT);
	pinMode(IN_A1,OUTPUT);
	pinMode(IN_B1, OUTPUT);
	pinMode(IN_A2,OUTPUT);
	pinMode(IN_B2, OUTPUT);
	digitalWrite(ENA, LOW);
	digitalWrite(ENB, LOW);
	digitalWrite(IN_A1, HIGH);
	digitalWrite(IN_B1, LOW);
	digitalWrite(IN_A2, LOW);
	digitalWrite(IN_B2, HIGH);
}

void motorsFwd(int speed)
{
	motorInit();
	
	analogWrite(ENA, speed);//start driving motorA
	analogWrite(ENB, speed);//start driving motorB	
}

void motorsStop(bool brake)
{
	if (brake)
	{
		analogWrite(ENA, 255);
		analogWrite(ENB, 255);
		digitalWrite(IN_A1, HIGH);
		digitalWrite(IN_B1, HIGH);
		digitalWrite(IN_A2, HIGH);
		digitalWrite(IN_B2, HIGH);	
	}
	else
	{
		analogWrite(ENA, 0);
		analogWrite(ENB, 0);
		digitalWrite(IN_A1, LOW);
		digitalWrite(IN_B1, LOW);
		digitalWrite(IN_A2, LOW);
		digitalWrite(IN_B2, LOW);
	
	}
	
	
}