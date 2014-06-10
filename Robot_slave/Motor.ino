int ENA = 5;	//lila E1
int ENB = 6;	//citromsárga E2
int IN_A1 = 10;	//fehér 1B
int IN_B1 = 9;	//szürke 1B
int IN_A2 = 7;	//barna 2A
int IN_B2 = 2;	//narancs 2B

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
	digitalWrite(IN_A1, LOW);
	digitalWrite(IN_B1, LOW);
	digitalWrite(IN_A2, LOW);
	digitalWrite(IN_B2, LOW);
}

void motorsFwd(int speed)
{
	motorInit();
	
	digitalWrite(IN_A1, LOW);
	digitalWrite(IN_B1, HIGH);
	digitalWrite(IN_A2, HIGH);
	digitalWrite(IN_B2, LOW);
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

bool turnRight(int crs)
{
	double offset=0, hdg=0;

	motorInit();
	calibrateSensors();
	readIMU();
	offset=gyroZangle;
	hdg=offset-gyroZangle;

	
		do 
		{
			readIMU();
			hdg=offset-gyroZangle;
#ifdef DEBUG
			Serial.println(hdg);
			Serial.print("IMU Z: ");
			Serial.println(gyroZangle);
			Serial.print("offset ");
			Serial.println(offset);
			Serial.println(crs);
			delay(2000);
#endif
		} while (hdg!=crs);
		

	//digitalWrite(IN_A1, LOW);
	//digitalWrite(IN_B1, HIGH);
	//digitalWrite(IN_A2, LOW);
	//digitalWrite(IN_B2, HIGH);
	//analogWrite(ENA, 230);//start driving motorA
	//analogWrite(ENB, 230);//start driving motorB

	return true; // turn completed
}

void turnLeft(int crs)
{
	motorInit();
	
	digitalWrite(IN_A1, HIGH);
	digitalWrite(IN_B1, LOW);
	digitalWrite(IN_A2, HIGH);
	digitalWrite(IN_B2, LOW);
	analogWrite(ENA, 230);//start driving motorA
	analogWrite(ENB, 230);//start driving motorB
}