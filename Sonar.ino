
void initSonar()
{
	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);	
}

boolean doSonarping()
{
		duration = sonar.ping();  // Send ping, get ping time in microseconds (uS). 		
		distanceCm = duration / US_ROUNDTRIP_CM; // convert the time into a distance	
	
	
	if(distanceCm <=5)
	{
		
	}
	else if(distanceCm<=40)
	{
		return true;
	}
	else
	{
		return false;
	}

	if (distanceCm <= 0)
	{
		//out of range
	}
	else
	{
		
	}

}

boolean singlePing(int angle, aping *apn)
{
	
	long sumOfDistances=0;
	for (int i=0; i < numberOfPings; i++)
	{
		duration = sonar.ping();  // Send ping, get ping time in microseconds (uS).
		distanceCm = duration / US_ROUNDTRIP_CM; // convert the time into a distance
		sumOfDistances += distanceCm;
		delay(35);
	}	
	avgDistance=sumOfDistances/numberOfPings;
	
	switch (angle)
	{
		case 0:
			apn[0].a=angle;
			apn[0].dist=avgDistance;
		break;
		
		case 30:
			apn[1].a=angle;
			apn[1].dist=avgDistance;
		break;

		case 90:
			apn[2].a=angle;
			apn[2].dist=avgDistance;
		break;

		case 150:
			apn[3].a=angle;
			apn[3].dist=avgDistance;
		break;

		case 180:
			apn[4].a=angle;
			apn[4].dist=avgDistance;
		break;
	}

	
		
	if(avgDistance <=5)
	{
		
	}
	else if(avgDistance<=40)
	{
		return true;
	}
	else
	{
		return false;
	}

	if (avgDistance <= 0)
	{
		//out of range
	}
	else
	{
		
	}

}

char whereToTurn(aping *apg)
{
	return 0;
}
