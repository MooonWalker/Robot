void lcdPrintsSTBY()
{
	lcd.setCursor(0, 0);
	lcd.print("STBY  ");
	lcd.print("O: ");
	lcd.print(distanceCm, DEC);
	lcd.print("cm");
	lcd.setCursor(15, 0);
	lcd.print( object);
	
	
	lcd.setCursor(0,1);
	lcd.print("BusV: ");
	lcd.print(busvoltage);
	lcd.print("V");
	
}

void lcdPrintsLook()
{
	lcd.setCursor(0, 0);
	lcd.print("LOOK");
	lcd.print("O: ");
	lcd.print(distanceCm, DEC);
	lcd.print("cm");
	lcd.setCursor(14, 0);
	lcd.print( object);
	
	
	lcd.setCursor(0,1);
	lcd.print("BusV: ");
	lcd.print(busvoltage);
	lcd.print("V");
	
}
