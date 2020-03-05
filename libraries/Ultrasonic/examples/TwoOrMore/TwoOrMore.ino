// Ultrasonic - Library for HR-SC04 Ultrasonic Ranging Module.
// Rev.4 (06/2012)
// J.Rodrigo ( www.jrodrigo.net )
// more info at www.ardublog.com

#include <Ultrasonic.h>

Ultrasonic ultraright(14,15);   // (Trig PIN,Echo PIN)
Ultrasonic ultraleft(16,17);  // (Trig PIN,Echo PIN)

void setup() {
  Serial.begin(115200); 
}

void loop()
{
  Serial.print("Left: ");
  Serial.print(ultraleft.Ranging(CM)); // CM or INC
  Serial.print(" cm     " );
  delay(50);
  Serial.print("Right: ");
  Serial.print(ultraright.Ranging(CM)); // CM or INC
  Serial.println(" cm" );
  delay(50);
}