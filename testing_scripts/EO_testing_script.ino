#include <HCSR04.h>

int n = 4;
HCSR04 hc(2, new int[n]{5, 6, 7, 8}, n); // class HCSR04 (trig pin, echo pin, number of sensor)

void setup()
{ Serial.begin(9600); }

void loop()
{
    for (int i = 0; i < n; i++ ) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" detects distance of ");
      Serial.print( hc.dist(i) ); //return curent distance in cm
      Serial.println("cm");
    }
    delay(60); // suggested 60ms sampling
}