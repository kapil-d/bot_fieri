#include <HCSR04.h>

#define ECHO 2
#define TRIG 3
#define TRANSFER 4
HCSR04 hc(3, 2); //initialisation class HCSR04 (trig pin , echo pin)

void setup()
{
    pinMode(TRANSFER, OUTPUT);
    Serial.begin(9600);
}



void loop()
{
    
    if (hc.dist() < 0.5 || hc.dist() > 110) {
      digitalWrite(TRANSFER, HIGH);
    }
    delay(10);                 // we suggest to use over 60ms measurement cycle, in order to prevent trigger signal to the echo signal.
}


