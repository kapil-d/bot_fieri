#include <HCSR04.h>

#define ECHO 2
#define TRIG 3
#define TRANSFER 4
HCSR04 hc(TRIG, ECHO); //initialisation class HCSR04 (trig pin , echo pin)

unsigned long distance;

void setup()
{
    pinMode(TRANSFER, OUTPUT);
    Serial.begin(9600);
}



void loop() {
    
    distance = hc.dist();
    Serial.println(distance);
    if (distance > 60 && distance < 100) {
      digitalWrite(TRANSFER, HIGH
      //Serial.println("HIGH");
    } else {
      digitalWrite(TRANSFER, LOW);
      //Serial.println("LOW");
    }
    delay(10);                 // we suggest to use over 60ms measurement cycle, in order to prevent trigger signal to the echo signal.
}


