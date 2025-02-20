// ME 210 Final Project: Bot Fieri
// Authors: Kapil Dheeriya, Cole Robins, Graham Shunk, Charlie Lyerly

#define USE_TIMER_1 true  
#define USE_TIMER_2 true

#include "TimerInterrupt.h"
#include "ISR_Timer.h"  

// -------------------- Pin Definitions --------------------
#define LEFT_PWM        9
#define LEFT_ENABLE     8
#define LEFT_DIRECTION1 7
#define LEFT_DIRECTION2 6

#define RIGHT_PWM       10
#define RIGHT_ENABLE    13
#define RIGHT_DIRECTION1 12
#define RIGHT_DIRECTION2 11

// -------------------- Global Variables --------------------
volatile bool inMotion = false; // Tracks if motors are currently moving


// -------------------- Function Prototypes --------------------
void setMotorsForward(int speed);
void setMotorsBackward(int speed);
void stopMotors();
void setDirection(bool motorDirection, int DIR_PIN1, int DIR_PIN2);
// -------------------- Setup --------------------
void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize motor pins
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_ENABLE, OUTPUT);
  pinMode(LEFT_DIRECTION1, OUTPUT);
  pinMode(LEFT_DIRECTION2, OUTPUT);

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_ENABLE, OUTPUT);
  pinMode(RIGHT_DIRECTION1, OUTPUT);
  pinMode(RIGHT_DIRECTION2, OUTPUT);

  // Always enable the motor driver, set motor speed to 0
  digitalWrite(LEFT_ENABLE, HIGH);
  digitalWrite(RIGHT_ENABLE, HIGH);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  
  // Initialize Timer1
  ITimer1.init();

  // Print instructions
  Serial.println("Type 'f' for forward, 'b' for backward, any other char to stop.");
}

// -------------------- Loop --------------------
void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 'f') {
      // Move motors forward at speed 255
      setMotorsForward(255);
      // Schedule a stop after 1 second (1000 ms)
      ITimer1.attachInterruptInterval(1000, stopMotors, 1000);
      inMotion = true;
    }
    else if (c == 'b') {
      // Move motors backward at speed 255
      setMotorsBackward(255);
      // Schedule a stop after 1 second (1000 ms)
      ITimer1.attachInterruptInterval(1000, stopMotors, 1000);
      inMotion = true;
    }
  
    else if (c == '\n' || c == '\r') {
      // Do nothing
    }
    else {
      // Any other character -> Stop immediately
      stopMotors();
    }
  }

  // Do other non-blocking tasks here if needed
}

// -------------------- Motor Control Functions --------------------

// Run both motors forward at a specified speed (0-255)
void setMotorsForward(int speed) {
  // Use helper function to set direction of each motor forward
  setDirection(true, LEFT_DIRECTION1, LEFT_DIRECTION2);
  setDirection(true, RIGHT_DIRECTION1, RIGHT_DIRECTION2);

  // Set PWM speed (0-255).
  analogWrite(LEFT_PWM, speed);
  analogWrite(RIGHT_PWM, speed);
  Serial.print("Motors moving forward at speed ");
  Serial.println(speed);
}

// Run both motors backward at a specified speed (0-255)
void setMotorsBackward(int speed) {
  // Use helper function to set direction of each motor backward
  setDirection(false, LEFT_DIRECTION1, LEFT_DIRECTION2);
  setDirection(false, RIGHT_DIRECTION1, RIGHT_DIRECTION2);

  // Set PWM speed (0-255).
  analogWrite(LEFT_PWM, speed);
  analogWrite(RIGHT_PWM, speed);
  Serial.print("Motors moving backward at speed ");
  Serial.println(speed);
}

// Stop both motors by setting PWM to 0
void stopMotors() {
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  // Cancel any scheduled timer interrupt
  //ITimer1.detachInterrupt();
  inMotion = false;
  Serial.println("Motors stopped.");
}

// -------------------- Direction Helper Function --------------------
// set the direction of a single motor
void setDirection(bool motorDirection, int DIR_PIN1, int DIR_PIN2) {
  digitalWrite(DIR_PIN1, motorDirection);
  digitalWrite(DIR_PIN2, !motorDirection);
}
