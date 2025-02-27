#include <Arduino.h>
#include <TimerOne.h> // Install this library via the Arduino Library Manager

// --------------------- Pin Definitions ---------------------
const int LEFT_MOTOR_PUL  = 3;
const int LEFT_MOTOR_DIR  = 2;
const int RIGHT_MOTOR_PUL = 4;
const int RIGHT_MOTOR_DIR = 5;
const int SLEEP = 7;

// --------------------- Robot / Stepper Parameters ---------------------
// Change these values to match your robot’s hardware.
const float WHEEL_DIAMETER_MM  = 40.0;          
const float WHEEL_CIRCUMFERENCE_MM = 3.14159 * WHEEL_DIAMETER_MM;
const int   STEPS_PER_REV      = 200;    // Full step (1.8° step) motor
const int   MICROSTEPPING      = 1;      // Set your DM542 DIP switches accordingly

// Total steps per revolution considering microstepping
const int STEPS_PER_REV_TOTAL = STEPS_PER_REV * MICROSTEPPING;

// --------------------- Motion Control Variables ---------------------
volatile bool movementActive = false; // Is a move in progress?
volatile long targetSteps    = 0;      // Total steps to move
volatile long stepsCompleted = 0;      // Steps executed so far
volatile bool stepPinState   = false;   // Current state of the step output

// Desired step interval (in microseconds) for each toggle.
// For example, an interval of 1000µs means the pin toggles every 1000µs,
// yielding a square wave with 50% duty cycle and a full period of 2000µs.
unsigned long stepIntervalMicros = 2000;

// --------------------- Timer Interrupt Service Routine ---------------------
// This ISR is called every 'stepIntervalMicros' microseconds.
void stepISR() {
  if (!movementActive)
    return;

  // Toggle the step pin state.
  stepPinState = !stepPinState;
  digitalWrite(LEFT_MOTOR_PUL, stepPinState);
  digitalWrite(RIGHT_MOTOR_PUL, stepPinState);

  // Count a full step when the signal goes LOW (completing one HIGH/LOW cycle).
  if (!stepPinState) {
    stepsCompleted++;
    if (stepsCompleted >= targetSteps) {
      movementActive = false;
      digitalWrite(SLEEP, LOW);
      // Detach the timer interrupt to stop stepping.
      Timer1.detachInterrupt();
    }
  }
}

// --------------------- Function to Initiate a Move ---------------------
void moveForwardNonBlocking(float distance_mm, unsigned long intervalUS) {
  // Calculate the number of steps needed.
  float revolutionsNeeded = distance_mm / WHEEL_CIRCUMFERENCE_MM;
  long stepsNeeded = (long)(revolutionsNeeded * STEPS_PER_REV_TOTAL);

  // Set the direction pins for forward motion.
  // Adjust LOW/HIGH depending on how your motor wiring defines "forward."
  digitalWrite(LEFT_MOTOR_DIR, LOW);
  digitalWrite(RIGHT_MOTOR_DIR, LOW);

  // Initialize motion variables.
  targetSteps = stepsNeeded;
  stepsCompleted = 0;
  stepIntervalMicros = intervalUS;
  stepPinState = false;
  digitalWrite(LEFT_MOTOR_PUL, stepPinState);
  digitalWrite(RIGHT_MOTOR_PUL, stepPinState);
  movementActive = true;
  digitalWrite(SLEEP, HIGH);

  // Initialize Timer1 with the specified period in microseconds.
  Timer1.initialize(stepIntervalMicros);
  Timer1.attachInterrupt(stepISR);
  
  Serial.print("Starting move: ");
  Serial.print(distance_mm);
  Serial.print(" mm (");
  Serial.print(targetSteps);
  Serial.println(" steps)");
}

void setup() {
  pinMode(SLEEP, OUTPUT);
  // Configure pin modes.
  pinMode(LEFT_MOTOR_PUL, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PUL, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  
  // Set initial states.
  digitalWrite(SLEEP, LOW);
  digitalWrite(LEFT_MOTOR_PUL, LOW);
  digitalWrite(RIGHT_MOTOR_PUL, LOW);
  digitalWrite(LEFT_MOTOR_DIR, LOW);
  digitalWrite(RIGHT_MOTOR_DIR, LOW);

  Serial.begin(9600);
  Serial.println("Timer interrupt based stepper control started.");

  // For demonstration, start a move after a short delay.
  delay(2000);
  moveForwardNonBlocking(300.0, 1000); // Move 300 mm with a 1000µs interval per toggle.
}

void loop() {
  // The main loop can run other tasks concurrently.
  if (movementActive) {
    Serial.print("Steps completed: ");
    Serial.println(stepsCompleted);
  } else {
    Serial.println("Movement complete.");
    // For demonstration, do nothing further once movement is complete.
    delay(2000);  // Just to slow down serial prints; not affecting step generation.
  }
}

