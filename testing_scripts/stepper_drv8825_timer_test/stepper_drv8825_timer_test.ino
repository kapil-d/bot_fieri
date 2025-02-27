#include <Arduino.h>
#include <TimerOne.h> // Install this library via the Arduino Library Manager

// --------------------- Pin Definitions ---------------------
const int LEFT_MOTOR_DIR  = 2;
const int LEFT_MOTOR_PUL  = 3;
const int RIGHT_MOTOR_DIR = 4;
const int RIGHT_MOTOR_PUL = 5;

const int SLEEP = 7;             //when sleep is low, driver stops

// --------------------- Robot / Stepper Parameters ---------------------
const float WHEEL_DIAMETER_MM  = 69.5;         
const float WHEEL_BASE_MM  = 69.5;        //TODO  
const float WHEEL_CIRCUMFERENCE_MM = 3.14159 * WHEEL_DIAMETER_MM;
const int   STEPS_PER_REV      = 200;    // Full step (1.8° step) motor
const int   MICROSTEPPING      = 1;      // Set your DM542 DIP switches accordingly
const int STEPS_PER_REV_TOTAL = STEPS_PER_REV * MICROSTEPPING;


// --------------------- Motion Control Variables ---------------------
volatile bool movementActive = false;   // Is a move in progress?
volatile long targetSteps    = 0;       // Total steps to move
volatile long stepsCompleted = 0;       // Steps executed so far
volatile bool stepPinState   = false;   // Current state of the step output
unsigned long stepIntervalMicros = 2000;



enum Motion {FORWARD, BACKWARD, ROTATE_CW, ROTATE_CCW};

typedef enum {FWD1, WAIT_AFTER1 BKWD1, WAIT_AFTER2, ROTATE1, WAIT_AFTER3} state; //for testing

state = FWD1;

void stepISR();
void move(int mode, unsigned long intervalUS, float distance_mm = 0, float degrees = 0);


void setup() {
  
  // Configure pin modes.
  pinMode(LEFT_MOTOR_PUL, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PUL, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(SLEEP, OUTPUT);
  
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

  switch(state) {
    case IDLE:
      return; //something to do nothing
    case FWD1:
      
      period = 1000 //us
      distance_mm = 300 //mm
      addtl_wait_duration = 1000 //ms
      wait_start_millis = millis();
      wait_duration = (distance_mm / WHEEL_CIRCUMFERENCE_MM * STEPS_PER_REV_TOTAL) * period + addtl_wait_duration

      move(FORWARD, period, distance_mm); 
      state = WAIT_AFTER1;
     
      
    case WAIT_AFTER1:
      if (millis() - wait_start_millis >= wait_duration) {
        state == BKWD1;
      }

    case BKWD1:
    period = 1000 //us
    distance_mm = 300 //mm
    addtl_wait_duration = 1000 //ms
    wait_start_millis = millis();
    wait_duration = (distance_mm / WHEEL_CIRCUMFERENCE_MM * STEPS_PER_REV_TOTAL) * period + addtl_wait_duration

    move(FORWARD, period, distance_mm); 
    state = WAIT_AFTER2;

  }
}



void checkGlobalEvents(void) {
  ifLimitSwitchesTrigger()
    RespToLimitSwitches();

}





// --------------------- Timer Interrupt Service Routine ---------------------
// Handles the pulse signal. Enabled when:
//  - movementActive == 1
//  - targetSteps > 0
// QUESTION: can't we have the clk running constantly and just pull sleep low when stopped?
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
void move(int mode, unsigned long intervalUS, float distance_mm = 0, float degrees = 0) {
  state++;

  switch(mode) {
    case FORWARD:
      float revolutionsNeeded = distance_mm / WHEEL_CIRCUMFERENCE_MM;
      long stepsNeeded = (long)(revolutionsNeeded * STEPS_PER_REV_TOTAL); //should be int?

      int dir1, dir2 = 0;

      break;
    case BACKWARD:
      float revolutionsNeeded = distance_mm / WHEEL_CIRCUMFERENCE_MM;
      long stepsNeeded = (long)(revolutionsNeeded * STEPS_PER_REV_TOTAL); //should be int?

      int dir1, dir2 = 1;

      break;
    case ROTATE_CW:
      float arc_length = degrees/360*3.1415926*WHEEL_BASE_MM;
      int stepsNeeded = (int)floor(arc_length*STEPS_PER_REV_TOTAL/WHEEL_CIRCUMFERENCE_MM);

      int dir1 = 0;
      int dir2 = 1;


      break;
    case ROTATE_CCW:
      float arc_length = degrees/360*3.1415926*WHEEL_BASE_MM;
      int stepsNeeded = (int)floor(arc_length*STEPS_PER_REV_TOTAL/WHEEL_CIRCUMFERENCE_MM);

      int dir1 = 1;
      int dir2 = 0;

      break;
  }


  // Calculate the number of steps needed.
  

  // Set the direction pins for forward motion.
  // Adjust LOW/HIGH depending on how your motor wiring defines "forward."
  digitalWrite(LEFT_MOTOR_DIR, dir1);
  digitalWrite(RIGHT_MOTOR_DIR, dir2);

  // Initialize motion variables.
  targetSteps = stepsNeeded;
  stepsCompleted = 0;
  stepIntervalMicros = intervalUS;
  stepPinState = false;
  digitalWrite(LEFT_MOTOR_PUL, stepPinState);
  digitalWrite(RIGHT_MOTOR_PUL, stepPinState);
  digitalWrite(SLEEP, HIGH);
  movementActive = true;

  // Initialize Timer1 with the specified period in microseconds.
  Timer1.initialize(stepIntervalMicros);
  Timer1.attachInterrupt(stepISR);
  
  Serial.print("Starting move: ");
  Serial.print(distance_mm);
  Serial.print(" mm (");
  Serial.print(targetSteps);
  Serial.println(" steps)");
}