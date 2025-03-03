#include <Arduino.h>
#include <TimerOne.h> // Install this library via the Arduino Library Manager
#include "ServoTimer2.h" 


// --------------------- Pin Definitions ---------------------
const int LEFT_MOTOR_DIR  = 2;
const int LEFT_MOTOR_PUL  = 3;
const int RIGHT_MOTOR_DIR = 4;
const int RIGHT_MOTOR_PUL = 5;

const int SLEEP = 7;             //when sleep is low, driver stops
const int LIMIT1 = 8;
const int LIMIT3 = 6;
const int LIMIT2 = 10;
const int LIMIT4 = 11;

const int SERVO = 9; 

const int US_TRANSFER = 12; //signal coming from second arduino


// --------------------- Robot / Stepper Parameters ---------------------
const float WHEEL_DIAMETER_MM  = 69.5;         
const float WHEEL_BASE_MM  = 221;          
const float WHEEL_CIRCUMFERENCE_MM = 3.14159 * WHEEL_DIAMETER_MM;
const int   STEPS_PER_REV      = 200;    // Full step (1.8° step) motor
const int   MICROSTEPPING      = 4;      // Set your DM542 DIP switches accordingly
const int STEPS_PER_REV_TOTAL = STEPS_PER_REV * MICROSTEPPING;


// --------------------- Motion Control Variables ---------------------
volatile bool movementActive = false;   // Is a move in progress?
volatile long targetSteps    = 0;       // Total steps to move
volatile long stepsCompleted = 0;       // Steps executed so far
volatile bool stepPinState   = false;   // Current state of the step output
unsigned long stepIntervalMicros = 2000;


int dir1;
int dir2;
float revolutionsNeeded;
long stepsNeeded;
float arc_length;

float period = 1000; //us
float distance_mm = 300; //mm
float degrees = 90; //degrees


//States
enum Motion {FORWARD, BACKWARD, ROTATE_CW, ROTATE_CCW};
typedef enum {IDLE, ORIENT_ROTATE, ORIENT_BACKWARD, FWD1} States_t; //for testing
States_t state = IDLE;


//Function Definitions
bool DetectFirstLimitSwitchTrigger();
bool DetectCorner();
bool DetectUSThreshold();
void stepISR();
void move(int mode, unsigned long intervalUS, float distance_mm = 0, float degrees = 0);
void stopPreviousMotion();


//SERVO SETUP
ServoTimer2 myServo;
long trans_millis;

void setup() {
  Serial.begin(9600);

  // Configure pin modes.
  pinMode(LEFT_MOTOR_PUL, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PUL, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(SLEEP, OUTPUT);
  
  pinMode(LIMIT1, INPUT);
  pinMode(LIMIT2, INPUT);
  pinMode(LIMIT3, INPUT);
  pinMode(LIMIT4, INPUT);

  pinMode(US_TRANSFER, INPUT);
  
  // Set initial states.
  digitalWrite(SLEEP, LOW);
  digitalWrite(LEFT_MOTOR_PUL, LOW);
  digitalWrite(RIGHT_MOTOR_PUL, LOW);
  digitalWrite(LEFT_MOTOR_DIR, LOW);
  digitalWrite(RIGHT_MOTOR_DIR, LOW);

  
  //SERVO CODE: 750 is 0 degrees (holding balls back), and 1500 is 90 degrees up
  myServo.attach(SERVO);
  myServo.write(750);
}




void loop() {

  switch(state) {
    case IDLE:
      if (DetectFirstLimitSwitchTrigger()) {
          move(ROTATE_CW, 1000, 0, 360); //rotate up to 720 degrees (for safety) with 1ms period
          state = ORIENT_ROTATE;
      }
      break;

    case ORIENT_ROTATE:
      if (DetectUSThreshold()) {
        stopPreviousMotion();      //sets motionactive, targetSteps, and sleep = 0
        move(BACKWARD, 1000, 1000); //move backward by up to 1 meter
        state = ORIENT_BACKWARD;
      }
      break;

    case ORIENT_BACKWARD:
      if (DetectCorner()) {
        stopPreviousMotion();
        move(FORWARD, 1000, 300); //move forward for 30cm
        myServo.write(1500);      //open the servo for test
        state = FWD1;
        //Serial.println("corner detected");
      }
      break;
      

    //COLE PICKS UP HERE
    case FWD1:
      if (movementActive == 0) { //forward move has completed
        state = IDLE;            //loop back to the beginning
        myServo.write(750);      //close the servo for test
        //Serial.println("forward completed");
      }
      break;
  }
}



void stopPreviousMotion() {
  movementActive = false;     //halt previous motion
  targetSteps = 0;            //halt previous motion
  digitalWrite(SLEEP, LOW);   //halt previous motion
  delay(500);
}

bool DetectFirstLimitSwitchTrigger() {
  return (digitalRead(LIMIT1) || digitalRead(LIMIT2) || digitalRead(LIMIT3) || digitalRead(LIMIT4));
}


bool DetectUSThreshold() {
  if (digitalRead(US_TRANSFER) == 1) {
    return true;
  }
  return false;
}

bool DetectCorner() {
  int limSwitch[4];
  limSwitch[0] = digitalRead(LIMIT1);
  limSwitch[1] = digitalRead(LIMIT2);
  limSwitch[2] = digitalRead(LIMIT3);
  limSwitch[3] = digitalRead(LIMIT4);

  if ((limSwitch[0] + limSwitch[1] + limSwitch[2] + limSwitch[3]) == 2) {
    return true;
  }
  return false;
}




// --------------------- Timer Interrupt Service Routine ---------------------
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

  switch(mode) {
    case FORWARD:
      revolutionsNeeded = distance_mm / WHEEL_CIRCUMFERENCE_MM;
      stepsNeeded = (long)(revolutionsNeeded * STEPS_PER_REV_TOTAL); //should be int?
      dir1 = 0;
      dir2 = 0;
      break;

    case BACKWARD:
      revolutionsNeeded = distance_mm / WHEEL_CIRCUMFERENCE_MM;
      stepsNeeded = (long)(revolutionsNeeded * STEPS_PER_REV_TOTAL); //should be int?
      dir1 = 1;
      dir2 = 1;
      break;

    case ROTATE_CW:
      arc_length = degrees/360*3.1415926*WHEEL_BASE_MM;
      stepsNeeded = (int)floor((arc_length/WHEEL_CIRCUMFERENCE_MM)*STEPS_PER_REV_TOTAL);
      dir1 = 1;
      dir2 = 0;
      break;

    case ROTATE_CCW:
      arc_length = degrees/360*3.1415926*WHEEL_BASE_MM;
      stepsNeeded = (int)floor((arc_length/WHEEL_CIRCUMFERENCE_MM)*STEPS_PER_REV_TOTAL);
      dir1 = 0;
      dir2 = 1;
      break;
  }
  

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
}