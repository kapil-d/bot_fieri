#include <Arduino.h>
#include <TimerOne.h> // Install this library via the Arduino Library Manager
#include <Servo.h>
#include <HCSR04.h>

// --------------------- Pin Definitions ---------------------
const int LEFT_MOTOR_DIR  = 2;
const int LEFT_MOTOR_PUL  = 3;
const int RIGHT_MOTOR_DIR = 4;
const int RIGHT_MOTOR_PUL = 5;

const int SLEEP = 7;             //when sleep is low, driver stops
const int LIMIT1 = 8;
const int LIMIT2 = 9;
const int LIMIT3 = 10;
const int LIMIT4 = 11;

const int SERVO = 6;

const int TRIG = A0;
const int ECHO = 12;

// --------------------- Robot / Stepper Parameters ---------------------
const float WHEEL_DIAMETER_MM  = 69.5;         
const float WHEEL_BASE_MM  = 221;          
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

volatile bool standby = true;               // Set to 0 once the challenge begins
volatile float ex_time = 0.0;               //these two signals are distinct!
volatile float execution_time = 0.0;        //these two signals are distinct!


int dir1;
int dir2;
float revolutionsNeeded;
long stepsNeeded;
float arc_length;

float period = 1000; //us
float distance_mm = 300; //mm
float addtl_wait_duration = 1000; //ms
float wait_start_millis = millis();
float wait_duration = (distance_mm / WHEEL_CIRCUMFERENCE_MM * STEPS_PER_REV_TOTAL) * period + addtl_wait_duration;
float degrees = 90; //degrees

// Ultrasonic Sensor Setup
int THRESHOLD = 120;
HCSR04 hc(TRIG, new int[1]{ECHO}, 1);

//States
enum Motion {FORWARD, BACKWARD, ROTATE_CW, ROTATE_CCW};
typedef enum {IDLE, WAIT, FWD1, BKWD1, ROTATE1, orientRotate, orientBack} States_t; //for testing
States_t state = IDLE;
volatile States_t next_state = IDLE; //IMPORTANT SIGNAL


//Function Definitions
void stepISR();
void checkGlobalEvents();
bool DetectFirstLimitSwitchTrigger();
void RespToFirstLimitSwitchTrigger();
float move(int mode, unsigned long intervalUS, float distance_mm = 0, float degrees = 0);
void orient(float distEO, int limSwitch[4]);


//SERVO CODE
Servo myServo;  // Create a Servo object

bool positionToggle = false;  // Toggle flag for movement


void setup() {
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
  
  // Set initial states.
  digitalWrite(SLEEP, LOW);
  digitalWrite(LEFT_MOTOR_PUL, LOW);
  digitalWrite(RIGHT_MOTOR_PUL, LOW);
  digitalWrite(LEFT_MOTOR_DIR, LOW);
  digitalWrite(RIGHT_MOTOR_DIR, LOW);

  
  Serial.begin(9600);
  int initialPos = 90;
  myServo.attach(SERVO);  // Attach the servo to the pin
  myServo.write(initialPos);

  Serial.println("Setup Finished");
}






void loop() {

  checkGlobalEvents();

  switch(state) {
    case IDLE:
      standby = true;
      return; //something to do nothing

    case WAIT:
      if (millis() - wait_start_millis >= wait_duration) {
        // Serial.println(wait_duration);
        // Serial.println("forward done");
        state = next_state;
      }
      break;

    case FWD1:
      // Serial.println("forward state starting");
      period = 3000; //us
      distance_mm = 300; //mm
      addtl_wait_duration = 1000; //ms
      execution_time = move(FORWARD, period, distance_mm);
      wait_start_millis = millis();
      wait_duration = execution_time + addtl_wait_duration; 
      state = WAIT;
      next_state = BKWD1;
      break;
      
     
    case BKWD1:
      // Serial.println("backward state starting");
      period = 3000; //us
      distance_mm = 300; //mm
      addtl_wait_duration = 1000; //ms
      execution_time = move(BACKWARD, period, distance_mm);
      wait_start_millis = millis();
      wait_duration = execution_time + addtl_wait_duration; 
      state = WAIT;
      next_state = BKWD1;
      break;

    
    case ROTATE1:
      // Serial.println("rotate state starting");
      period = 5000; //us
      degrees = 90; //mm
      addtl_wait_duration = 1000; //ms
      execution_time = move(ROTATE_CW, period, 0, degrees);
      wait_start_millis = millis();
      wait_duration = execution_time + addtl_wait_duration; 
      state = WAIT;
      next_state = IDLE;
      break;

    
    case orientRotate:
      move(ROTATE_CW, 1000, 0, 360);
      break;

    
    case orientBack:
      move(FORWARD, 1000, 16, 0);
      break;
  }
}



void checkGlobalEvents(void) {
  // if checked after standby wouldnt work
  if (state == orientRotate && DetectUSThreshold()) {
    RespToUSThreshold();
  }

  if (DetectCorner()) {
    RespToCorner();
  }

  if (standby && DetectFirstLimitSwitchTrigger()) {
      RespToFirstLimitSwitchTrigger();    //start timer, set state to the first motion state
  }
}

bool DetectFirstLimitSwitchTrigger() {
  return (digitalRead(LIMIT1) || digitalRead(LIMIT2) || digitalRead(LIMIT3) || digitalRead(LIMIT4));
}

void RespToFirstLimitSwitchTrigger() {
  standby = false;
  state = orientRotate;
  //ALSO START 2m 10s timer!!!
}

bool DetectUSThreshold() {
  return (hc.dist(0) >= THRESHOLD);
}

void RespToUSThreshold() {
  // STOP ROTATE
  targetSteps = 0;
  state = orientBack;
}

bool DetectCorner() {
  // is there a clever logic gate return to this???
  if (digitalRead(LIMIT1) || digitalRead(LIMIT2) || digitalRead(LIMIT3) || digitalRead(LIMIT4)) {
    int limSwitch[4] = {digitalRead(LIMIT1), digitalRead(LIMIT2), digitalRead(LIMIT3), digitalRead(LIMIT4)};
    int count = 0;
    for (int i = 0; i < 4; i++) {
      if (limSwitch[i]) {
        count++;
      }

      if (count > 1) {
        return true;
      }
    }
  }
  return false;
}

void RespToCorner() {
  targetSteps = 0;
  state = BKWD1;
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
float move(int mode, unsigned long intervalUS, float distance_mm = 0, float degrees = 0) { //returns time to execute

  switch(mode) {
    case FORWARD:
      revolutionsNeeded = distance_mm / WHEEL_CIRCUMFERENCE_MM;
      stepsNeeded = (long)(revolutionsNeeded * STEPS_PER_REV_TOTAL); //should be int?
      dir1 = 0;
      dir2 = 0;
      ex_time = (distance_mm / WHEEL_CIRCUMFERENCE_MM * STEPS_PER_REV_TOTAL) * (period/1000);
      break;

    case BACKWARD:
      revolutionsNeeded = distance_mm / WHEEL_CIRCUMFERENCE_MM;
      stepsNeeded = (long)(revolutionsNeeded * STEPS_PER_REV_TOTAL); //should be int?
      dir1 = 1;
      dir2 = 1;
      ex_time = (distance_mm / WHEEL_CIRCUMFERENCE_MM * STEPS_PER_REV_TOTAL) * (period/1000);
      break;

    case ROTATE_CW:
      arc_length = degrees/360*3.1415926*WHEEL_BASE_MM;
      stepsNeeded = (int)floor((arc_length/WHEEL_CIRCUMFERENCE_MM)*STEPS_PER_REV_TOTAL);
      dir1 = 1;
      dir2 = 0;
      ex_time = ((degrees/360*3.1415926*WHEEL_BASE_MM) / WHEEL_CIRCUMFERENCE_MM * STEPS_PER_REV_TOTAL) * (period/1000);
      break;

    case ROTATE_CCW:
      arc_length = degrees/360*3.1415926*WHEEL_BASE_MM;
      stepsNeeded = (int)floor((arc_length/WHEEL_CIRCUMFERENCE_MM)*STEPS_PER_REV_TOTAL);
      dir1 = 0;
      dir2 = 1;
      ex_time = ((degrees/360*3.1415926*WHEEL_BASE_MM) / WHEEL_CIRCUMFERENCE_MM * STEPS_PER_REV_TOTAL) * (period/1000);
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

  // Serial.println(state);

  return ex_time;
}

void orient(float distEO, int limSwitch[4]) {
  /* 
  float: distEO
  bool array len 4: limSwitch
  float: THRESHOLD PARAM
  */

  // are these supposed to be false? --> set now though as always driving
  digitalWrite(LEFT_MOTOR_PUL, false);
  digitalWrite(RIGHT_MOTOR_PUL, false);
  
  while (distEO <= THRESHOLD) {
    digitalWrite(LEFT_MOTOR_DIR, 0);
    digitalWrite(RIGHT_MOTOR_DIR, 1);

    // note we are oversampling! should have 60 ms delay supposedly
    distEO = hc.dist(0);
  }

  // angle selected, so drive backwards
  digitalWrite(LEFT_MOTOR_DIR, 1);
  digitalWrite(RIGHT_MOTOR_DIR, 1);

  // slightly weak, maybe find some kill conditin
  // also maybe inbuilt for this?
  while (true) {
    int count = 0;
    for (int i = 0; i < 4; i++) {
      if (limSwitch[i]) {
        count++;
      }
    }

    if (count >= 2) {

      // Now centered so set off and forward. set coordinates for procedure.
      // setCoords();

      digitalWrite(LEFT_MOTOR_DIR, 1);
      digitalWrite(RIGHT_MOTOR_DIR, 1);
      break;
    } else {
      limSwitch[0] = digitalRead(LIMIT1);
      limSwitch[1] = digitalRead(LIMIT2);
      limSwitch[2] = digitalRead(LIMIT3);
      limSwitch[3] = digitalRead(LIMIT4);
    }
  }

}

