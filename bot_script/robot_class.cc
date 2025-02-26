#include "robot_class.hh"
#include <cmath>
#include <math.h>
#include <Arduino.h>

Robot::Robot(int steps_per_rotation, 
             float wheel_circumference,  
             float wheel_base,          //distance between the center of each wheel
             int DirPinLeftMotor, 
             int PulsePinLeftMotor,
             int DirPinRightMotor, 
             int PulsePinRightMotor) {

    int steps_per_rotation = steps_per_rotation;
    float wheel_circumference = wheel_circumference;
    float wheel_base = wheel_base;
    int DirPinLeftMotor = DirPinLeftMotor;
    int PulsePinLeftMotor = PulsePinLeftMotor;
    int DirPinRightMotor = DirPinRightMotor;
    int PulsePinRightMotor = PulsePinRightMotor;

    Coords coords = {0, 0, M_PI/4};
}


void Robot::moveForward(float distance) {
    float rotations_to_turn = distance/wheel_circumference;
    int steps_to_drive = steps_per_rotation*rotations_to_turn;

    digitalWrite(dirPinLeftMotor, 1); 
    digitalWrite(dirPinRightMotor, 1); 

    //TODO write to pulse pin
    // generatePulseSignal(steps_to_drive);

    coords.x += distance*cos(coords.bearing()); //in radians
    coords.y += distance*sin(coords.bearing()); //in radians
}

void Robot::moveForward(float distance) {
    float rotations_to_turn = distance/wheel_circumference;
    int steps_to_drive = steps_per_rotation*rotations_to_turn;

    digitalWrite(dirPinLeftMotor, 0); 
    digitalWrite(dirPinRightMotor, 0); 

    //TODO write to pulse pin
    // generatePulseSignal(steps_to_drive);

    coords.x -= distance*cos(coords.bearing()); //in radians
    coords.y -= distance*sin(coords.bearing()); //in radians
}


void Robot::rotateCCW(float degrees) {
    float rotations_to_turn = distance/wheel_circumference;
    float arc_length = degrees/360*M_PI*wheel_base
    int steps_to_drive = (int)floor(arc_length*steps_per_rotation/wheel_circumference);


    digitalWrite(dirPinLeftMotor, 0); //left wheel backwards
    digitalWrite(dirPinRightMotor, 1); //right wheel forwards

    //TODO write to pulse pin
    // generatePulseSignal(steps_to_drive);

    coords.bearing += degrees*2*M_PI/360; //in radians
}

void Robot::rotateCW(float degrees) {
    float rotations_to_turn = distance/wheel_circumference;
    float arc_length = degrees/360*M_PI*wheel_base
    int steps_to_drive = (int)floor(arc_length*steps_per_rotation/wheel_circumference);


    digitalWrite(dirPinLeftMotor, 1); //left wheel backwards
    digitalWrite(dirPinRightMotor,0); //right wheel forwards

    //TODO write to pulse pin
    // generatePulseSignal(steps_to_drive);

    coords.bearing -= degrees*2*M_PI/360; //in radians
}


