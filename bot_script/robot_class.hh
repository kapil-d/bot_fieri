#ifndef ROBOT_H
#define ROBOT_H


class Robot {
    public:

        Robot(int steps_per_rotation, float wheel_circumference, int DirPin, int PulsePin);

        void moveForward(float distance);

        void moveBackward(float distance);

        void rotateLeft(float degrees);

        void rotateRight(float degrees);

    private:
        struct Coords {
            float x;
            float y;
            float bearing;
        };

        int steps_per_rotation;
        float wheel_circumference;
        float wheel_base;
        
        int DirPinLeftMotor;
        int PulsePinLeftMotor;
        int DirPinRightMotor;
        int PulsePinRightMotor;
};

#endif 