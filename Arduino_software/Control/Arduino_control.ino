#include <Stepper.h>
#include <Servo.h>

// Number of steps per internal motor revolution 
const float STEPS_PER_REV = 32; 
 
//  Amount of Gear Reduction
const float GEAR_RED = 64;
 
// Number of steps per geared output rotation
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;
 
// Number of steps to turn one degree
const float STEPS_PER_DEG_OUT_REV = STEPS_PER_OUT_REV / 360;

// Integer value for turning one degree
const int StepOneDeg = STEPS_PER_DEG_OUT_REV;
 
//Setup function for stepper motor
Stepper steppermotor(STEPS_PER_REV, 8, 10, 9, 11);

const byte armingSignal = 2;

// Pin numbers used for servo controll
const int servo1Pin = 3;
const int servo2Pin = 5;

// Creating servo objects
Servo Servo1;
Servo Servo2;
 
void setup()
{
    pinMode(armingSignal, INPUT);
    
    Servo1.attach(servo1Pin);
    Servo2.attach(servo2Pin);
    Servo1.writeMicroseconds(1100);
    Servo2.writeMicroseconds(1100);

    Serial.begin(115200);
}
 
void loop()
{
    int input = 0;

    while(Serial.available() > 0)
    {
        input = Serial.parseInt();

        if(input > 0 && input < 255) setServo(input);        
    }    
}

//Stepper that for yaw
void setStepper(int target)
{
    // Coordinates is the value where the target is standing
    int Coordinates = target;
    // aimDirection is the way the machine is aiming now
    int aimDirection = 0;
    // Absoulute value of degrees needed to reach target
    int degNeeded = abs(Coordinates - aimDirection);

    // Funcktion for aiming the machine
    if (Coordinates < aimDirection)
    {
        steppermotor.setSpeed(500);
        steppermotor.step(-degNeeded);
    }
    else if (Coordinates > aimDirection)
    {
        steppermotor.setSpeed(500);
        steppermotor.step(degNeeded);
    }
}

void spinUpServo() 
{
    const int startValue = 1000;
    int targetValue = 1800;

    int linearAcceleration(int startValue, int targetValue)
    {
        for(int i = startValue; i < targetValue + 1; i++)
        {
            //Maybe some kind of delay here?
            Servo1.writeMicroseconds(i);
            Servo2.writeMicroseconds(i);
        }    
    }
}

//Motors used to fire the puck
void setMotorSpeed()
{
    
}