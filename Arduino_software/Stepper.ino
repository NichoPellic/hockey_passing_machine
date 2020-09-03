#include <Stepper.h>

// Number of steps per internal motor revolution 
const float STEPS_PER_REV = 32; 
 
//  Amount of Gear Reduction
const float GEAR_RED = 64;
 
// Number of steps per geared output rotation
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;
 
// Number of steps to turn one degree
const float STEPS_PER_DEG_OUT_REV = STEPS_PER_OUT_REV / 360;

const int StepOneDeg = STEPS_PER_DEG_OUT_REV;
 
//Setup function for stepper motor
Stepper steppermotor(STEPS_PER_REV, 8, 10, 9, 11);

const byte armingSingal = 2;
 
void setup()
{
    pinMode(armingSingal, INPUT);
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
    //int Coordinates = GetCoordinates();
    int Coordinates = target
    int aimDirection = 0;

    //Needs to remove the while loop so the program isn't stuck trying to set the position
    while (Coordinates != aimDirection)
    {
        if (Coordinates < aimDirection)
        {
            steppermotor.setSpeed(500);
            steppermotor.step(-StepOneDeg);
            aimDirection -= 1;
        }
        if (Coordinates > aimDirection)
        {
            steppermotor.setSpeed(500);
            steppermotor.step(StepOneDeg);
            aimDirection += 1;
        }
    }
    delay(1000);
}

//Motors used to fire the puck
void steMotorSpeed()
{
    
}