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

//Starts our aim at 0 degrees
int AimDirection = 0;
 
void setup(){
// Nothing  (Stepper.h liberary sets pins as outputs)
}
 
void loop(){
int Coordinates = GetCoordinates();
  while (Coordinates != AimDirection){
    if (Coordinates < AimDirection){
      steppermotor.setSpeed(500);
      steppermotor.step(-StepOneDeg);
      AimDirection -= 1;
    }
    if (Coordinates > AimDirection){
      steppermotor.setSpeed(500);
      steppermotor.step(StepOneDeg);
      AimDirection += 1;
    }
  }
  delay(1000);
}

int GetCoordinates(){
  int deg = 60;
  return deg; 
}
