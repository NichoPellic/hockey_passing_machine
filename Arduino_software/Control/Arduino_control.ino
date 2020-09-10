//Puck Buddy V0.1

#include <Stepper.h>
#include <Servo.h>

//I/O Pins
const byte limitSwitch1 = 2;
const byte limitSwitch2 = 3;
const byte signalLed = 4;
const byte esc1Pin = 5;
const byte esc2Pin = 6;
const byte armingSignal = 7;
const byte firingRelay = 8;
const byte stepperPin1 = 10;
const byte stepperPin2 = 11;
const byte stepperPin3 = 12;
const byte stepperPin4 = 13;

//Analog Inputs
int potmeter = A0;

bool manualMode = false;
bool escCalibrated = false;

//Stepper setup
//Number of steps per internal motor revolution 
const float STEPS_PER_REV = 32; 
 
//Amount of Gear Reduction
const float GEAR_RED = 64;
 
//Number of steps per geared output rotation
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;
 
//Number of steps to turn one degree
const float STEPS_PER_DEG_OUT_REV = STEPS_PER_OUT_REV / 360;

//Integer value for turning one degree
const int StepOneDeg = STEPS_PER_DEG_OUT_REV;

//Setup function for stepper motor
Stepper steppermotor(STEPS_PER_REV, stepperPin1, stepperPin2, stepperPin3, stepperPin4);

const byte lowerDegreesLimit = 0;
const byte highestDegreesLimit = 60;

//Current stepper position
byte stepperPosition = 0;

//Current speed sent to the ESC's
int currentMotorSpeed = 0;

//Creating servo objects
Servo ESC1;
Servo ESC2;
 
void setup()
{
    //The delay allows the terminal to start reading the data
    //Applies to VS Code due to the serial monitor using some time before reading the input buffer
    delay(1000);
    //Setup I/O pins
    pinMode(armingSignal, INPUT);
    pinMode(signalLed, OUTPUT);
    pinMode(firingRelay, OUTPUT);

    //Attach servo objects to pins
    ESC1.attach(esc1Pin);
    ESC2.attach(esc2Pin);
    
    Serial.begin(115200);  
    
}
 
void loop()
{
    int input = 0;

    Serial.println("Select mode for Arduino");
    Serial.println("To control from terminal enter    (1)");   
    Serial.println("For the Arduino to run auto enter (2)");   

    while(!Serial.available());

    while(Serial.available() > 0)
    {
        input = Serial.parseInt();

        //Controlled from a terminal
        if(input == 1)
        {
            input = 0;
            Serial.println("Arduino in manual mode!");
            Serial.println("Enter (1) to exit manual mode");
            Serial.println("Enter (2) to activate firing mechanisme");
            Serial.println("Enter (3) to toogle ESC");
            Serial.println("Enter (4) to set target direction");

            manualMode = true;

            int degrees = 0;
            bool spinMotors = false;
            bool runStepper = false;            

            while(true)
            {
                if(spinMotors) setMotorSpeed(0);     
                if(runStepper) runStepper = !setStepper(degrees);           

                if(Serial.available() > 0)               
                {
                    input = Serial.parseInt();

                    //Maybe replace with a switch statement?
                    //Would then have to redo while() loop
                    if(input == 1) break;

                    else if(input == 2) firePuck();

                    else if(input == 3) 
                    
                    {
                        spinMotors = !spinMotors;

                        if(spinMotors) Serial.println("Toggled ESC on");

                        else Serial.println("Toggled ESC off"); 
                    }

                    else if(input == 4)
                    {
                        Serial.print("Enter target value in degrees: ");

                        while(!Serial.available());

                        degrees = Serial.parseInt();

                        Serial.println(degrees);

                        if((degrees >= lowerDegreesLimit) && (degrees <= highestDegreesLimit)) runStepper = true;                    
                    }

                    else if(input == 0) ; //Do nothing, a bug in VS Code sends extra data over the serial line

                    else Serial.println("Please enter a valid value!");                             
                }                
            }
        }   

        //Automatically controlled
        else if(input == 2)  
        {
            input = 0;
            Serial.println("Arduino in auto mode!");

            while(true)
            {
                if(Serial.available() > 0)
                {
                    input = Serial.parseInt();
                    
                    //if(input < 100 && input > 0) setStepper(input);               
                }
            }            
        }  
    }    
}

//Stepper for yaw
bool setStepper(int target)
{
    //Returns true if the stepper has reached target value
    bool returnVal = false;
   
    //Absoulute value of degrees needed to reach target
    int degNeeded = abs(target - stepperPosition);

    // Function for aiming the machine
    if (target < stepperPosition)
    {
        steppermotor.setSpeed(500);
        steppermotor.step(-degNeeded);
    }
    else if (target > stepperPosition)
    {
        steppermotor.setSpeed(500);
        steppermotor.step(degNeeded);
    }

    if((stepperPosition == target) && (armingSignal)) firePuck();
   
    return returnVal;
}

void setMotorSpeed(int targetValue) 
{
    if(!escCalibrated) calibrateESC();

    if(manualMode)
    {
        int speedValue = map(analogRead(potmeter), 0 , 1024, 1000, 2000);
        ESC1.writeMicroseconds(speedValue);
        ESC2.writeMicroseconds(speedValue);
        Serial.print("ESC speedvalue: ");
        Serial.print(speedValue/100);
        Serial.println("%");
    }

    else
    {        
        ESC1.writeMicroseconds(targetValue);
        ESC2.writeMicroseconds(targetValue);           
    }
}

void firePuck()
{
    if(manualMode)
    {
        Serial.println("Firing puck in manual mode");
        digitalWrite(firingRelay, HIGH);
        delay(2000); //For test purpose
        digitalWrite(firingRelay, LOW);
    }

    else
    {
        if(currentMotorSpeed >= 1000)
        {
            digitalWrite(firingRelay, HIGH);
            delay(500);
            digitalWrite(firingRelay, LOW);
        }
    }    
}

void calibrateESC()
{    
    //Set minimum range
    ESC1.writeMicroseconds(1000);
    ESC2.writeMicroseconds(1000);
    delay(10);
    //Set maximum range
    ESC1.writeMicroseconds(2000);
    ESC2.writeMicroseconds(2000);
    delay(10);
    
    //Sets ESC to minimum speed
    ESC1.writeMicroseconds(1000);
    ESC2.writeMicroseconds(1000);

    escCalibrated = true;
}