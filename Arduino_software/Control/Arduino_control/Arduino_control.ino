//Puck Buddy V0.1

#include <Stepper.h>
#include <Servo.h>

//I/O Pins
const byte limitSwitch = 2;
const byte batterPower = 3;
const byte signalLed = 4;
const byte esc1Pin = 5;
const byte firingServoPin = 6;
const byte armingSignal = 7;
const byte stepperDirection = 9;
const byte stepperPulse = 10;
//const byte stepperPin1 = 10;
//const byte stepperPin2 = 11;
//const byte stepperPin3 = 12;
//const byte stepperPin4 = 13;

//Analog Inputs
int potmeter = A0;

//Global boolean values
bool headlessMode = true;       //Preferable to short this / make it a digital input
bool manualMode = false;
bool escCalibrated = false;
bool connectBattery = true;   
bool puckLoaded = false; 

unsigned long previousMillis = 0;

//How often variables are written to the serial port
const int printDelay = 1000;

//Fire rate settings
unsigned long lastPuckFired = 0;
const int fireRateDelay = 5000; //Only fires every 5 seconds, even if target is aquired

//Waiting periode for the time between step pulses
const int stepperSpeed = 1;


//Fixed speed settings for the continues servo
const int BACKWARD = 1600;
const int FORWARD = 1400;
const int NEUTRAL = 1500;

//Stepper setup
//Number of steps per internal motor revolution 
//const float STEPS_PER_REV = 32; 
 
//Amount of Gear Reduction
const float GEAR_RED = 4;//0.25;
 
//Full steps
const int FULL_STEPS = 200;

//Half steps
const int HALF_STEPS = 400;

//Number of steps per geared output rotation
//const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;
 
//Number of steps to turn one degree
const float STEPS_PER_DEG_OUT_REV = float((HALF_STEPS / float(360)) * GEAR_RED); // / float(360);

//Integer value for turning one degree
const int StepOneDeg = STEPS_PER_DEG_OUT_REV;

//Lost steps per degree
const float LOST_STEPS= STEPS_PER_DEG_OUT_REV - StepOneDeg;

//Input limits 
const int lowerDegreesLimit = 0;
const int highestDegreesLimit = 360;
const int lowerESCLimit = 1000;
const int highestESCLimit = 2000;

//Current stepper position
volatile int stepperPosition = 0;

//Resets the target value
volatile bool resetTarget = false;

//Compensate for float to int conversion
int additionalSteps = 0;

//Current speed sent to the ESC's
int currentMotorSpeed = 0;

//Time variable for limit switch, used for debounce
volatile unsigned long lastInterrupLimitSwitch = 0;

//Creating servo objects
Servo ESC1;
Servo FiringServo;
 
void setup()
{

    FiringServo.attach(firingServoPin);

    //Attach servo objects to pins
    ESC1.attach(esc1Pin);

    ESC1.writeMicroseconds(1000);

    //Setup I/O pins
    pinMode(signalLed, OUTPUT);
    pinMode(batterPower, OUTPUT);
    pinMode(limitSwitch, INPUT);
    pinMode(armingSignal, INPUT);
    pinMode(stepperDirection, OUTPUT);
    pinMode(stepperPulse, OUTPUT);

    //The signal to the relay is inverted
    digitalWrite(batterPower, HIGH);

    //Limitswitch to calibrate stepper
    attachInterrupt(digitalPinToInterrupt(limitSwitch), stepperLimit, RISING);

    digitalWrite(stepperDirection, HIGH);

    //Set speed stepper
    //steppermotor.setSpeed(10);
    
    Serial.begin(115200);    
    //Set timeout low to prevent Seria.parseInt() from waiting to long
    Serial.setTimeout(10); 

    //If testing the arduino with anything connected headlessMode can be set to true
    //avoiding homing the stepper
    if(headlessMode) 
    {
        Serial.println("Arduino in headless mode, skipping stepper"); 
        return;
    }
    
    Serial.println("Homing stepper...");
    calibrateStepper();    
    Serial.println("Done!");
}
 
void loop()
{
    int input = 0;

    Serial.println("Select mode for Arduino");
    Serial.println("Enter (1) to control from terminal");   
    Serial.println("Enter (2) to run in auto mode");   

    //Waits for serial connection
    while(!Serial.available());

    //If serial data available
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
            Serial.println("Enter (5) to toggle battery to ESC");
            Serial.println("Enter (6) to load puck");

            manualMode = true;

            int degrees = 0;
            int steps = 0;
            bool spinMotors = false;
            bool runStepper = false;    

            while(true)
            {
                if(resetTarget) runStepper = false, resetTarget = false;

                if(spinMotors) setMotorSpeed(0);     

                if(runStepper) setStepper(steps);               

                if(Serial.available() > 0)               
                {
                    input = Serial.parseInt();

                    //Exits the while loop
                    if(input == 1) break;

                    //Activate firing mechanism
                    else if(input == 2) firePuck();

                    //Toggles the ESC's
                    else if(input == 3) 
                    
                    {
                        spinMotors = !spinMotors;

                        if(spinMotors) Serial.println("Toggled ESC on");

                        else Serial.println("Toggled ESC off"); 
                    }

                    //Set stepper position
                    else if(input == 4)
                    {
                        Serial.print("Enter target value in degrees: ");

                        while(!Serial.available());

                        degrees = Serial.parseInt();

                        Serial.println(degrees);

                        if((degrees >= lowerDegreesLimit) && (degrees <= highestDegreesLimit))
                        {
                            runStepper = true;
                            float requiredSteps = degrees * StepOneDeg;
                            steps = int(requiredSteps);
                            additionalSteps = LOST_STEPS * degrees;   
                        }   
                    }

                    //Activate battery relay
                    else if(input == 5)
                    {
                        connectBattery = !connectBattery;    
                        escCalibrated = false;                    
                        digitalWrite(batterPower, connectBattery);                        

                        if(!connectBattery) Serial.println("Battery connected!");

                        else Serial.println("Battery disconnected!");
                    }

                    //Load a new puck to fire
                    else if (input == 6) loadPuck();
                    

                    //Do nothing, a bug in VS Code sends extra data over the serial line                
                    else if(input == 0) ; 
                    
                    else Serial.println("Please enter a valid value!");                             
                }                
            }
        }   

        //Automatically controlled
        else if(input == 2)  
        {
            Serial.println("Arduino in auto mode!");

            digitalWrite(signalLed, HIGH);
            connectBattery = false;

            int degrees = 0;
            int motorSpeed = 0;
            int steps = 0;
            bool runStepper = false;

            calibrateESC();
            String inputString = "";            

            while(true)
            {         
                //Don't run stepper if it isn't homed       
                if(resetTarget) runStepper = false, resetTarget = false;

                ESC1.writeMicroseconds(1050);
                
                if(runStepper) setStepper(steps);

                if(Serial.available() > 0)
                {
                    //degrees = Serial.parseInt();
                    inputString = Serial.readStringUntil(";");
                    
                    const int arraySize = 10;
                    char inputArray[arraySize];   
                    byte parameter = 0; 

                    inputString.toCharArray(inputArray, arraySize);

                    String degreesString = "";
                    String motorSpeedString = "";

                    //Parse input message
                    for(int i = 0; i < arraySize; i++)
                    {
                        if(inputArray[i] == ',') parameter++, i++;

                        else if(inputArray[i] == ';') break;

                        switch (parameter)
                        {
                        case 0:
                            degreesString += inputArray[i];
                            break;
                        
                        case 1:
                            motorSpeedString += inputArray[i];
                            break;
                        default:                        
                            break;
                        }
                    }

                    degrees = degreesString.toInt();
                    motorSpeed = motorSpeedString.toInt();

                    //Sets the motorspeed
                    //if((motorSpeed >= lowerESCLimit) && (motorSpeed <= highestESCLimit)) setMotorSpeed(1050);
                    //setMotorSpeed(1050);

                    //Turns the stepper in target direction
                    if((degrees >= lowerDegreesLimit) && (degrees <= highestDegreesLimit))
                    {
                        runStepper = true;

                        //Calcluate the delta of movement instead of absoulut based on pervious position
                        steps = degrees * StepOneDeg;
                        additionalSteps = LOST_STEPS * degrees;
                    }                                      
                }
            }
        }
    }    
}

//Stepper for yaw
//Target given in steps
void setStepper(int target)
{    
    //Serial.print("Stepper target: ");
    //Serial.println(target + additionalSteps);

    //Sets stepper direction
    if (target + additionalSteps < stepperPosition) digitalWrite(stepperDirection, LOW), stepperPosition--;  
       
    else if (target + additionalSteps > stepperPosition) digitalWrite(stepperDirection, HIGH), stepperPosition++;    

    //Moves one step
    if((target + additionalSteps) != stepperPosition) moveStepper();

    //if((digitalRead(armingSignal)) && ((target + additionalSteps) == stepperPosition)) firePuck();  
    if(((target + additionalSteps) == stepperPosition)) firePuck();  
}

void setMotorSpeed(int targetValue) 
{
    if(!escCalibrated) calibrateESC();

    if(escCalibrated)
    {
        if(manualMode)
        {
            //int speedValue = map(analogRead(potmeter), 0 , 1024, lowerESCLimit, highestESCLimit);
            int speedValue = 1050;
            ESC1.writeMicroseconds(speedValue);

            if(millis() - previousMillis >= printDelay)
            {
                previousMillis = millis();
                Serial.print("ESC speedvalue: ");
                Serial.print((speedValue-1000)/10);
                Serial.println("%");
            }        
        }

        else
        {        
            ESC1.writeMicroseconds(targetValue);
        }
    }    
}


void firePuck()
{       
    if(millis() - lastPuckFired >= fireRateDelay)
    {
        loadPuck(); //Fires the puck
        lastPuckFired = millis();
    }
}

void calibrateESC()
{  
    if(!connectBattery)
    {
        

        Serial.println("Calibrating ESC..."); 
        
        //Set minimum range for ESC
        ESC1.writeMicroseconds(lowerESCLimit);
        //ESC2.writeMicroseconds(lowerESCLimit);

        //Allow the ESC to registrer new min value
        delay(1000);

        //Set maximum range for ESC
        ESC1.writeMicroseconds(highestESCLimit);
        //ESC2.writeMicroseconds(highestESCLimit);

        //Allow the ESC to registrer new max value
        delay(10);
        
        //Sets ESC to minimum speed
        ESC1.writeMicroseconds(lowerESCLimit);
        //ESC2.writeMicroseconds(lowerESCLimit);

        delay(5000);

        escCalibrated = true;

        Serial.println("ESC calibrated!");   
    }

    else Serial.println("Battery not connected, please turn on battery in settings and retry");
}

void calibrateStepper()
{   
    //Go left until limit switch is hit     
    while(!resetTarget) moveStepper();
}


//ISR function
void stepperLimit()
{
    //De-bounce
    if((millis() - lastInterrupLimitSwitch) > 50)
    {
        stepperPosition = 0;
        resetTarget = true;
        lastInterrupLimitSwitch = millis();
    }    
}

void moveStepper()
{
    digitalWrite(stepperPulse, HIGH);
    delay(3);
    digitalWrite(stepperPulse, LOW);
}

void loadPuck()
{
    //Move servo all the way back and load new puck
    FiringServo.writeMicroseconds(BACKWARD);
    delay(550);
    //Limit switch trigger
    FiringServo.writeMicroseconds(FORWARD);
    delay(600);    
    FiringServo.writeMicroseconds(NEUTRAL);
    puckLoaded = true;
}