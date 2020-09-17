//Puck Buddy V0.1

#include <Stepper.h>
#include <Servo.h>

//I/O Pins
const byte limitSwitch = 2;
const byte batterPower = 3;
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

bool headlessMode = true;
bool manualMode = false;
bool escCalibrated = false;
bool connectBattery = true;    

unsigned long previousMillis = 0;

const int printDelay(1000);

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

//Lost steps per degree
const float LOST_STEPS= STEPS_PER_DEG_OUT_REV - StepOneDeg;

//Setup function for stepper motor
Stepper steppermotor(STEPS_PER_OUT_REV, stepperPin1, stepperPin3, stepperPin2, stepperPin4);

//Input limits 
const int lowerDegreesLimit = 0;
const int highestDegreesLimit = 360;

//Current stepper position
volatile int stepperPosition = 0;

//Resets the target value
volatile bool resetTarget = false;

//Compensate for float to int conversion
int additionalSteps = 0;

//Current speed sent to the ESC's
int currentMotorSpeed = 0;

volatile unsigned long lastInterrupLimitSwitch = 0;

//Creating servo objects
Servo ESC1;
Servo ESC2;
 
void setup()
{
    //Setup I/O pins
    pinMode(signalLed, OUTPUT);
    pinMode(firingRelay, OUTPUT);
    pinMode(batterPower, OUTPUT);
    pinMode(limitSwitch, INPUT);
    pinMode(armingSignal, INPUT);

    //The signal to the relay is inverted
    digitalWrite(batterPower, HIGH);

    //Limitswitch to calibrate stepper
    attachInterrupt(digitalPinToInterrupt(limitSwitch), stepperLimit, RISING);

    //Set speed stepper
    steppermotor.setSpeed(10);
    
    Serial.begin(115200);    
    //Set timeout low to prevent Seria.parseInt() from waiting to long
    Serial.setTimeout(10);  

    if(headlessMode) return;
    
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
            Serial.println("Enter (5) to toggle battery to ESC");

            manualMode = true;

            int degrees = 0;
            int steps = 0;
            bool spinMotors = false;
            bool runStepper = false;    

            while(true)
            {
                if(resetTarget) 
                {
                    runStepper = false;
                    resetTarget = false;
                }

                if(spinMotors) setMotorSpeed(0);     
                if(runStepper) setStepper(steps);               

                if(Serial.available() > 0)               
                {
                    input = Serial.parseInt();

                    //Exits the while loop
                    if(input == 1) break;

                    //Activate firing mechanism
                    else if(input == 2) firePuck();

                    //
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

                        if((degrees >= lowerDegreesLimit) && (degrees <= highestDegreesLimit))
                        {
                            runStepper = true;
                            steps = degrees * StepOneDeg;
                            additionalSteps = LOST_STEPS * degrees;
                        }   
                    }

                    else if(input == 5)
                    {
                        connectBattery = !connectBattery;    
                        escCalibrated = false;                    
                        digitalWrite(batterPower, connectBattery);                        

                        if(!connectBattery) Serial.println("Battery connected!");

                        else Serial.println("Battery disconnected!");
                    }

                    else if(input == 0) ; //Do nothing, a bug in VS Code sends extra data over the serial line

                    else Serial.println("Please enter a valid value!");                             
                }                
            }
        }   

        //Automatically controlled
        else if(input == 2)  
        {
            int degrees = 0;
            int steps = 0;
            bool runStepper = false;
            Serial.println("Arduino in auto mode!");

            while(true)
            {                
                if(resetTarget)
                {
                   runStepper = false;
                   resetTarget = false;
                }
                
                if(runStepper) setStepper(steps);

                if(Serial.available() > 0)
                {
                    digitalWrite(signalLed, HIGH);

                    degrees = Serial.parseInt();

                    if((degrees >= lowerDegreesLimit) && (degrees <= highestDegreesLimit))
                    {
                        runStepper = true;
                        steps = degrees * StepOneDeg;
                        additionalSteps = LOST_STEPS * degrees;
                    }               

                    digitalWrite(signalLed, LOW);
                }
            }            
        }  
    }    
}

//Stepper for yaw
//Target given in steps
void setStepper(int target)
{       
    //Function for aiming the machine
    //Only moves one step each loop to allow for new coordinates
    if (target + additionalSteps < stepperPosition)
    {        
        steppermotor.step(-1);
        stepperPosition -= 1;
    }
    else if (target + additionalSteps > stepperPosition)
    {
        steppermotor.step(1);
        stepperPosition += 1;        
    }

    if(armingSignal && target == stepperPosition) firePuck();  
}

void setMotorSpeed(int targetValue) 
{
    if(!escCalibrated) calibrateESC();

    if(escCalibrated)
    {
        if(manualMode)
        {
            int speedValue = map(analogRead(potmeter), 0 , 1024, 1000, 2000);
            ESC1.writeMicroseconds(speedValue);
            ESC2.writeMicroseconds(speedValue);

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
            ESC2.writeMicroseconds(targetValue);           
        }
    }    
}

void firePuck()
{
    if(manualMode)
    {
        digitalWrite(firingRelay, HIGH);
        delay(500); //For test purpose
        digitalWrite(firingRelay, LOW);
    }

    else
    {
        //Seems redundant?
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
    if(!connectBattery)
    {
        //Attach servo objects to pins
        ESC1.attach(esc1Pin);
        ESC2.attach(esc2Pin);

        Serial.println("Calibrating ESC..."); 
        //Set minimum range for ESC
        ESC1.writeMicroseconds(1000);
        ESC2.writeMicroseconds(1000);
        delay(500);
        //Set maximum range for ESC
        ESC1.writeMicroseconds(2000);
        ESC2.writeMicroseconds(2000);
        delay(500);
        
        //Sets ESC to minimum speed
        ESC1.writeMicroseconds(1000);
        ESC2.writeMicroseconds(1000);

        escCalibrated = true;

        Serial.println("ESC calibrated!");   
    }

    else Serial.println("Battery not connected, please turn on battery in settings and retry");
}

void calibrateStepper()
{   
    //Go left untill limit switch is hit     
    while(!resetTarget) steppermotor.step(-1);
}

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