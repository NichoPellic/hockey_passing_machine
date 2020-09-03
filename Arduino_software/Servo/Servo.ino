#include <Servo.h>

const int servo1Pin = 3;
const int servo2Pin = 5;

const int potmeter = A0;

Servo Servo1;
Servo Servo2;

void setup() 
{    
    Servo1.attach(servo1Pin);
    Servo2.attach(servo2Pin);
    Servo1.writeMicroseconds(1100);
    Servo2.writeMicroseconds(1100);
    
    Serial.begin(115200);
}

void loop() 
{
    int speedValue = analogRead(potmeter);

    speedValue = map(speedValue, 0, 1000, 1000, 2000);

    Serial.println(speedValue);

    Servo1.writeMicroseconds(speedValue);
    Servo2.writeMicroseconds(speedValue);
}