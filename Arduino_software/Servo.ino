#include <servo.h>

int servo1Pin = 3
int servo2Pin = 4

Servo Servo1;
Servo Servo2;

void setup() {
    Servo1.attach(servo1Pin);
    Servo2.attach(servo2Pin);
    Servo1.writeMicroseconds(1100);
    Servo2.writeMicroseconds(1100)
}

void loop() {

}