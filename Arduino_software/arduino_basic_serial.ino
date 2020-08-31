//Basic program to verify that the serial python script works as intended

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);    
    Serial.begin(115200);
}

void loop()
{    
    int input = 0;

    if(Serial.available() > 0)
    {
        input = Serial.parseInt();

        if(input == 1) digitalWrite(LED_BUILTIN, HIGH);        

        else if(input == 2) digitalWrite(LED_BUILTIN, LOW);
        
        else Serial.print(input);        
    }    
}