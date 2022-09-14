#include <Servo.h>

Servo xServo;
Servo yServo;
byte inputBuffer[4];
int xCoord;
int yCoord;

void setup() {

    Serial.setTimeout(10);
    xCoord = 90;
    yCoord = 90;
    xServo.attach(8);
    yServo.attach(7);
    Serial.begin(9600);
    xServo.write(xCoord);
    yServo.write(yCoord);
    char receivedChar;

}

void loop() {

    if (Serial.available() > 0){
        xCoord = Serial.parseInt();
        yCoord = Serial.parseInt();
        xServo.write(xCoord);
        yServo.write(yCoord);
        Serial.flush();
    }

}
