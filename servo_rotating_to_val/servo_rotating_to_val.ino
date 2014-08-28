/*
Arduino Servo Test sketch
*/
#include <Servo.h>
Servo servoMain; // Define our Servo
Servo servoMain2; // Define our Servo

int button = 7;
int buttonState = 0;
int angularSpeed = 5;
int rotation = 0;


void setup()
{
   servoMain.attach(10); // servo on digital pin 10
   servoMain2.attach(11); // servo on digital pin 10
   pinMode(button, INPUT); //read button
   Serial.begin(9600);
}

void loop()
{
   //First Motor
   char buffer[] = {' ',' ',' ',' ',' ',' ',' '}; // Receive up to 7 bytes
   while (!Serial.available()); // Wait for characters
   Serial.readBytesUntil('\n', buffer, 7);
   int incomingValue = atoi(buffer);
   Serial.println(incomingValue);
 
   float sensorValue = analogRead(A0);
   rotation = sensorValue/1023*180;
   Serial.println("rotation1: ");
   Serial.println(incomingValue);
   
   
   
   servoMain.write(incomingValue);  // Turn Servo Left to 45 degrees
   delay(10);          // Wait 1 second
   
   
   //Second Motor
   char buffer2[] = {' ',' ',' ',' ',' ',' ',' '}; // Receive up to 7 bytes
   while (!Serial.available()); // Wait for characters
   Serial.readBytesUntil('\n', buffer2, 7);
   incomingValue = atoi(buffer2);
   Serial.println(incomingValue);
   
   sensorValue = analogRead(A0);
   rotation = sensorValue/1023*180;
   Serial.println("rotation2: ");
   Serial.println(incomingValue);
   
   
   
   servoMain2.write(incomingValue);  // Turn Servo Left to 45 degrees
   delay(10);          // Wait 1 second
     
   
     
   
   
}
