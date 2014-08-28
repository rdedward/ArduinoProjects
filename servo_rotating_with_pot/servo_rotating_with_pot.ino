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
   if (digitalRead(button) == HIGH) {
   float sensorValue = analogRead(A0);
   rotation = sensorValue/1023*180;
   Serial.println(rotation);
   
   
   
   servoMain.write(rotation);  // Turn Servo Left to 45 degrees
   delay(10);          // Wait 1 second
   
   
   }
   else{
     float sensorValue = analogRead(A0);
   rotation = sensorValue/1023*180;
   Serial.println(rotation);
   
   
   
   servoMain2.write(rotation);  // Turn Servo Left to 45 degrees
   delay(10);          // Wait 1 second
     
   }
     
   
   
}
