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

int signalWire = 12;

void setup()
{
   servoMain.attach(11); // servo on digital pin 10
   servoMain2.attach(10); // servo on digital pin 10
   pinMode(4, OUTPUT); // Buzzer pin
   pinMode(signalWire, OUTPUT); // Buzzer pin
   pinMode(button, INPUT); //read button
   digitalWrite(signalWire, LOW);
   Serial.begin(9600);
}

void loop()
{
   int found = 10;
   //First Motor
   char buffer[] = {' ',' ',' ',' ',' ',' ',' '}; // Receive up to 7 bytes
   while (!Serial.available()){
     if(found < 0)
       digitalWrite(signalWire, LOW);
     found -= 1; // Wait for characters
     delay(30);
   }
   Serial.readBytesUntil('\n', buffer, 7);
   int incomingValue = atoi(buffer);
   digitalWrite(signalWire, HIGH);
   /*
   if (incomingValue == 11111) {
      digitalWrite(signalWire, HIGH);
      Serial.println("sending found signal");
      return;
   }   
   if (incomingValue == 00000) {
      digitalWrite(signalWire, LOW);
      return;
   }
   */
   
   int servoNum = incomingValue % 2;
   incomingValue = incomingValue/10;
  
   float sensorValue = analogRead(A0);
   rotation = sensorValue/1023*180;
   Serial.print("rotation");
   Serial.print(servoNum);
   Serial.print(":");
   Serial.println(incomingValue);
   
   /*
   for (int x=0; x<180; x++) {
     // convert degrees to radians then obtain sin value
     float sinVal = (sin(x*(3.1412/180)));
     // generate a frequency from the sin value
     float toneVal = 1000+(int(sinVal*1000));
     tone(5, toneVal);
   }
   
   noTone(5);
   delay(10);
   */

   if(servoNum == 0) {
     if(incomingValue > 132) 
       incomingValue = 132;
     if(incomingValue < 68)
       incomingValue = 68;
     servoMain.write(incomingValue);  // Turn Servo Left to 45 degrees
   }
   if(servoNum == 1) {
     if(incomingValue > 115) 
       incomingValue = 115;
     if(incomingValue < 70)
       incomingValue = 70;
     servoMain2.write(incomingValue);
   }
   delay(10);          // Wait   
   
}

