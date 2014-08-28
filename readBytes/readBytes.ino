void setup() {
 Serial.begin(9600);
}
 
void loop() {
 char buffer[] = {' ',' ',' ',' ',' ',' ',' '}; // Receive up to 7 bytes
 while (!Serial.available()); // Wait for characters
 Serial.readBytesUntil('\n', buffer, 7);
 int incomingValue = atoi(buffer);
 Serial.println(incomingValue);
}
