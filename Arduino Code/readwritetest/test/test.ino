//HEADER FILES
#include <stdio.h>
#include <stdlib.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define BAUD_RATE 9600

//Functions 
void setup();

int incomingByte = 0;
String readString;
void setup() {
  Serial.begin(BAUD_RATE);
}

//_____________________________________________________________________________



void loop () {
  char bytes[5];
  while(Serial.available()) {
   delay(10);
   if(Serial.available() == 5) {
     for(int i = 0; i < 5; i++) {
      bytes[i] = Serial.read(); 
     }
   }
  }
  
  if(readString.length() > 0) {
   Serial.write(bytes);
   for(int j = 0; j < 5; j++) {
     bytes[j] = ' ';
   }
  }
}

