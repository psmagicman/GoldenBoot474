//HEADER FILES
#include <stdio.h>
#include <stdlib.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define BAUD_RATE 57600

//Functions 
void setup();

int incomingByte = 0;
String readString;
void setup() {
  Serial.begin(BAUD_RATE);
}

//_____________________________________________________________________________



void loop () {
  char bytes;
  while(Serial.available()) {
   if(Serial.available() == 1) {
     bytes = Serial.read(); 
   }
   delay(500);
   Serial.print(bytes);
  }
  
  //if(strlen(bytes)() > 0) {
   /*for(int j = 0; j < 5; j++) {
     bytes[j] = ' ';
   }
  }*/
}

