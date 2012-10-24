//HEADER FILES
#include <stdio.h>
#include <stdlib.h>
#include <Servo.h>

#include <PID_v1.h>
//#include <avr/interrupt.h>
#include <TimerOne.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <math.h>

#define LOW 0
#define HIGH 1
#define rate 100

//Functions 
void setup();

int incomingByte = 0;

void setup() {
  Serial.begin(9600);
}

//_____________________________________________________________________________



void loop () {
  if(Serial.available() > 0) {
   incomingByte = Serial.read();
  
   Serial.print("incomingByte = ");
   Serial.println(incomingByte, DEC); 
  }
}

