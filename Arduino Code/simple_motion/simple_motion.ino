/* 
Simple code for turning the wheels 
*/

//HEADER FILES
#include <stdio.h>
#include <stdlib.h>
#include <Servo.h>
#include <PID_v1.h>

#define LOW 0
#define HIGH 1
#define rate 100

//Functions 
void setup();
void LeftTurn(int pwm_control); 
void RightTurn(int pwm_control);
void Accelerate(int pwm_control);
void Reverse(int pwm_control);
void Stop();

// Pin numbers defined here
int motor1_pin_1 = 5; // H-Bridge input pin 1 for Motor 1
int motor1_pin_2 = 7; // H-Bridge input pin 2 for Motor 1
int motor2_pin_1 = 2; // H-Bridge input pin 3 for Motor 2
int motor2_pin_2 = 4; // H-bridge input pin 4 for Motor 2
int enablepin_1 = 6; // H-Bridge enable pin for Motor 2
int enablepin_2 = 3; // H-Bridge enable pin for Motor 1
int encoder1_in = A5;
//int encoder2_in = A; 

void setup() {
  //Initialize all the pins
  Serial.begin(9600);
  pinMode(motor1_pin_1, OUTPUT);
  pinMode(motor1_pin_2, OUTPUT);
  pinMode(motor2_pin_1, OUTPUT);
  pinMode(motor2_pin_2, OUTPUT);
  pinMode(enablepin_1, OUTPUT);
  pinMode(enablepin_2, OUTPUT);
  pinMode(encoder1_in, INPUT);  
  pinMode(encoder2_in, INPUT); 
  delay(500);
}

void LeftTurn(int pwm_control) {
  
  analogWrite(enablepin_1);
  analogWrite(enablepin_2);
  digitalWrite(motor1_pin_1, LOW);
  digitalWrite(motor1_pin_2, HIGH);
  digitalWrite(motor2_pin_1, HIGH);
  digitalWrite(motor2_pin_2, LOW);
   
}
void RightTurn(int pwm_control){
   
  analogWrite(enablepin_1);
  analogWrite(enablepin_2);
  digitalWrite(motor1_pin_1, HIGH);
  digitalWrite(motor1_pin_2, LOW);
  digitalWrite(motor2_pin_1, LOW);
  digitalWrite(motor2_pin_2, HIGH);
  
}

void Reverse(int pwm_control) {
  
  analogWrite(enablepin_1);
  analogWrite(enablepin_2);
  digitalWrite(motor1_pin_1, LOW);
  digitalWrite(motor1_pin_2, HIGH);
  digitalWrite(motor2_pin_1, LOW);
  digitalWrite(motor2_pin_2, HIGH);
  
}

void Accelerate(int pwm_control) {
  
  analogWrite(enablepin_1);
  analogWrite(enablepin_2);
  digitalWrite(motor1_pin_1, HIGH);
  digitalWrite(motor1_pin_2, LOW);
  digitalWrite(motor2_pin_1, HIGH);
  digitalWrite(motor2_pin_2, LOW);
  
}

void Stop() {
  
  analogWrite(enablepin_1, 0);
  analogWrite(enablepin_2, 0);
  digitalWrite(motor1_pin_1, HIGH);
  digitalWrite(motor1_pin_2, HIGH);
  
}

//_____________________________________________________________________________
void loop ()
{
 Accelerate (255); 
 delay(2500);
 Stop();
 delay(2500);
}


