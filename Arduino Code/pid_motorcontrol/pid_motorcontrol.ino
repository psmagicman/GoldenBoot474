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
void Accelerate(int pwm_1, int pwm_2);
void RightTurn(int pwm_1, int pwm_2);
void LeftTurn(int pwm_1, int pwm_2);
void Reverse(int pwm_1, int pwm_2);
void Stop();
void Movement(int pos_1,int pos_2);
void Position(int pos_1, int pos_2);
void Check();


// Pin numbers defined here
int motor1_pin_1 = 2; // H-Bridge input pin 1 for Motor 1
int motor1_pin_2 = 4; // H-Bridge input pin 2 for Motor 1
int motor2_pin_1 = 5; // H-Bridge input pin 3 for Motor 2
int motor2_pin_2 = 7; // H-bridge input pin 4 for Motor 2
int enablepin_1 = 6; // H-Bridge enable pin for Motor 2
int enablepin_2 = 3; // H-Bridge enable pin for Motor 1
int encoder1_in = A2;
int encoder2_in = A4; 
int dir_1 = A5;
int dir_2 = A3;

//int k = (255/128);
int k = 50;
int pwm_1 = 255;
int pwm_2 = 255;
int pos_1, pos_2; 

volatile int16_t enc1_Count=0; 
volatile int16_t enc2_Count=0;

void setup() {

Serial.begin(9600);

//Initialize all the pins
pinMode(motor1_pin_1, OUTPUT);
pinMode(motor1_pin_2, OUTPUT);
pinMode(motor2_pin_1, OUTPUT);
pinMode(motor2_pin_2, OUTPUT);
pinMode(enablepin_1, OUTPUT);
pinMode(enablepin_2, OUTPUT);

//Initialize encoder pin #1 
pinMode(encoder1_in, INPUT); 
pinMode(dir_1, INPUT);
digitalWrite(encoder1_in, HIGH);
digitalWrite(dir_1, HIGH);
PCintPort::attachInterrupt(encoder1_in, enc1, RISING);
//Initialize encoder pin #2
pinMode(encoder2_in, INPUT);
pinMode(dir_2, INPUT);
digitalWrite(encoder2_in, HIGH);
digitalWrite(dir_2, HIGH);
PCintPort::attachInterrupt(encoder2_in, enc2, RISING);

delay(500);
}

void Accelerate(int pwm_1, int pwm_2) 
{
analogWrite(enablepin_1, pwm_1);
analogWrite(enablepin_2, pwm_2);
digitalWrite(motor1_pin_1, HIGH);
digitalWrite(motor1_pin_2, LOW);
digitalWrite(motor2_pin_1, HIGH);
digitalWrite(motor2_pin_2, LOW);
}

void LeftTurn(int pwm_1, int pwm_2) 
{   
  analogWrite(enablepin_1, pwm_1);
  analogWrite(enablepin_2, pwm_2);
  digitalWrite(motor1_pin_1, HIGH);
  digitalWrite(motor1_pin_2, LOW);
  digitalWrite(motor2_pin_1, LOW);
  digitalWrite(motor2_pin_2, HIGH); 
}

void RightTurn(int pwm_1, int pwm_2) 
{  
  analogWrite(enablepin_1, pwm_1);
  analogWrite(enablepin_2, pwm_2);
  digitalWrite(motor1_pin_1, LOW);
  digitalWrite(motor1_pin_2, HIGH);
  digitalWrite(motor2_pin_1, HIGH);
  digitalWrite(motor2_pin_2, LOW);  
}

void Reverse(int pwm_1, int pwm_2)
{  
  analogWrite(enablepin_1, pwm_1);
  analogWrite(enablepin_2, pwm_2);
  digitalWrite(motor1_pin_1, LOW);
  digitalWrite(motor1_pin_2, HIGH);
  digitalWrite(motor2_pin_1, LOW);
  digitalWrite(motor2_pin_2, HIGH);
  
}

void Stop() 
{
analogWrite(enablepin_1, 0);
analogWrite(enablepin_2, 0);
digitalWrite(motor1_pin_1, HIGH);
digitalWrite(motor1_pin_2, HIGH);
digitalWrite(motor2_pin_1, HIGH); 
digitalWrite(motor2_pin_2, HIGH);
}

//_____________________________________________________________________________

void loop () 
{
  Serial.print('\n'); 
  Serial.print("Begin:  ");
//// Call a function - XBEE

    pos_1 = -8;
    pos_2 = 8;

   {Serial.print('\n');
   Serial.print("Encoder count _1  _2 : ");
   Serial.print(enc1_Count);
   Serial.print("  ");
   Serial.print(enc2_Count);
   Serial.print('\n');
  }
   Movement(pos_1, pos_2);
  {
    Serial.print("Before the check pwm_1  pwm_2 : ");
   Serial.print(pwm_1);
   Serial.print("  ");
   Serial.print(pwm_2);
   Serial.print('\n');
  }
   Check ();
  {
    Serial.print("After the check pwm_1  pwm_2 : ");
   Serial.print(pwm_1);
   Serial.print("  ");
   Serial.print(pwm_2);
  Serial.print('\n');
  }
   Position(pos_1, pos_2);
   {
     Serial.print("Encoder count _1  _2 : ");
   Serial.print(enc1_Count);
   Serial.print("  ");
   Serial.print(enc2_Count);
   Serial.print('\n');
  }
   
}


//Interrupt Function
void enc1()
{// if(digitalRead(dir_1) == LOW )
   enc1_Count++;
 // else 
  // enc1_Count--;
}

void enc2()
{ //if(digitalRead(dir_2) == HIGH )
   enc2_Count++;
  //else
  // enc2_Count--;
}

void Check()
{int diff,k; 
  if(enc1_Count > enc2_Count)
  {diff = (enc1_Count - enc2_Count);
   pwm_2 += diff;
   pwm_1 -= diff; 
  }
  else if(enc1_Count < enc2_Count)
   {diff = (enc2_Count  - enc1_Count); 
  pwm_1 += diff ; 
  pwm_2 -= diff ;
  }
  
  else 
  {pwm_1 += 0 ; 
   pwm_2 -= 0 ;
 }
  if (pwm_1 >= 255)
     pwm_1 = 255;
    if (pwm_2 >= 255)
     pwm_2 = 255;
    if (pwm_1 <= 0)
     pwm_1 = 0;
    if (pwm_2 <= 0)
     pwm_2 = 0;
}

void Movement(int pos_1,int pos_2)
{  
  if (abs(pos_1) == abs(pos_2))
   {if ( pos_1 >= 0 && pos_2 >= 0 ) 
    {Accelerate (pwm_1, pwm_2); 
     Serial.print("Accelerate");
    }
   else if ( pos_1 >= 0 && pos_2 < 0 )
    {RightTurn(pwm_1, pwm_2);
     Serial.print("Right Turn");
    }
   else if ( pos_1 < 0 && pos_2 >= 0 )
    {LeftTurn(pwm_1, pwm_2); 
     Serial.print("Left Turn");
    }
   else if ( pos_1 < 0 && pos_2 < 0 )
    {Reverse(pwm_1,pwm_2);
     Serial.print("Reverse");
    }
   else
    {Stop();
     Serial.print("Stop");
    }
   } 
}

void Position(int pos_1, int pos_2)
{  /*if ((enc1_Count > abs(pos_1) * 0.5 ) && (enc2_Count > abs(pos_2)* 0.5))
    { if ((enc1_Count < abs(pos_1)) && (enc2_Count < abs(pos_2)))
     {Serial.print("HELP:  ");
      Serial.print(abs(pos_1) * 0.5);
      pwm_1 = 100;
      pwm_2 = 100;
     }
    }
    */
  if ((enc1_Count >= abs(pos_1)) && (enc2_Count >= abs(pos_2)))
   {Stop();
    delay(500);
    enc1_Count =0;
    enc2_Count =0;
    pwm_1 = 255;
    pwm_2 = 255;
   }
}

