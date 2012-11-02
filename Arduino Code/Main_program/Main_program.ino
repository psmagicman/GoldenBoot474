/*
This code combines Motor + Pid + Sensor + Linear Actuator  
*/
//HEADER FILES
#include <stdio.h>
#include <stdlib.h>
#include <Servo.h>
#include <PID_v1.h>
#include <TimerOne.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <math.h>

// Variable definition 
#define LOW 0
#define HIGH 1
#define PWM_MAX 255

//Function declaration

void setup();
void enc1();
void enc2();
void Check();
void Movement();
void Position();
void ReadXbee();

// Pin number definition 
int motor1_pin_1 = 2; // H-Bridge input pin 1 for Motor 1
int motor1_pin_2 = 4; // H-Bridge input pin 2 for Motor 1
int motor2_pin_1 = 5; // H-Bridge input pin 1 for Motor 2
int motor2_pin_2 = 7; // H-bridge input pin 2 for Motor 2
int enablepin_1 = 6; // H-Bridge enable pin for Motor 1
int enablepin_2 = 3; // H-Bridge enable pin for Motor 2
int encoder1_in = A2;  // Encoder clock cycle input from Motor 1
int encoder2_in = A4;  // Encoder clock cycle input from Motor 2
int dir_1 = A5;  // Encoder direction input from Motor 1
int dir_2 = A3;  // Encoder direction input from Motor 2
int sensorPin = A6; //Sensor Pin
//Pins needed for the Linear Actuator
int actuator_pin1 = ;
int actuator_pin2 = ;
int enable_actuator = ;
int actuator_input = ; 

//Variable declaration
int pwm_1 = 255; 
int pwm_2 = 255;
volatile int pos_1 =0;
volatile int pos_2 =0;
volatile int abspos_1 =0;
volatile int abspos_2 =0;
volatile int state =0;
int input1done = 0; //Flags for reading the input from Xbee
int input2data = 1; //Flags for reading the input from Xbee

//Variables for Sensors 
int sensorValue = 0;  // variable to store the value coming from the sensor
int bufferA = 0;
int bufferB = 0;
int bufferC = 0;
int sensorValAvg = 0;
int distance = 0;

volatile int16_t enc1_Count=0; 
volatile int16_t enc2_Count=0;
volatile int16_t tempdebug1 =0;
volatile int16_t tempdebug2 =0;

//---------------------------------------------------------- Setup
void setup() 
{Serial.begin(57600);
//Initialize all the Motor pins
pinMode(motor1_pin_1, OUTPUT);
pinMode(motor1_pin_2, OUTPUT);
pinMode(motor2_pin_1, OUTPUT);
pinMode(motor2_pin_2, OUTPUT);
pinMode(enablepin_1, OUTPUT);
pinMode(enablepin_2, OUTPUT);

//Initializing the encoder #1 pins
pinMode(encoder1_in, INPUT); 
pinMode(dir_1, INPUT);
digitalWrite(encoder1_in, HIGH);
digitalWrite(dir_1, HIGH);
PCintPort::attachInterrupt(encoder1_in, enc1, RISING);

//Initializing the encoder #2 pins
pinMode(encoder2_in, INPUT);
pinMode(dir_2, INPUT);
digitalWrite(encoder2_in, HIGH);
digitalWrite(dir_2, HIGH);
PCintPort::attachInterrupt(encoder2_in, enc2, RISING);

//Initializing the sensor pins
//pinMode(ledPin, OUTPUT); 
pinMode(sensorPin, INPUT); 

//Initializing the linear actuator pins
pinMode(actuator_pin1, OUTPUT);
pinMode(actuator_pin2, OUTPUT);
pinMode(enable_actuator, OUTPUT); 
pinMode(actuator_input, INPUT);
digitalWrite(actuator_input, HIGH);

delay(500);
}

//------------------------------------------------------ Interrupt functions
void enc1()
{// if(digitalRead(dir_1) == LOW )
   enc1_Count++;
   Position();
 // else 
  // enc1_Count--;
}

void enc2()
{ //if(digitalRead(dir_2) == HIGH )
   enc2_Count++;
   Position();
  //else
  // enc2_Count--;
}

//--------------------------------------------------------------- PWM check
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

//----------------------------------------------------------Motor Movement 
void Movement()
{  
  if (abspos_1 == abspos_2)
    {
   if ( pos_1 > 0 && pos_2 > 0 ) 
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
   else
   {
   Stop();
   state=0;
   }
}

//----------------------------------------------------------------Position check
void Position()
{  /*if ((enc1_Count > abs(pos_1) * 0.5 ) && (enc2_Count > abs(pos_2)* 0.5))
    { if ((enc1_Count < abs(pos_1)) && (enc2_Count < abs(pos_2)))
     {Serial.print("HELP:  ");
      Serial.print(abs(pos_1) * 0.5);
      pwm_1 = 100;
      pwm_2 = 100;
     }
    }
    */
  if ((enc1_Count >= abspos_1) && (enc2_Count >= abspos_2))
   {Stop();
    tempdebug1 = enc1_Count;
    tempdebug2 = enc1_Count;  
    enc1_Count =0;
    enc2_Count =0;
    pwm_1 = 255;
    pwm_2 = 255;
    pos_1 = 0;
    pos_2 = 0;
    state = 0;
   }
}

//-------------------------------------------------------------------Sensor functions

// ------------------------------------------Reading values from Xbee
void ReadXbee()
{
   Serial.print('\n');
   Serial.print("Reading position values from Xbee ");
     if((Serial.available() == 5) && (input2data == 1 )){
    char bytes1[5];
    bytes1[0] = Serial.read();  
    bytes1[1] = Serial.read();
    bytes1[2] = Serial.read();
    bytes1[3] = Serial.read();
    bytes1[4] = Serial.read();
   // bytes1[5] = '\0';
    pos_1 = atoi(bytes1);
    abspos_1 = abs(pos_1);
    Serial.print("pos_1 = ");
    Serial.print(pos_1, DEC);
    Serial.println();
    input1done =1;
    input2data=0;
}
  if((Serial.available() == 5) && (input1done == 1)){
  char bytes2[5];
  bytes2[0] = Serial.read();  
  bytes2[1] = Serial.read();
  bytes2[2] = Serial.read();
  bytes2[3] = Serial.read();
  bytes2[4] = Serial.read();
 // bytes2[5] = '\0';
  pos_2 = atoi(bytes2);
  abspos_2 = abs(pos_2);
  Serial.print("pos_2 = ");
  Serial.print(pos_2, DEC);
  Serial.println();
  state = 1;
  input1done = 0;
  input2data = 1;
  Serial.print(state,DEC );
  Serial.println();  
}
}

//-----------------------------------------------------------Main Loop 
void loop () 
{
  while( state == 0){
    ReadXbee();
   }
   
  while(state == 1)
 {    
  Serial.print(tempdebug1);
  Serial.print(tempdebug2);
  Serial.print('\n'); 
  Serial.print("Begin:  ");
 
  {Serial.print('\n');
   Serial.print("Encoder count _1  _2 : ");
   Serial.print(enc1_Count);
   Serial.print("  ");
   Serial.print(enc2_Count);
   Serial.print('\n');
  }
   Movement();
  {
    Serial.print("Pos1 Pos2 : ");
   Serial.print(pos_1);
   Serial.print("  ");
   Serial.print(pos_2);
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
   {
     Serial.print("Encoder count _1  _2 : ");
   Serial.print(enc1_Count);
   Serial.print("  ");
   Serial.print(enc2_Count);
   Serial.print('\n');
  }
 }
}

