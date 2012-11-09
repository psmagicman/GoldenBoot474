/*This code is the robot control code of "Team 8 - Golden boot"
*/

//HEADER FILES
#include <stdio.h>
#include <stdlib.h>
#include <Servo.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <math.h>
#include <iterator>
#include <vector>
#include <pnew.cpp>

#define LOW 0
#define HIGH 1
#define rate 100
#define MOVE 1
#define STANDBY 0
#define RETRACT 72
#define EXTEND 1023
#define FALSE 0 
#define TRUE 1

using namespace std;

vector<int> path;
vector<vector<int> > _path;

//Functions 
void setup();
void Accelerate(int pwm_1, int pwm_2);
void RightTurn(int pwm_1, int pwm_2);
void LeftTurn(int pwm_1, int pwm_2);
void Reverse(int pwm_1, int pwm_2);
void Stop();
void Movement();
void Position();
void Check();

//SENSOR
int sensorPin = A6;   
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
int bufferA = 0;
int bufferB = 0;
int bufferC = 0;
int sensorValAvg = 0;
int SenseDistance = 0;

//Pins needed for the Linear Actuator
int actuator_pin1 = 12;
int actuator_pin2 = 13;
int actuator_input = A7;

// Pin numbers defined here
int motor1_pin_1 = 2; // H-Bridge input pin 1 for Motor 1 (Right)
int motor1_pin_2 = 4; // H-Bridge input pin 2 for Motor 1 
int motor2_pin_1 = 5; // H-Bridge input pin 3 for Motor 2 (Left)
int motor2_pin_2 = 7; // H-bridge input pin 4 for Motor 2
int enablepin_1 = 6; // H-Bridge enable pin for Motor 2
int enablepin_2 = 3; // H-Bridge enable pin for Motor 1
int encoder1_in = A2;
int encoder2_in = A4; 
int dir_1 = A3;
int dir_2 = A5;

int actuator_length; 

volatile int pwm_1 = 255;
volatile int pwm_2 = 255;
//volatile int pos_1 =0;
//volatile int pos_2 =0;
int pos_1 = 0;
int pos_2 = 0;
int abspos_1 =0;
int abspos_2 =0;
int poslist =0;


//Flags
int state = STANDBY;
int input1done = 0;
int input2done = 1;
int poslistFlag = 1;

volatile int enc1_Count=0; 
volatile int enc2_Count=0;
volatile int tempdebug1 =0;
volatile int tempdebug2 =0;

volatile int error;

void setup() {

	Serial.begin(57600);
	Serial.print("Welcome to motor control");
	Serial.print('\n');

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
        //Actuator functions 
        pinMode(actuator_pin1, OUTPUT);
        pinMode(actuator_pin2, OUTPUT);
        pinMode(actuator_input, INPUT);
        digitalWrite(actuator_input, HIGH);
        
        Reset();
        ActuatorControl(RETRACT);
	delay(500);
}


//_____________________________________________________________________________



void loop () 
{
       Sensor();
	if(state == MOVE){
                MotorControl();
        }
	else{
                ReadInput();
        }
} //Close the loop


void CatchtheBall()
{ 
  int caught =0;
  while(caught == 0){
                CheckforE();
                Sensor();
                Serial.println(SenseDistance);
        if(SenseDistance <=30){
                Serial.println("I see the ball");
                Serial.println(SenseDistance);
                Accelerate(pwm_1,pwm_2);
                //Check();
        }
        if(SenseDistance <= 4){
                Serial.println("Ball in the caster .");
                Serial.println(SenseDistance);
                // Ask image processing for all the values 
                // Call kick function
                caught=1;
         }
  }
        //enc1_Count =0;
        //enc2_Count =0;
        Reset();
}

void KicktheBall()
{ 
   Reset();
   int flag1 = 1; 
   int i;  
   int flag = 1; 
      
   //Get instructions from image processing to kick the ball  
   Actuator_Read();
   while (actuator_length < EXTEND )
        {
         CheckforE();
         Actuator_Activate();  
         Actuator_Read();
         flag= 0; 
        }
   delay(300);
   if (flag ==0 )
     {
      Serial.println("Actuator is fully out");                
      pos_1 = 150; 
      pos_2 = 150;  
      abspos_1 =150;
      abspos_2 =150;
      enc1_Count =0;
      enc2_Count =0;
      Serial.println("Accelerate");
      while (enc1_Count < pos_1 && enc2_Count < pos_2)   
           { 
             CheckforE();
             Accelerate(pwm_1,pwm_2); 
             //Position();
             Serial.println();
             Serial.print("Encoder1:  ");
             Serial.print(enc1_Count);
             Serial.print("Encoder2:  ");
             Serial.print(enc2_Count);
           } 
      Stop();
     }           
     //while(flag1 == 1) 
         // {  
     delay(300);  
     Sensor();
     if(SenseDistance >= 4)
       {
        Serial.println("Ball just out of the caster ");
        //  Serial.println(SenseDistance);            
        Actuator_Read();
        while (actuator_length > RETRACT )
             { 
               CheckforE();
               Actuator_Deactivate(); 
               Actuator_Read(); 
               flag = 1; 
             }
         delay(300);
         if (flag ==1)
           {
            Serial.println("Actuator is fully in "); 
            pos_1 = 150; 
            pos_2 = 150; 
            abspos_1 =150;
            abspos_2 =150;
            enc1_Count =0;
            enc2_Count =0;
            pwm_1=255;
            pwm_2=255;
            Serial.println("Reverse ");
            while( enc1_Count < pos_1 && enc2_Count < pos_2) 
                 { 
                   CheckforE();
                   Reverse(pwm_1, pwm_2); 
                   //Position();
                   Serial.println();
                   Serial.print("Encoder1:  ");
                   Serial.print(enc1_Count);
                   Serial.print("Encoder2:  ");
                   Serial.print(enc2_Count);
                 }
          }
         // else 
           // flag1 =1; 
      }          
      // Reverse in order to avaiod touching the same ball  
         //}
      Serial.println("Done with the ball kicking") ;      
      enc1_Count =0;
      enc2_Count =0;
      Reset();
}
