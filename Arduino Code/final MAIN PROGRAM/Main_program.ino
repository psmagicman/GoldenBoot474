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


#define DEBUG 1
#define LOW 0
#define HIGH 1
#define rate 100
#define MOVE 1
#define STANDBY 0
#define RETRACT 72
#define EXTEND 1023
#define FALSE 0 
#define TRUE 1
#define RIGHTONE 1
#define RIGHTTWO 2
#define LEFTONE 3
#define LEFTTWO 4
#define MIDDLE 5
#define TENNISBALL 25
#define ON 1
#define OFF 0
#define GRABFLAG 11111
#define KICKFLAG 22222


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
int actuator_pin1 = 13;
int actuator_pin2 = 12;
int actuator_input = A7;
int actuator_enable = 11;

// Pin numbers defined here
int motor1_pin_1 = 2; // H-Bridge input pin 1 for Motor 1 (Right)
int motor1_pin_2 = 4; // H-Bridge input pin 2 for Motor 1 
int motor2_pin_1 = 5; // H-Bridge input pin 3 for Motor 2 (Left)
int motor2_pin_2 = 7; // H-bridge input pin 4 for Motor 2
int enablepin_1 = 6; // H-Bridge enable pin for Motor 1
int enablepin_2 = 3; // H-Bridge enable pin for Motor 2
int encoder1_in = A4;
int encoder2_in = A2; 
int dir_1 = A5;
int dir_2 = A3;

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
int slowdown1 =0;
int slowdown2 =0;


//Flags
int state = STANDBY;
int input1done = 0;
int input2done = 1;
int poslistFlag = 1;
int emergency = 0;
int GrabDir =0;

volatile int enc1_Count=0; 
volatile int enc2_Count=0;
volatile int tempdebug1 =0;
volatile int tempdebug2 =0;

volatile int error1;
volatile int error2;
volatile int sumError1 =0;
volatile int sumError2 =0;
int KI = 5;
int KP = 70;
int motor = 0;

void setup() {

	Serial.begin(57600);
	log1("Welcome to motor control");
	log1('\n');

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
	PCintPort::attachInterrupt(encoder1_in, enc1, CHANGE);
	//Initialize encoder pin #2
	pinMode(encoder2_in, INPUT);
	pinMode(dir_2, INPUT);
	digitalWrite(encoder2_in, HIGH);
	digitalWrite(dir_2, HIGH);
	PCintPort::attachInterrupt(encoder2_in, enc2, CHANGE);
        //Actuator functions 
        pinMode(actuator_enable, OUTPUT);
        pinMode(actuator_pin1, OUTPUT);
        pinMode(actuator_pin2, OUTPUT);
        pinMode(actuator_input, INPUT);
        digitalWrite(actuator_input, HIGH);
        digitalWrite(actuator_enable, LOW);
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



void KicktheBall()
{ 
   //Reset();
   int flag1 = 1; 
   int i;  
   int flag = 0; 
   CheckforE();
   //Get instructions from image processing to kick the ball  
   digitalWrite(actuator_enable, HIGH); 
   Actuator_Read();
   while (actuator_length < EXTEND )
        {
         if( CheckforE() == TRUE){
            flag = 1;
            break;
         }
         Actuator_Activate();  
         Actuator_Read();
         flag= 0; 
        }
   //delay(300);
      Sensor();
      Sensor();
      Sensor();
     if(SenseDistance >= 4 && flag == 0)
       {
         logln("Actuator is fully in "); 
         pos_1 = 150; 
         pos_2 = 150; 
         abspos_1 =150;
         abspos_2 =150;
         enc1_Count =0;
         enc2_Count =0;
         pwm_1=255;
         pwm_2=255;
         logln("Accelerate ");
         while( enc1_Count < pos_1 && enc2_Count < pos_2) 
         { 
           if( CheckforE() == TRUE){ 
             flag = 1;
             break;
           }
           Accelerate(pwm_1, pwm_2); 
           //Position();
           logln(' ');
           log1("Encoder1:  ");
           log1(enc1_Count);
           log1("Encoder2:  ");
           log1(enc2_Count);
          }
          delay(300);
          pos_1 = 150; 
          pos_2 = 150; 
          pwm_1=255;
          pwm_2=255; 
          abspos_1 = 150;
          abspos_2 = 150;
          enc1_Count =0;
          enc2_Count =0;
          if (flag ==0)
            {
             logln("Ball just out of the caster ");
             //  logln(SenseDistance);            
             Actuator_Read();
             while (actuator_length > RETRACT && flag == 0 )
             { 
               if( CheckforE() == TRUE){ 
                   flag =1; 
                   break;
               }
               Actuator_Deactivate(); 
               Actuator_Read(); 
               //flag = 0; 
             }
             while (enc1_Count < pos_1 && enc2_Count < pos_2 && flag == 0)   
             { 
               if( CheckforE() == TRUE){ 
                  flag = 1;
                  break;
               }
               Reverse(pwm_1,pwm_2); 
               //delay(300); 
            }
            } 
      }
      logln("Done with the ball kicking") ;   
      Serial.print(4);   
      enc1_Count =0;
      enc2_Count =0;
      digitalWrite(actuator_enable, LOW); 
      //Reset();
}






void log1(String message)
{
	if(DEBUG)
		Serial.print(message);
}

void log1(int value)
{
  if(DEBUG)
    Serial.print(value);
}

void log1(char character)
{
  if(DEBUG)
    Serial.print(character);
}

void logln(String message)
{
	if(DEBUG)
		Serial.println(message);
}

void logln(int value)
{
  if(DEBUG)
    Serial.println(value);
}

void logln(char character)
{
  if(DEBUG)
    Serial.println(character);
}
