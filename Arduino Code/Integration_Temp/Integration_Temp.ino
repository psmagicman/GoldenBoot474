//HEADER FILES
#include <stdio.h>
#include <stdlib.h>
#include <Servo.h>
//#include <PID_v1.h>
//#include <avr/interrupt.h>
//#include <TimerOne.h>
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
//int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
int bufferA = 0;
int bufferB = 0;
int bufferC = 0;
int sensorValAvg = 0;
int SenseDistance = 0;


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
        Reset();
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
                ReadMotorInput();
        }
} //Close the loop


void CatchtheBall()
{ 
  int caught =0;
  while(caught == 0){
                Sensor();
                Serial.println(SenseDistance);
        if(SenseDistance <=30){
                Serial.println("I see the ball");
                Serial.println(SenseDistance);
                //Accelerate(255,255);
                //Check();
        }
        if(SenseDistance <= 4){
                enc1_Count =0;
                enc2_Count =0;
                pos_1 =10;
                pos_2 =10;
                //Accelerate(255,255);
                //Position();
                Serial.println("I got the ball");
                caught=1;
        }
  }
        enc1_Count =0;
        enc2_Count =0;
        Reset();
}
