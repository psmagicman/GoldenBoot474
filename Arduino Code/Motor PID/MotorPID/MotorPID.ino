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

int state =0;

int input1done = 0;
int input2data = 1;
int poslistFlag = 1;

volatile int enc1_Count=0; 
volatile int enc2_Count=0;
volatile int tempdebug1 =0;
volatile int tempdebug2 =0;

//int KP = 1;
//int KI = 0;
//int KD = 1;
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
	digitalWrite(enablepin_1, HIGH);
	digitalWrite(enablepin_2, HIGH);
	digitalWrite(motor1_pin_1, HIGH);
	digitalWrite(motor1_pin_2, HIGH);
	digitalWrite(motor2_pin_1, HIGH); 
	digitalWrite(motor2_pin_2, HIGH);
}

//_____________________________________________________________________________



void loop () 
{
	//Serial.print("Welcome to motor control");
	Serial.print('\n');
	
	
	//  while( state == 0){
	//    ReadInputs();
	//  }
	while( state == 0 ){
                enc1_Count =0;
                enc2_Count =0;
                Reset();
                
		if(Serial.peek() == 'N'){
			state = 1;
			while(Serial.available()){
				Serial.read();
			}

			Serial.println("hello");
			Serial.println( _path[0][0]);
			Serial.println( _path[0][1]);
			Serial.println( _path[1][0]);
			Serial.println( _path[1][1]);
			Serial.println(_path.size());
		}  



		if((Serial.available() == 5) && (input2data == 1 )){
			char bytes1[6];
			bytes1[0] = Serial.read();  
			bytes1[1] = Serial.read();
			bytes1[2] = Serial.read();
			bytes1[3] = Serial.read();
			bytes1[4] = Serial.read();
			bytes1[5] = '\0';
			pos_1 = atoi(bytes1);
			path.clear();
			path.push_back(pos_1);
			abspos_1 = abs(pos_1);

			Serial.print('\n');
			Serial.print(enc1_Count);
			Serial.print('\t');
			Serial.print(enc2_Count);
			Serial.print('\n');

			Serial.print("pos_1 = ");
			Serial.print(pos_1, DEC);
			Serial.println();
			input1done =1;
			input2data=0;
			//    enc1_Count = 0;
			//    enc2_Count = 0;
		}
		if((Serial.available() == 5) && (input1done == 1)){
			char bytes2[6];
			bytes2[0] = Serial.read();  
			bytes2[1] = Serial.read();
			bytes2[2] = Serial.read();
			bytes2[3] = Serial.read();
			bytes2[4] = Serial.read();
			bytes2[5] = '\0';
			pos_2 = atoi(bytes2);
			path.push_back(pos_2);
			_path.push_back(path);
			abspos_2 = abs(pos_2);
			Serial.print("pos_2 = ");
			Serial.print(pos_2, DEC);
			//  Serial.print(path[0]);
			//  Serial.print(path[1]);
			Serial.println();
			//state = 1;
			input1done = 0;
			input2data = 1;
			Serial.print(state,DEC );
			Serial.println();
		}   
	}
	if(state == 1){    
                
		if (poslistFlag == 1) {
                        Stop();
                        delay(300);
  
                        pos_1 = _path[poslist][0];
			pos_2 = _path[poslist][1];
			abspos_1 = abs(pos_1);
			abspos_2 = abs(pos_2);
		        
                        poslist++;
                        
                        

			poslistFlag = 0;  
			enc1_Count = 0;
			enc2_Count = 0;
                        if(abspos_1 != abspos_2){
                          poslist = _path.size()+1;
                        }
		} 
		if( poslist == (_path.size()+1)){
			state = 0;
                        enc1_Count=0;
                        enc2_Count=0;
                        Reset();
                        poslist =0;
                        path.clear();
                        _path.clear();
                        poslistFlag =1;
                        Serial.write('1');
		} else
		{
			
			Serial.print("Begin:  ");
			Serial.print('\n');
			Serial.print("Error : ");
			Serial.print(error);
			Serial.print('\n');
			
			Movement();
			
			Serial.print("Pos1 Pos2 : ");
			Serial.print(pos_1);
			Serial.print("  ");
			Serial.print(pos_2);
			Serial.print('\n');
			
			//Check ();
			
			//  Serial.print(error);
			Serial.print("After the check pwm_1  pwm_2 : ");
			Serial.print(pwm_1);
			Serial.print("  ");
			Serial.print(pwm_2);
			Serial.print('\n');
			
			//Position(pos_1, pos_2);
			
			Serial.print("Encoder count _1  _2 : ");
			Serial.print(enc1_Count);
			Serial.print("  ");
			Serial.print(enc2_Count);
			Serial.print('\n');
			Serial.print(poslist);
			Serial.print('\n');
		}
	}
} //Close the loop

//Interrupt Function
void enc1()
{
	enc1_Count++;
        Check();	
        Position();
        
}

void enc2()
{
	enc2_Count++;
	Check();
        Position();
}

void Check()
{ 
//  int error;
// int lastError = 0;
 //int sumError = 0;
// float adjustment = 0;
	if( enc1_Count == enc2_Count){
            if (pos_1 != pos_2)
              {
                pwm_1 =100;
                pwm_2 =100;
              }
             else 
              {
		pwm_1 = 255;
		pwm_2 = 255; //Adjust to taste
              }
		error = 0;
	}
	else if(enc1_Count > enc2_Count)
	{ 
		error = (enc1_Count - enc2_Count);
		//   adjustment = (KP*error + KD*(error - lastError)+ KI*sumError);
		//pwm_2 += error + 12;
		pwm_1 -= error + 60;
		//  lastError = error;
		//  sumError += error;
	}
	else if(enc1_Count < enc2_Count)
	{
		error = (enc2_Count  - enc1_Count); 
		// adjustment = KP*error + KD*(error - lastError)+ KI*sumError;
		pwm_2 -= error + 60;
		//pwm_1 += error + 11; 
		  // lastError = error;
		  // sumError += error;
	}
	else 
	{
		pwm_1+=0;
		pwm_2+=0;
	}
	if (pwm_1 >= 255)
		pwm_1 = 255;
	if (pwm_2 >= 255)
		pwm_2 = 255;
    if (pwm_1 <= 30)
		pwm_1 = 30;
    if (pwm_2 <= 0)
		pwm_2 = 30;
}

void Movement()
{  
  
	if (abspos_1 == abspos_2)
    {
		if ( pos_1 > 0 && pos_2 > 0 ) 
		{
			Accelerate (pwm_1, pwm_2); 
			Serial.print("Accelerate");
		}
		else if ( pos_1 >= 0 && pos_2 < 0 )
		{
			RightTurn(pwm_1, pwm_2);
			Serial.print("Right Turn");
		}
		else if ( pos_1 < 0 && pos_2 >= 0 )
		{
			LeftTurn(pwm_1, pwm_2); 
			Serial.print("Left Turn");
		}
		else if ( pos_1 < 0 && pos_2 < 0 )
		{
			Reverse(pwm_1,pwm_2);
			Serial.print("Reverse");
		}
		else 
		{
			Stop();
			//state = 0;
			Serial.print("Stop");
		}
	} 
	else
	{
		Stop();
		//state=0;
	}
}

void Position()
{  
	if ((enc1_Count >= abspos_1) && (enc2_Count >= abspos_2))
	{
		tempdebug1 = enc1_Count;
		tempdebug2 = enc2_Count; 
		Reset();
		//    Stop(); 
		//    enc1_Count =0;
		//    enc2_Count =0;
		//    pwm_1 = 255;
		//    pwm_2 = 255;
		//    pos_1 = 0;
		//    pos_2 = 0;
		//state = 0;
		poslistFlag = 1;
	}
}

void Reset(){
	Stop(); 
	//   enc1_Count =0;
	//   enc2_Count =0;
             if (pos_1 != pos_2)
              {
                pwm_1 =100;
                pwm_2 =100;
              }
             else 
              {
		pwm_1 = 255;
		pwm_2 = 255; 
              }
   pos_1 = 0;
    pos_2 = 0;
    //state = 0;
}
