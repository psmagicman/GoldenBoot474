
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
	digitalWrite(motor1_pin_1, LOW);
	digitalWrite(motor1_pin_2, HIGH);
	digitalWrite(motor2_pin_1, HIGH);
	digitalWrite(motor2_pin_2, LOW); 
}

void RightTurn(int pwm_1, int pwm_2) 
{  
	analogWrite(enablepin_1, pwm_1);
	analogWrite(enablepin_2, pwm_2);
	digitalWrite(motor1_pin_1, HIGH);
	digitalWrite(motor1_pin_2, LOW);
	digitalWrite(motor2_pin_1, LOW);
	digitalWrite(motor2_pin_2, HIGH);  
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
	digitalWrite(motor1_pin_1, LOW);
	digitalWrite(motor1_pin_2, LOW);
	digitalWrite(motor2_pin_1, LOW); 
	digitalWrite(motor2_pin_2, LOW);
}


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
              /*  if( (enc1_Count*100 >= abspos_1*80) && (enc2_Count*100 >= abspos_2*80)){  //Slow down before stopping
                  pwm_1 = 60;
                  pwm_2 = 60;
                  
                }
                else{*/
                  pwm_1 =100;
                  pwm_2 =100;
             //   }
              }
             else 
              {
              /*  if( (enc1_Count*100 >= abspos_1*80) && (enc2_Count*100 >= abspos_2*80)){  //Slow down before Stopping
                  pwm_1 = 80;
                  pwm_2 = 80;
                }
                else{*/
                  pwm_1 = 255;
		  pwm_2 = 255; //Adjust to taste
             //   }
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
    if (pwm_1 <= 50)
		pwm_1 = 50;
    if (pwm_2 <= 50)
		pwm_2 = 50;
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
			LeftTurn(pwm_1, pwm_2);
			Serial.print("Left Turn");
		}
		else if ( pos_1 < 0 && pos_2 >= 0 )
		{
			RightTurn(pwm_1, pwm_2); 
			Serial.print("Right Turn");
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
         if((enc1_Count >= abspos_1) && (enc2_Count >= abspos_2))
         {
                Stop();
		    //enc1_Count =0;
		    //enc2_Count =0;
		//    pwm_1 = 255;
		//    pwm_2 = 255;
		//    pos_1 = 0;
		//    pos_2 = 0;
		//state = 0;
		poslistFlag = 1;
                
	}
/*	if ((abspos_1 != 0) && (abspos_2 !=0) && (enc1_Count*100 >= abspos_1*85) && (enc2_Count*100 >= abspos_2*85) )
	{
		
                if( pos_1 > pos_2){
                  RightTurn(pwm_1,pwm_2);
                }
                else if(pos_1 < pos_2){
                  LeftTurn(pwm_1,pwm_2);
                }
                else if(abspos_1 == abspos_2){
                  Reverse(pwm_1, pwm_2); 
                }
                else{ Stop();} 
         
         }*/
        
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
        state = STANDBY;
        input1done = 0;
        input2done = 1;
        poslistFlag = 1;
        
}

void MotorControl(){
  		if (poslistFlag == 1) {
                        Stop();
                        delay(300);
  
                        pos_1 = _path[poslist][0];
			pos_2 = _path[poslist][1];
			abspos_1 = abs(pos_1)-(abs(pos_1)*0.05+4);
			abspos_2 = abs(pos_2)-(abs(pos_2)*0.05+4);
		        
                        poslist++;
                        Serial.println("ENCODER");
                        Serial.println(enc1_Count);
                        Serial.println(enc2_Count);
                        
                        
                        pwm_1 =255;
                        pwm_2 =255;
			poslistFlag = 0;  
			enc1_Count = 0;
			enc2_Count = 0;
                        if(abspos_1 != abspos_2){
                          poslist = _path.size()+1;
                        }
		} 
		if( poslist == (_path.size()+1)){
			//state = STANDBY;
                        enc1_Count=0;
                        enc2_Count=0;
                        Reset();
                        poslist =0;
                        path.clear();
                        _path.clear();
                        //poslistFlag =1;
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
