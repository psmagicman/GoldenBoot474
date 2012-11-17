
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
            motor =0;
            if (pos_1 != pos_2)
              {
                
                if( (enc1_Count >= slowdown1) && (enc2_Count >= slowdown2)){  //Slow down before stopping
                  pwm_1 = 70;
                  pwm_2 = 70;
                  
                }
                else{
                  pwm_1 =100;
                  pwm_2 =100;
                }
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
		error1 = 0;
                error2 = 0;
	}
	else if(enc1_Count > enc2_Count)
	{ 
                motor =1;
                
		error1 = (enc1_Count - enc2_Count);
	        //adjustment = (KP*error + KD*(error - lastError)+ KI*sumError);
		//pwm_2 += error*30+KI*sumError1;
		pwm_1 -= (error1*KP+KI*sumError1);  
		//  lastError = error;
		  sumError1 += error1;
	}
	else if(enc1_Count < enc2_Count)
	{      
                motor = 2;
                
		error2 = (enc2_Count  - enc1_Count); 
		// adjustment = KP*error + KD*(error - lastError)+ KI*sumError;
		pwm_2 -= (error2*KP+KI*sumError2);
		//pwm_1 += error*60+KI*sumError2; 
		  // lastError = error;
		   sumError2 += error2;
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
    if (pwm_1 <= 0)
		pwm_1 = 0;
    if (pwm_2 <= 0)
		pwm_2 = 0;
}

void Movement()
{  
  
	if (abspos_1 == abspos_2)
    {
		if ( pos_1 > 0 && pos_2 > 0 ) 
		{
			Accelerate (pwm_1, pwm_2); 
			Serial.println("Accelerate");
		}
		else if ( pos_1 >= 0 && pos_2 < 0 )
		{
			LeftTurn(pwm_1, pwm_2);
			Serial.println("Left Turn");
		}
		else if ( pos_1 < 0 && pos_2 >= 0 )
		{
			RightTurn(pwm_1, pwm_2); 
			Serial.println("Right Turn");
		}
		else if ( pos_1 < 0 && pos_2 < 0 )
		{
			Reverse(pwm_1,pwm_2);
			Serial.println("Reverse");
		}
		else 
		{
			Stop();
			//state = 0;
			Serial.println("Stop");
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
         /*if((enc1_Count >= abspos_1*0.8) && (enc2_Count >= abspos_2*0.8)){
               pwm_1 =70;
               pwm_2 =70;
         }*/
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



void MotorControl(){
  
                if(motor == 0){
                  Serial.println("Neurtal");}
                  else if(motor == 1){
                    Serial.println("Faster Right");}
                    else if(motor ==2 ){
                      Serial.println("Faster Left");}
                    else{}
                    
                        
                
  		if (poslistFlag == 1) {
                        Stop();
                        delay(300);
  
                        pos_1 = _path[poslist][0];
			pos_2 = _path[poslist][1];
                        //abspos_1 = abs(pos_1);
                        //abspos_2 = abs(pos_2);
                        if(( (abs(pos_1) <= 2) && (pos_1 != 0)) || ((abs(pos_2) <= 2) && (pos_2 !=0))){
                          abspos_1 =1;
                          abspos_2 =1;
                        
                        }
                        else{
                          //abspos_1 = abs(pos_1)-(abs(pos_1)*0.07+4);
                          //abspos_2 = abs(pos_2)-(abs(pos_2)*0.07+4);
                          abspos_1 =abs(pos_1)-1;
                          abspos_2 =abs(pos_2)-1;
		        }
                        slowdown1 =abspos_1*0.7;
                        slowdown2 =abspos_2*0.7;

                        poslist++;
                        Serial.println("ENCODER END");
                        Serial.println(enc1_Count);
                        Serial.println(enc2_Count);
                        
                        
                        pwm_1 =255;
                        pwm_2 =255;
			poslistFlag = 0;  
			enc1_Count = 0;
			enc2_Count = 0;
                        sumError1= 0;
                        sumError2 =0;
                        if(abspos_1 != abspos_2){
                          poslist = _path.size()+1;
                        }
		} 
		if( poslist == (_path.size()+1)){
			//state = STANDBY;
                       // enc1_Count=0;
                       // enc2_Count=0;
                        Reset();
                       // poslist =0;
                       // path.clear();
                       // _path.clear();
                        //poslistFlag =1;
                        Serial.write('1');
		} else
		{
			
			Serial.print("Begin:  ");
			Serial.print('\n');
			Serial.print("Error : ");
			Serial.print(error1);
                        Serial.print('\t');
                        Serial.print(error2);
			Serial.print('\n');
			
			Movement();
			
	/*		Serial.print("Pos1 Pos2 : ");
			Serial.print(pos_1);
			Serial.print("  ");
			Serial.print(pos_2);
			Serial.print('\n');*/
			
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
