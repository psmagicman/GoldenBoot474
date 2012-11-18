void CatchtheBall()
{ 
  //int flag = 0;
  int caught =0;
  GrabDir =LEFTONE;
 //Reset();
  while(caught == 0){
        if( CheckforE() == TRUE){ 
          Reset();
          break;
          }
        Sensor();
        if( GrabDir == LEFTONE){
        if(SenseDistance > TENNISBALL){
          LeftTurn(70,70);
          if(enc1_Count >= 24 && enc2_Count >= 24){
            Stop();
            enc1_Count = 0;
            enc2_Count = 0;
            GrabDir = RIGHTONE;
            Serial.println("Ball is on my right");
          }
          
        }
        else {
          Stop();
          delay(300);
          GrabDir = LEFTTWO;
          Serial.println("Found Ball");
        }
        }
        
        if(GrabDir == LEFTTWO){
          if(SenseDistance <= TENNISBALL){
            LeftTurn(70,70);
            
          }
          else{
            Serial.println("Edge of Ball,,,,turning back");
            Stop();
            delay(300);
            enc1_Count=0;
            enc2_Count=0;
            abspos_1 =8;
            abspos_2 =8;
            sumError1=0;
            sumError2=0;
            while(enc1_Count <= abspos_1 && enc2_Count <= abspos_2){
               RightTurn(70,70);
             }
             Stop();
             delay(300);
             GrabDir = MIDDLE;
            }
        }
        
        if( GrabDir == RIGHTONE){
        if(SenseDistance > TENNISBALL){
          RightTurn(70,70);
          if(enc1_Count >= 47 && enc2_Count >= 47){
            Stop();
            Reset();
            Serial.write(2);
            Serial.println("Ball is not here");
            break;
          }
        }
        else {
          Stop();
          delay(300);
          GrabDir = RIGHTTWO;
          Serial.println("Found Ball");
        }
        }
        
        if(GrabDir == RIGHTTWO){
          if(SenseDistance <= TENNISBALL){
            RightTurn(70,70);
            
          }
          else{
            Serial.println("Edge of Ball,,,,turning back");
            Stop();
            delay(300);
            enc1_Count=0;
            enc2_Count=0;
            abspos_1 =6;
            abspos_2 =6;
            sumError1=0;
            sumError2=0;
            while(enc1_Count <= abspos_1 && enc2_Count <= abspos_2){
               LeftTurn(70,70);
             }
             Stop();
             delay(300);
             GrabDir = MIDDLE;
             enc1_Count =0;
             enc2_Count =0;
             abspos_1 = 0;
             abspos_2 = 0;
             pwm_1 = 255;
             pwm_2 = 255;
            }
        }
        
        Serial.println(SenseDistance);
        
        Sensor();
        Sensor();
        Sensor();
        if(GrabDir == MIDDLE){
             if(SenseDistance <= TENNISBALL){
               Serial.println("accelerating");
               Accelerate(pwm_1,pwm_2);
             }
             else{
               Serial.println("Scan AGAIN");
               GrabDir = LEFTONE;
               enc1_Count=0;
               enc2_Count=0;
               abspos_1 =1;
               abspos_2 =1;
               sumError1=0;
               sumError2=0;
               while(enc1_Count <= abspos_1 && enc2_Count <= abspos_2){
                     RightTurn(70,70);
               }
             }
        }
        
        if(SenseDistance <= 4){
                Stop();
                Serial.write(1);
                Serial.println("Ball in the caster .");
                Serial.println(SenseDistance);
                caught=1;
         }
  } 
        //enc1_Count =0;
        //enc2_Count =0;
        Reset();
}


void CatchBallDistance()
{ int distance_1,distance_2,distance_3;
  int caught =0;
  while(caught == 0)
  {Sensor();
   distance_1 = SenseDistance;
   //Turn right 45 deg
  enc1_Count = 0; 
  enc2_Count = 0;
  abspos_1 = 1;
  abspos_2 = 1;
   while((enc1_Count <= abspos_1) && (enc2_Count <= abspos_2))
     {RightTurn(70,70);
     }
   Stop();
   
   Sensor(); 
   distance_2 = SenseDistance; 
   //Turn left 90 deg
   enc1_Count = 0; 
   enc2_Count = 0;
   abspos_1 = 2;
   abspos_2 = 2;
    while((enc1_Count < abspos_1) && (enc2_Count < abspos_2))
     {LeftTurn(70,70);
     }
   Stop();
 
   Sensor(); 
   distance_3 = SenseDistance; 
   
    //Compare the distances 
     if ((distance_1 <= distance_2) && (distance_1 <= distance_3))
      {
        Serial.print("Shotest distance is :  ");
        Serial.print(distance_1);
        Serial.println();
        //distance_short = distance_1; 
        //Turn right 45 degree
       enc1_Count = 0; 
       enc2_Count = 0;
       abspos_1 = 23;
       abspos_2 = 23;
        while((enc1_Count < abspos_1) && (enc2_Count < abspos_2))
         {RightTurn(100,100);
         }
        Stop(); 
      }
    else if ((distance_2 <= distance_1) && (distance_2 <= distance_3))
      {
        Serial.print("Shotest distance is :  ");
        Serial.print(distance_2);
        Serial.println();
        //distance_short = distance_2; 
         //Turn right 90 degree
         enc1_Count = 0; 
         enc2_Count = 0;
         abspos_1 = 48;
         abspos_2 = 48;
        while((enc1_Count < abspos_1) && (enc2_Count < abspos_2))
         {RightTurn(100,100);
         }
        Stop(); 
      }
    else
     {Serial.print("Shotest distance is :  ");
      Serial.print(distance_3);
      Serial.println();
     }
     // distance_short = distance_3; 
       //Stay in the same position  
       
      Accelerate(pwm_1, pwm_2);
      Sensor();
      if(SenseDistance <= 4){
                Stop();
                Serial.println("Ball in the caster .");
                Serial.println(SenseDistance);
                caught=1;
         }
  }
}

