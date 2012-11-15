void CatchtheBall()
{ 
  int caught =0;
  Reset();
  while(caught == 0){
        if( CheckforE() == TRUE){ 
          Reset();
          break;
          }
         Sensor();
         Serial.println(SenseDistance);
         
         if( GrabDir == RIGHT){
           RightTurn(70,70);
         }
         if(GrabDir == LEFT){
           LeftTurn(70,70);
         }
         
         Sensor();
         if(SenseDistance <= 20){
           if(GrabDir == LEFT || GrabDir == RIGHT){
             GrabDir = 100;
             delay(300);
           }
           //abspos_1 =20;
           //abspos_2 =20;
           //enc1_Count =0;
           //enc2_Count =0;
           //Stop();
           Accelerate(pwm_1,pwm_2);
         }
        
        if(SenseDistance <= 4){
                Stop();
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
  abspos_1 = 23;
  abspos_2 = 23;
   while((enc1_Count < abspos_1) && (enc2_Count < abspos_2))
     {RightTurn(100,100);
     }
   Stop();
   
   Sensor(); 
   distance_2 = SenseDistance; 
   //Turn left 90 deg
   enc1_Count = 0; 
   enc2_Count = 0;
   abspos_1 = 48;
   abspos_2 = 48;
    while((enc1_Count < abspos_1) && (enc2_Count < abspos_2))
     {LeftTurn(100,100);
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

