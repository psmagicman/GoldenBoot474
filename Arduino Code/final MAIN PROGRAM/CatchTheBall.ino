void CatchtheBall()
{ 
  //int flag = 0;
  int caught =0;
  GrabDir =LEFTONE;
 //Reset();
 enc1_Count = 0;
 enc2_Count = 0;
 sumError1 =0;
 sumError2 =0;
 
  while(caught == 0){
        if( CheckforE() == TRUE){ 
          Reset();
          break;
          }
        Sensor();
        Sensor();
        Sensor();
        if( GrabDir == LEFTONE){
        if(SenseDistance > TENNISBALL){
          LeftTurn(70,70);
          if(enc1_Count >= 24 && enc2_Count >= 24){
            Stop();
            enc1_Count = 0;
            enc2_Count = 0;
            sumError1 = 0;
            sumError2 = 0;
            GrabDir = RIGHTONE;
            logln("Ball is on my right");
          }
          
        }
        else {
          Stop();
          delay(300);
          GrabDir = LEFTTWO;
          logln("Found Ball");
        }
        }
        
        if(GrabDir == LEFTTWO){
          if(SenseDistance <= TENNISBALL){
            LeftTurn(70,70);
            
          }
          else{
            logln("Edge of Ball,,,,turning back");
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
            Serial.print(3);
            logln("Ball is not here");
            break;
          }
        }
        else {
          Stop();
          delay(300);
          GrabDir = RIGHTTWO;
          logln("Found Ball");
        }
        }
        
        if(GrabDir == RIGHTTWO){
          if(SenseDistance <= TENNISBALL){
            RightTurn(70,70);
            
          }
          else{
            logln("Edge of Ball,,,,turning back");
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
        
        logln(SenseDistance);
        
        Sensor();
        Sensor();
        Sensor();
        if(GrabDir == MIDDLE){
             if(SenseDistance <= TENNISBALL){
               logln("accelerating");
               Accelerate(pwm_1,pwm_2);
             }
             else{
               logln("Scan AGAIN");
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
                enc1_Count = 0;
                enc2_Count = 0;
                sumError1 = 0;
                sumError2 = 0;
                while( enc1_Count <= 10 && enc2_Count <= 10){
                  Accelerate(pwm_1, pwm_2);
                }
                Stop();
                Serial.print(2);
                logln("Ball in the caster .");
                logln(SenseDistance);
                caught=1;
                StillGotTheBall = 1;
         }
  } 
        //enc1_Count =0;
        //enc2_Count =0;
        Reset();
}


