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

