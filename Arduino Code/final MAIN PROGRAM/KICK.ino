void NEWkick(){
  
    //Reset();
   int flag = 0; 
   CheckforE();
   pos_1 = 48; 
   pos_2 = -48;
   enc1_Count =0;
   enc2_Count =0;
   pwm_1=255;
   pwm_2=255;
   abspos_1 =abs(pos_1)-(abs(pos_1)*0.08+1);
   abspos_2 = abs(pos_2)-(abs(pos_2)*0.08+1);
   slowdown1 =abspos_1*0.9;                      
   slowdown2 =abspos_2*0.9;
   while( enc1_Count < abspos_1 && enc2_Count < abspos_2 && flag ==0) 
   { 
     if( CheckforE() == TRUE){ 
       flag = 1;
       break;
       }
       LeftTurn(pwm_1, pwm_2); 
       //Position();
       logln(' ');
       log1("Encoder1:  ");
       log1(enc1_Count);
       log1("Encoder2:  ");
       log1(enc2_Count);
       }
   delay(300);
   
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
         CheckforE();
         logln("Actuator is fully in "); 
         pos_1 = -48; 
         pos_2 = 48;
         enc1_Count =0;
         enc2_Count =0;
         pwm_1=255;
         pwm_2=255;
         abspos_1 = abs(pos_1)-(abs(pos_1)*0.08+3);
         abspos_2 = abs(pos_2)-(abs(pos_2)*0.08+3);
         slowdown1 =abspos_1*0.9;
         slowdown2 =abspos_2*0.9;
         logln("Right Turn");
         
         delay(300);
         
         while( enc1_Count < abspos_1 && enc2_Count < abspos_2 && flag ==0) 
         { 
           if( CheckforE() == TRUE){ 
             flag = 1;
             break;
           }
           RightTurn(pwm_1, pwm_2); 
           //Position();
           logln(' ');
           log1("Encoder1:  ");
           log1(enc1_Count);
           log1("Encoder2:  ");
           log1(enc2_Count);
          }
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
            } 
      }
      logln("Done with the ball kicking") ;   
      Serial.print(4);   
      enc1_Count =0;
      enc2_Count =0;
      digitalWrite(actuator_enable, LOW); 
      //Reset();
}
