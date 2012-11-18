/*
This function is used to kick the ball 
*/

//Function Declaration
void KicktheBall();

//Function Definition
void KicktheBall()
{ 
   Reset();
   int flag1 = 1; 
   int i;  
   int flag = 1; 
      
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
  /* if (flag ==0 )
     {
      logln("Actuator is fully out");                
      pos_1 = 80; 
      pos_2 = 80; 
      pwm_1=255;
      pwm_2=255; 
      abspos_1 = 80;
      abspos_2 = 80;
      enc1_Count =0;
      enc2_Count =0;
      logln("Accelerate");
      while (enc1_Count < pos_1 && enc2_Count < pos_2)   
           { 
             if( CheckforE() == TRUE){ 
                flag = 1;
                break;
             }
             Reverse(pwm_1,pwm_2); 
             //Position();
             logln();
             log1("Encoder1:  ");
             log1(enc1_Count);
             log1("Encoder2:  ");
             log1(enc2_Count);
           } */ 
      //Stop();
     //}           
     //while(flag1 == 1) 
         // {  
     //delay(300);  
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
           //logln();
           log1("Encoder1:  ");
           log1(enc1_Count);
           log1("Encoder2:  ");
           log1(enc2_Count);
          }
          delay(300);
          if (flag ==0)
            {
             logln("Ball just out of the caster ");
             //  logln(SenseDistance);            
             Actuator_Read();
             while (actuator_length > RETRACT )
             { 
               if( CheckforE() == TRUE){ 
                   flag =1; 
                   break;
               }
               Actuator_Deactivate(); 
               Actuator_Read(); 
               flag = 0; 
             }
             delay(300); 
            } 
      }
      logln("Done with the ball kicking") ;      
      enc1_Count =0;
      enc2_Count =0;
      digitalWrite(actuator_enable, LOW); 
      Reset();
}
