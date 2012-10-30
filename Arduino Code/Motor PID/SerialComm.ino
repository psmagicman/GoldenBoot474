void serialEvent(){
        ReadInputs();
}

  
void ReadInputs(){
        if(Serial.available() > 0){
           char emergency[1];
           emergency[0] = Serial.peek();
       
           if( emergency[0] == 'E'){
                 Reset();
                 Serial.write('1');
                 while(Serial.available()){
                   Serial.read();
                 }
           }
        

        
        
           else{
         
                 if((Serial.available() == 5) && (input2data == 1 )){
                        char bytes1[5];
                        bytes1[0] = Serial.read();  
                        bytes1[1] = Serial.read();
                        bytes1[2] = Serial.read();
                        bytes1[3] = Serial.read();
                        bytes1[4] = Serial.read();
                        // bytes1[5] = '\0';
                        pos_1 = atoi(bytes1);
                        abspos_1 = abs(pos_1);
                        Serial.print("pos_1 = ");
                        Serial.print(pos_1, DEC);
                        Serial.println();
                        input1done =1;
                        input2data=0;
                        enc1_Count = 0;
                        enc2_Count = 0;
                        }
           }
         }
        if((Serial.available() > 0) && (input1done == 1)){
                       char emergency[0];
                       emergency[0] = Serial.peek();
                       if( atoi(emergency) == 9){
                          Reset();
                          while(Serial.available()){
                   Serial.read();}
                       }
        else{
                            if(Serial.available() == 5){
            char bytes2[5];
            bytes2[0] = Serial.read();  
            bytes2[1] = Serial.read();
            bytes2[2] = Serial.read();
            bytes2[3] = Serial.read();
            bytes2[4] = Serial.read();
            // bytes2[5] = '\0';
            pos_2 = atoi(bytes2);
            abspos_2 = abs(pos_2);
            Serial.print("pos_2 = ");
            Serial.print(pos_2, DEC);
            Serial.println();
            state = 1;
            input1done = 0;
            input2data = 1;
            Serial.print(state,DEC );
            Serial.println();
          }
        }
}
}




