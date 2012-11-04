void serialEvent(){
        ReadInputs();
}

  
void ReadInputs(){
        if(Serial.available() > 0){
           char emergency[1];
           emergency[0] = Serial.peek();
       
           if( emergency[0] == 'E'){
                
                 while(Serial.available()){
                   Serial.read();                   
                 }
                 state = 0;
                 enc1_Count=0;
                 enc2_Count=0;
                 Reset();
                 poslist =0;
                 poslistFlag = 1;
                 path.clear();
                 _path.clear();
                 Serial.write('1');
           }
        }
}
        

        
        
