/*This code determines the serial communication
*/
void serialEvent(){
  //logln("serialEvent");
        CheckforE();
}

boolean CheckforE(){
        //if(Serial.available() > 0){
         
           if( Serial.peek() == 'E'){
                 emergency = 1;
                 logln("Emergency");
                 Serial.print(1);
                 while(Serial.available()){
                   Serial.read();                   
                 }
                 Reset();
                 return TRUE;
           }
          else return FALSE;
}

void ReadInput(){

     if(Serial.available() > 0){  
                //enc1_Count =0;
                //enc2_Count =0;
                //Reset();
                CheckforE();
                
                if(Serial.peek() == 'G'){
                        logln("Grabbing the ball");
                        
                        pos_1 = GRABFLAG;
			path.clear();
			path.push_back(pos_1);

                        pos_2 = GRABFLAG;
			path.push_back(pos_2);
			_path.push_back(path);
                        logln(path[0]);
                        logln('\t');
                        logln(path[1]);
                        while(Serial.available()){
		                Serial.read();
			}
                }
                else if(Serial.peek() == 'K'){
                        logln("Kicking the ball ");

                        pos_1 = KICKFLAG;
			path.clear();
			path.push_back(pos_1);

                        pos_2 = KICKFLAG;
			path.push_back(pos_2);
			_path.push_back(path);

                        while(Serial.available()){
		                Serial.read();
			}
                }
                
		else if(Serial.peek() == 'R'){
			state = MOVE;
                        logln("Run");
			while(Serial.available()){
				Serial.read();
			}
		}

                else if (Serial.peek() == 'I'){
                        logln("Reading the input values ");
		        Serial.read();
                        Readposition();
                }
         else 
          Serial.read();
     }
}

boolean CheckGarbbage(char value[6])
{int i; 
   for(i=0; i<5; ++i)
    {
      if (
          (value[i] == '0') ||
          (value[i] == '1') ||
          (value[i] == '2') ||
          (value[i] == '3') ||
          (value[i] == '4') ||
          (value[i] == '5') ||
          (value[i] == '6') ||
          (value[i] == '7') ||
          (value[i] == '8') ||
          (value[i] == '9') ||
          (value[i] == '-') 
          )
           { }
        else
        return FALSE;  
    }
    return TRUE;
}

void Readposition()
{
   char bytes1[6];
   char bytes2[6];
   while(TRUE){
             
               if (CheckforE() == TRUE){
                 break;
               }
               if((Serial.available() == 5)  && (input2done == 1 )){
			
			      bytes1[0] = Serial.read();  
			      bytes1[1] = Serial.read();
			      bytes1[2] = Serial.read();
			      bytes1[3] = Serial.read();
			      bytes1[4] = Serial.read();
                              bytes1[5] = '\0';
                              if (CheckGarbbage(bytes1) == FALSE){
                                  logln("RESET check barbbage");
                                  Reset();
                                  break;
                               }
                              else {
			            input1done =1;
		                    input2done=0;
		                    }
                }
		if((Serial.available() == 5) && (input1done == 1)){
			
			bytes2[0] = Serial.read();  
			bytes2[1] = Serial.read();
			bytes2[2] = Serial.read();
			bytes2[3] = Serial.read();
			bytes2[4] = Serial.read();
			bytes2[5] = '\0';
                         if( CheckGarbbage(bytes2) == TRUE){
                        /////////////////////////////////////////// 
			pos_1 = atoi(bytes1);
			path.clear();
			path.push_back(pos_1);

                        pos_2 = atoi(bytes2);
			path.push_back(pos_2);
			_path.push_back(path);
                        ///////////////////////////////////////////////
                        
			log1('\n');
			log1(enc1_Count);
			log1('\t');
			log1(enc2_Count);
		        log1('\n');
      			log1("pos_1 = ");
		        log1(pos_1);
			logln(' ');

			log1("pos_2 = ");
			log1(pos_2);
			//  log1(path[0]);
			//  log1(path[1]);
			logln(' ');
			//state = 1;
			input1done = 0;
			input2done = 1;
			log1(state);
			logln(' ');
                        break;
                        }
                        else{
                           Reset();
                           //Serial.println("garbage garbage");
                           break;
		        }    
         }
  }
}

