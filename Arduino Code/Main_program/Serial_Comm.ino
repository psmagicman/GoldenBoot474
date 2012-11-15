/*This code determines the serial communication
*/
void serialEvent(){
        CheckforE();
}

boolean CheckforE(){
        //if(Serial.available() > 0){
         
           if( Serial.peek() == 'E'){
                 emergency = 1;
                 Serial.println("Emergency");
                 while(Serial.available()){
                   Serial.read();                   
                 }
                 //state = 0;
                 //enc1_Count=0;
                 //enc2_Count=0;
                 Reset();
                 //poslist =0;
                 //poslistFlag = 1;
                 //path.clear();
                 //_path.clear();
                 Serial.write('1');
                 return TRUE;
           }
          else return FALSE;
        //}
        //else return FALSE;
}

void ReadInput(){

     if(Serial.available() > 0){  
                //enc1_Count =0;
                //enc2_Count =0;
                //Reset();
                CheckforE();
                
                if(Serial.peek() == 'G'){
                        Serial.println("Grabbing the ball");
                        GrabDir =LEFTONE;
                        while(Serial.available()){
		                Serial.read();
			}
                        CatchtheBall();
                       // CatchBallDirection();
                }
                
                /*                if(Serial.peek() == 'L'){
                        Serial.println("Grabbing the ball to my left");
                        GrabDir =LEFT;
                        while(Serial.available()){
		                Serial.read();
			}
                        CatchtheBall();
                        //CatchBallDirection();
                }*/
                          
                 else if(Serial.peek() == 'K'){
                        Serial.println("Kicking the ball ");
                        while(Serial.available()){
		                Serial.read();
			}
                        KicktheBall();
                }
                
		else if(Serial.peek() == 'R'){
			state = MOVE;
                        Serial.println("Run");
			while(Serial.available()){
				Serial.read();
			}
		}

                else if (Serial.peek() == 'I'){
                        Serial.println("Reading the input values ");
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
             
               CheckforE();
               if((Serial.available() == 5)  && (input2done == 1 )){
			
			      bytes1[0] = Serial.read();  
			      bytes1[1] = Serial.read();
			      bytes1[2] = Serial.read();
			      bytes1[3] = Serial.read();
			      bytes1[4] = Serial.read();
                              bytes1[5] = '\0';
                              if (CheckGarbbage(bytes1) == FALSE){
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
                        
			Serial.print('\n');
			Serial.print(enc1_Count);
			Serial.print('\t');
			Serial.print(enc2_Count);
		        Serial.print('\n');
      			Serial.print("pos_1 = ");
		        Serial.print(pos_1, DEC);
			Serial.println();

			Serial.print("pos_2 = ");
			Serial.print(pos_2, DEC);
			//  Serial.print(path[0]);
			//  Serial.print(path[1]);
			Serial.println();
			//state = 1;
			input1done = 0;
			input2done = 1;
			Serial.print(state,DEC );
			Serial.println();
                        break;
                        }
                        else{
                           Reset();
                           break;
		        }    
         }
  }
}

