/*
*/

//Function Declaration
void Sensor_Read();
void Sensor_Motion();
void Sensor_Ball();

// Function Definition
int Sensor_Read()
{// read the value from the sensor:
  sensorValue = analogRead(sensorPin);  
  
  // Do averaging:
  bufferA = bufferB;
  bufferB = bufferC;
  bufferC = sensorValue;
  sensorValAvg = (bufferA+bufferB+bufferC)/3;
  
  //Write to serial output
  Serial.print("sensor value = ");
  Serial.println(sensorValAvg); 
  
  // Convert ADC data into distance value
   distance = (2914/(sensorValAvg+5))-1;
   Serial.println(distance); 
   return(distance);
  /* 
  // turn the ledPin on
  digitalWrite(ledPin, HIGH);  
  
  // stop the program for <sensorValue> milliseconds:
  delay(sensorValue);          
  
  // turn the ledPin off:        
  digitalWrite(ledPin, LOW);   
  
  // stop the program for for <sensorValue> milliseconds:
  delay(sensorValue); 
 */
}

void Sensor_Motion()
{ if (distance <= 5)// give it a value 5 cm
  {Stop();
   state = 0;
  }
}

/*
void Sensor_Ball()
{// If the ball is in front then move recored distance .. move anglr and record distance again .. if its the smallest value .. then go forward in tht case 
  int dist_0,dist_1,dist_2;
  if (ball == TRUE) // if its a ball in front 
   { dist_0 = Sensor_Read();
     // turn right by 45deg.
      pos_1 = 10; 
      pos_2 =  -10; 
      RightTurn(pwm_1,pwm_2);
     dist_1 = Sensor_Read();
     //turn left by 90deg.
      pos_1 = -20;
      pos_2 = 20; 
      LeftTurn(pwm_1,pwm_2);
     dist_2 = Sensor_Read();
   
   //Compare all the distances and go forward in the direction of the shortest distance   
     if ((dist_0 > dist_1) && (dist_1 > dist_2))
       { small = dist_2;
       }
     else if ((dist_0 > dist_1) && (dist_2 > dist_1))
       { small = dist_1; 
       }
     else 
      small = dist_0;
     
     switch (small)
    {case dist_0 ://Turn right by 45 deg.
                  pos_1 = 10; 
                  pos_2 = -10; 
                  RightTurn(pwm_1, pwm_2); 
                  Accelerate();
                  break;
     case dist_1 ://Turn right by 90 deg.
                  pos_1 = 20; 
                  pos_2 = -20; 
                  RightTurn(pwm_1, pwm_2); 
                  Accelerate();
                  break;
     case dist_2 : 
                  break;
     default : break;
    } 
    
   }
}
*/

