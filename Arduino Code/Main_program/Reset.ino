/*
This function resets the memory buffer
*/
//Function Definition 
void Reset();

//Function Declaration
void Reset(){
	Stop(); 
	//   enc1_Count =0;
	//   enc2_Count =0;
	pwm_1 = 255;
	pwm_2 = 255; 
        pos_1 = 0;
        pos_2 = 0;
        state = STANDBY;
        enc1_Count=0;
        enc2_Count=0;
        poslist =0;
        poslistFlag = 1;
        path.clear();
        _path.clear();
        input1done = 0;
        input2done = 1;
        poslistFlag = 1;
        sumError1 =0;
        sumError2 =0;
        
        ActuatorControl(RETRACT);
        Serial.println("Robot Reset()");
        while (Serial.available())
        {Serial.read();
        }
}
