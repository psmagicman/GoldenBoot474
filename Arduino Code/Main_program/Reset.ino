
void Reset(){
	Stop(); 
	//   enc1_Count =0;
	//   enc2_Count =0;
             if (pos_1 != pos_2)
              {
                pwm_1 =100;
                pwm_2 =100;
              }
             else 
              {
		pwm_1 = 255;
		pwm_2 = 255; 
              }
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
        ActuatorControl(RETRACT);
        Serial.println("Robot Reset()");
        while (Serial.available())
        {Serial.read();
        }
}
