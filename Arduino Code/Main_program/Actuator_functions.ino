/*
This sections contains the functions used to activate the Linear actuator 
*/

//Function Declaration
void Actuator_Activate();
void Actuator_Deactivate();
void Actuator_Read();

//Function Definition
void Actuator_Deactivate() //Pushes the rod out  
{
  digitalWrite(actuator_pin1, HIGH);
  digitalWrite(actuator_pin2, LOW);
}

void Actuator_Activate() //Pulls the rod in 
{
  digitalWrite(actuator_pin1, LOW);
  digitalWrite(actuator_pin2, HIGH);
  
}

void Actuator_Read() //Reads the rod value when it's out or in
{
  actuator_length = analogRead(actuator_input);
}

void ActuatorControl(int length)
{            
           digitalWrite(actuator_enable, HIGH); 
           Actuator_Read();
           if(actuator_length < length)
            {
              /*   while(actuator_length < length)
                 {
                  CheckforE();
                  //Serial.println("Current length : idle state  ");
                  //Serial.print(actuator_length);
                  Actuator_Activate();
                  Actuator_Read();
                 }*/
            }
            else
            {
                while (actuator_length > length )
                {
                 //Serial.println("Current length :  ");
                 // Serial.print(actuator_length); 
                 CheckforE();
                 Actuator_Deactivate(); 
                 Actuator_Read(); 
                }
            }
            digitalWrite(actuator_enable, LOW);
}
            
