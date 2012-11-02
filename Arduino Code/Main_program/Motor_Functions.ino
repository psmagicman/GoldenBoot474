/*
*/

//Function Declaration 
void Accelerate(int pwm_1, int pwm_2);
void RightTurn(int pwm_1, int pwm_2);
void LeftTurn(int pwm_1, int pwm_2);
void Reverse(int pwm_1, int pwm_2);
void Stop();

// Function Definition
void Accelerate(int pwm_1, int pwm_2) 
{
analogWrite(enablepin_1, pwm_1);
analogWrite(enablepin_2, pwm_2);
digitalWrite(motor1_pin_1, HIGH);
digitalWrite(motor1_pin_2, LOW);
digitalWrite(motor2_pin_1, HIGH);
digitalWrite(motor2_pin_2, LOW);
}

void LeftTurn(int pwm_1, int pwm_2) 
{   
  analogWrite(enablepin_1, pwm_1);
  analogWrite(enablepin_2, pwm_2);
  digitalWrite(motor1_pin_1, HIGH);
  digitalWrite(motor1_pin_2, LOW);
  digitalWrite(motor2_pin_1, LOW);
  digitalWrite(motor2_pin_2, HIGH); 
}

void RightTurn(int pwm_1, int pwm_2) 
{  
  analogWrite(enablepin_1, pwm_1);
  analogWrite(enablepin_2, pwm_2);
  digitalWrite(motor1_pin_1, LOW);
  digitalWrite(motor1_pin_2, HIGH);
  digitalWrite(motor2_pin_1, HIGH);
  digitalWrite(motor2_pin_2, LOW);  
}

void Reverse(int pwm_1, int pwm_2)
{  
  analogWrite(enablepin_1, pwm_1);
  analogWrite(enablepin_2, pwm_2);
  digitalWrite(motor1_pin_1, LOW);
  digitalWrite(motor1_pin_2, HIGH);
  digitalWrite(motor2_pin_1, LOW);
  digitalWrite(motor2_pin_2, HIGH);
  
}

void Stop() 
{
digitalWrite(enablepin_1, HIGH);
digitalWrite(enablepin_2, HIGH);
digitalWrite(motor1_pin_1, LOW);
digitalWrite(motor1_pin_2, LOW);
digitalWrite(motor2_pin_1, LOW); 
digitalWrite(motor2_pin_2, LOW);
}

