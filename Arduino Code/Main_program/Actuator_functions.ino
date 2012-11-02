
/*
This sections contains the functions used to activate the Linear actuator ---- TESTED IT WORKS !!!!!
*/
// Variable definition 
#define LOW 0
#define HIGH 1
#define PWM_MAX 255

int actuator_length, actuator_length_1;

//Pins needed for the Linear Actuator
int actuator_pin1 = 12;
int actuator_pin2 = 13;
int actuator_input = A7; 

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

void Actuator_Read() 
{
  actuator_length = analogRead(actuator_input);
  Serial.print('\n');
  Serial.print("Actuator value analog : ");
  Serial.print(actuator_length);
}

//Setup -- DELETE after Testing
void setup()
{ Serial.begin(57600);
pinMode(actuator_pin1, OUTPUT);
pinMode(actuator_pin2, OUTPUT);
pinMode(actuator_input, INPUT);
digitalWrite(actuator_input, HIGH);

delay(500);
}

//LOOP --DELETE after testing
void loop ()
{ char c;
  if(Serial.available() > 0)
  {c=Serial.read();
   if (c == 'a')
    {Actuator_Activate();
    // delay(2500);
    Serial.print('\n');
   // Serial.print(c);
    Serial.print("Eject");
    Serial.print("After activation ");
    Actuator_Read();
    }
  
   
  if (c == 'e')
  {Actuator_Deactivate();
  Serial.print('\n');
    //Serial.print(c);
    Serial.print("Retract");
 //     delay(2500);
    Serial.print('\n');
  Serial.print("After de-activation ");
 Actuator_Read();
   }
 
  }
  
//  Serial.write(1);
 // Serial.write("DONE");
  
}
