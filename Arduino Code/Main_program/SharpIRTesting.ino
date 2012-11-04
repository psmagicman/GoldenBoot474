/*
int sensorPin = A6;   
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
int bufferA = 0;
int bufferB = 0;
int bufferC = 0;
int sensorValAvg = 0;
int distance = 0;
*/
/*
void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);  
  Serial.begin(57600);
}
*/

void Sensor() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);  
  
  // Do averaging:
  bufferA = bufferB;
  bufferB = bufferC;
  bufferC = sensorValue;
  sensorValAvg = (bufferA+bufferB+bufferC)/3;
  
  //Write to serial output
//  Serial.print("sensor value = ");
//  Serial.println(sensorValAvg); 
  
  // Convert ADC data into distance value
   SenseDistance = (2914/(sensorValAvg+5))-1;
  //Serial.println(SenseDistance); 
   
  // turn the ledPin on
  digitalWrite(ledPin, HIGH);  
  
  // stop the program for <sensorValue> milliseconds:
//  delay(sensorValue);          
  
  // turn the ledPin off:        
  digitalWrite(ledPin, LOW);   
  
  // stop the program for for <sensorValue> milliseconds:
  //delay(sensorValue);                  
}
