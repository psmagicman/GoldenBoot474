/*
This code lists the sensor functions 
*/

//Function definition
void Sensor();

//Function declaration
void Sensor() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);   
  // Do averaging:
  bufferA = bufferB;
  bufferB = bufferC;
  bufferC = sensorValue;
  sensorValAvg = (bufferA+bufferB+bufferC)/3;
  //Convert to distance in cms
  SenseDistance = (2914/(sensorValAvg+5))-1;             
}
