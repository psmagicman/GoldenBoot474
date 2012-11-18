#include <iostream>
#include <string>
#include <Windows.h>

#include "Serial.h"

using namespace std;

/*
 * this is a simple read/write program using serial port communications
 * the program will send some input from the user through the serial port
 * the arduino will read the input and send the same input back to the user
 * reading will occur after 1 second
 */

/*
 * I made this work by sending some byte to the arduino the moment the program starts
 * The arduino then receives the byte and sends that same byte back 500ms later
 * While the arduino is receiving and sending data, the program waits for 1s before reading 
 * the byte that is saved into the buffer
 * The trick here is to make sure that the delay between receiving and sending for the arduino
 * is less than the delay between sending and receiving for the program. Having the arduino delay be
 * half the delay of the program worked nicely
 */
void main(int argc, char * argv[]) {
	CSerial serial;
	
	while(serial.Open(3, 57600)) {
		char * sendBuffer = {"0"};
		int nBytesSent = serial.SendData(sendBuffer, strlen(sendBuffer));
		cout << "Sending out the string " << sendBuffer << " now!" << endl;
		// this will output the string that is sent to the arduino
		Sleep(1000);
		char * readBuffer = new char[1];
		int nBytesRead = serial.ReadData(readBuffer, 1);
		int test = atoi(readBuffer);
		cout << "Arduino sends back " << test << endl;
		// expect to receive the same string back
		//Sleep(1000);
	}
}