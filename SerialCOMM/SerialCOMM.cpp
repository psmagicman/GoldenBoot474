#include <string>
#include <iostream>
#include <Windows.h>

#include "Serial.h"

using namespace std;

void main(int argc, char * argv[])
{

	CSerial serial;
	string firstInput;
	string secondInput;
	while (serial.Open(1, 57600)) {
		cin >> firstInput;
		if (firstInput != "OK") {
			if (firstInput == "STOP") {
				char * STOP = {"E"};
				int stopByte = serial.SendData(STOP, strlen(STOP));
			} else if (firstInput == "EOL") {
				char * EOL = {"N"};
				int eolByte = serial.SendData(EOL, strlen(EOL));
			} else if (firstInput == "KICK") {
				char * KICK = {"K"};
				int kickByte = serial.SendData(KICK, strlen(KICK));
			} else if (firstInput == "END") {
				char * END = {"Z"};
				int endByte = serial.SendData(END, strlen(END));
			} else {
				cin >> secondInput;
				int rightMotor = atoi(firstInput.c_str());
				int leftMotor = atoi(secondInput.c_str());
				char * leftDigit = new char[6];
				leftDigit[5] = '\0';
				char * rightDigit = new char[6];
				rightDigit[5] = '\0';
				
				if (leftMotor > 0) {
					for (int i = 0; i < 5; i++) {
						leftDigit[4-i] = (char)(((int)'0')+leftMotor%10);
						leftMotor /= 10;
					}
				} else {
					leftMotor = abs(leftMotor);
					leftDigit[0] = '-';
					for (int i = 0; i < 4; i++) {
						leftDigit[4-i] = (char)(((int)'0')+leftMotor%10);
						leftMotor /= 10;
					}
				}

				if (rightMotor > 0) {
					for (int i = 0; i < 5; i++) {
						rightDigit[4-i] = (char)(((int)'0')+rightMotor%10);
						rightMotor /= 10;
					}
				} else {
					rightMotor = abs(rightMotor);
					rightDigit[0] = '-';
					for (int i = 0; i < 4; i++) {
						rightDigit[4-i] = (char)(((int)'0')+rightMotor%10);
						rightMotor /= 10;
					}
				}
				int leftByte = serial.SendData(leftDigit, strlen(leftDigit));
				int rightByte = serial.SendData(rightDigit, strlen(rightDigit));
			}
		}
		char * input = new char[1];
		int nBytesRead = serial.ReadData(input,1);
		cout << input;
		Sleep(100);
	}
}