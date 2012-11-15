#include <string>
#include <iostream>
#include <Windows.h>

#include "Serial.h"

using namespace std;

void main(int argc, char * argv[])
{
	CSerial serial;
	string input;
	while (serial.Open(1, 57600)) {
		cin >> input;
		if (input == "EXIT") {
			return;
		} else if (input == "STOP") {
			char * STOP = {"E"};
			serial.SendData(STOP, strlen(STOP));
		} else if (input == "EOL") {
			char * EOL = {"R"};
			serial.SendData(EOL, strlen(EOL));
		} else if (input == "KICK") {
			char * KICK = {"K"};
			serial.SendData(KICK, strlen(KICK));
		} else if (input == "START") {
			char * BEGIN = {"I"};
			serial.SendData(BEGIN, strlen(BEGIN));

			int motorDigit;
			char * motorChar = new char[6];
			motorChar[5] = '\0';

			// Right Motor
			cin >> input;
			motorDigit = atoi(input.c_str());
				
			if (motorDigit > 0) {
				for (int i = 0; i < 5; i++) {
					motorChar[4-i] = (char)(((int)'0')+motorDigit%10);
					motorDigit /= 10;
				}
			} else {
				motorDigit = abs(motorDigit);
				motorChar[0] = '-';
				for (int i = 0; i < 4; i++) {
					motorChar[4-i] = (char)(((int)'0')+motorDigit%10);
					motorDigit /= 10;
				}
			}
			serial.SendData(motorChar, strlen(motorChar));

			// Right Motor
			cin >> input;
			motorDigit = atoi(input.c_str());
				
			if (motorDigit > 0) {
				for (int i = 0; i < 5; i++) {
					motorChar[4-i] = (char)(((int)'0')+motorDigit%10);
					motorDigit /= 10;
				}
			} else {
				motorDigit = abs(motorDigit);
				motorChar[0] = '-';
				for (int i = 0; i < 4; i++) {
					motorChar[4-i] = (char)(((int)'0')+motorDigit%10);
					motorDigit /= 10;
				}
			}
			serial.SendData(motorChar, strlen(motorChar));
		}
		Sleep(100);
	}
}