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
		} else if (input == "GRAB") {
			char * GRAB = {"G"};
			if(serial.SendData(GRAB, strlen(GRAB)))
				cout << GRAB << " ";
		} else if (input == "STOP") {
			char * STOP = {"E"};
			if(serial.SendData(STOP, strlen(STOP)))
				cout << STOP << " ";
		} else if (input == "RUN") {
			char * EOL = {"R"};
			if(serial.SendData(EOL, strlen(EOL)))
				cout << EOL << " ";
		} else if (input == "KICK") {
			char * KICK = {"K"};
			if(serial.SendData(KICK, strlen(KICK)))
				cout << KICK << " ";
		} else if (input == "START") {
			char * BEGIN = {"I"};

			int motorDigit;
			char * motorChar = new char[6];
			motorChar[5] = '\0';

			// Right Motor
			cin >> input;
			motorDigit = atoi(input.c_str());
			if (motorDigit != 0) {
				if(serial.SendData(BEGIN, strlen(BEGIN)))
					cout << BEGIN << " ";

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
				if(serial.SendData(motorChar, strlen(motorChar)))
					cout << motorChar << " ";
			}

			// Right Motor
			cin >> input;
			motorDigit = atoi(input.c_str());
			if (motorDigit != 0) {
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
				if(serial.SendData(motorChar, strlen(motorChar)))
					cout << motorChar << " ";
			}
		}
		char * readBuffer = new char[1];
		readBuffer[0] = '0';
		if (serial.ReadData(readBuffer,1)) {
			cout << ":" << atoi(&readBuffer[0]) << endl;
		}
	}
}