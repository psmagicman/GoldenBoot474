#include "Serial.h"
#include <stdio.h>
#include <assert.h>
#include <Windows.h>
#include <iostream>

using namespace std;

int main(int argc, char * argv[]) {
	CSerial serial;
	if(serial.Open(1, 57600)) {
		/*const char* buffer = new char[8];
		buffer = "Hello";
		int nBytesSent = serial.SendData(buffer, strlen(buffer));
		assert(nBytesSent == strlen(buffer));
		printf("buffer = %s\n", buffer);
		Sleep(10000);
		char* lpBuffer = new char[8];
		int nBytesRead = serial.ReadData(lpBuffer, 8);
		printf("nBytesRead = %d\n", nBytesRead);
		printf("lpBuffer = %s\n", lpBuffer);
		delete []lpBuffer;*/
		//int testLeft = 100;
		//int testRight = 100;
		char leftInput[40];
		char rightInput[40];
		//cin >> leftInput;
		//cin >> rightInput;
		//int testLeft = atoi(leftInput);
		//int testRight = atoi(rightInput);
		int testRight = -104;
		int testLeft = 104;
		if (argc > 1) {
			testLeft = atoi(argv[1]);
			testRight = atoi(argv[2]);
		}
		/*
		else {
			cin >> leftInput;
			cin >> rightInput;
			testLeft = atoi(leftInput);
			testRight = atoi(rightInput);
		}
		*/
		char* leftDigit = new char[6];
		leftDigit[5] = '\0';
		//leftDigit = "00000";
		char* rightDigit = new char[6];
		rightDigit[5] = '\0';
				//rightDigit[5] = {'\0'};
		//rightDigit = "00000";
		//leftDigit = "00100";
		//rightDigit = "00100";
		/*
		for(int k = 0; k < 3; k++) {
			leftDigit[k] = '0';
			rightDigit[k] = '0';
		}
		for(int k = 3; k < 5; k++) {
			leftDigit[k] = '1';
			rightDigit[k] = '1';
		}
		*/
		if(testLeft > 0) {
			for(int j = 0; j < 5; j++) {
				leftDigit[4-j] = (char)(((int)'0')+testLeft%10);
				testLeft /= 10;
			}
		}
		else {
			testLeft = abs(testLeft);
			leftDigit[0] = '-';
			for(int j = 1; j < 4; j++) {
				leftDigit[4-j] = (char)(((int)'0')+testLeft%10);
				testLeft /= 10;
			}
		}
		if(testRight > 0) {
			for(int j = 0; j < 5; j++) {
				rightDigit[4-j] = (char)(((int)'0')+testRight%10);
				testRight /= 10;
			}
		}
		else {
			testRight = abs(testRight);
			rightDigit[0] = '-';
			for(int j = 0; j < 4; j++) {
				rightDigit[4-j] = (char)(((int)'0')+testRight%10);
				testRight /= 10;
			}
		}
		//int leftByte = serial.SendData(leftDigit, strlen(leftDigit));
		//int rightByte = serial.SendData(rightDigit, strlen(rightDigit));
	}
	else
		printf("Error\n");

	return 0;
}