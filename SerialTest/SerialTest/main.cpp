#include "Serial.h"
#include <stdio.h>
#include <assert.h>
#include <Windows.h>
#include <iostream>

using namespace std;

int main() {
	CSerial serial;
	if(serial.Open(3, 57600)) {
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
		cin >> leftInput;
		cin >> rightInput;
		int testLeft = atoi(leftInput);
		int testRight = atoi(rightInput);
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
				leftDigit[j] = (char)('0'+testLeft%10);
				testLeft /= 10;
			}
		}
		else {
			leftDigit[0] = '-';
			for(int j = 1; j < 5; j++) {
				leftDigit[j] = (char)(((int)'0')+testLeft%10);
				testLeft /= 10;
			}
		}
		if(testRight > 0) {
			for(int j = 0; j < 5; j++) {
				rightDigit[j] = (char)(((int)'0')+testRight%10);
				testRight /= 10;
			}
		}
		else {
			rightDigit[0] = '-';
			for(int j = 1; j < 5; j++) {
				rightDigit[j] = (char)(((int)'0')+testRight%10);
				testRight /= 10;
			}
		}
		int leftByte = serial.SendData(leftDigit, strlen(leftDigit));
		int rightByte = serial.SendData(rightDigit, strlen(rightDigit));
	}
	else
		printf("Error\n");

	system("PAUSE");
	return 0;
}