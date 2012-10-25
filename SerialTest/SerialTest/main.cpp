#include "Serial.h"
#include <stdio.h>
#include <assert.h>
#include <Windows.h>


int main() {
	CSerial serial;
	if(serial.Open(4, 9600)) {
		const char* buffer = new char[8];
		buffer = "Hello";
		int nBytesSent = serial.SendData(buffer, strlen(buffer));
		assert(nBytesSent == strlen(buffer));
		printf("buffer = %s\n", buffer);
		Sleep(10000);
		char* lpBuffer = new char[8];
		int nBytesRead = serial.ReadData(lpBuffer, 8);
		printf("nBytesRead = %d\n", nBytesRead);
		printf("lpBuffer = %s\n", lpBuffer);
		delete []lpBuffer;
	}
	else
		printf("Error\n");

	system("PAUSE");
	return 0;
}