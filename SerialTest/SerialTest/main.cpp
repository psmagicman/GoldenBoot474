#include "Serial.h"
#include <iostream>
#include <assert.h>

using namespace std;

int main() {
	/*int i = 100;
	char bytes1[2];
	bytes1[0] = i & 0xFF;
	bytes1[1] = (i >> 8) & 0xFF;*/

	CSerial serial;

	//cout << "3,57600: " << serial.Open(3, 57600) << endl;
	if(serial.Open(3, 57600)) {
		int i = 100;
		char bytes[2];
		bytes[0] = i & 0xFF;
		bytes[1] = (i >> 8) & 0xFF;
		int j = -i;
		char bytes2[2];
		bytes2[0] = j & 0xFF;
		bytes2[1] = (j >> 8) & 0xFF;
		int nBytesSent = serial.SendData(bytes, strlen(bytes));
		int nBytesSent2 = serial.SendData(bytes2, strlen(bytes2));
		assert(nBytesSent == strlen(bytes));
		assert(nBytesSent2 == strlen(bytes2));
		cout << "i = " << i << endl;
		cout << "Number of bytes sent: " << nBytesSent << endl;
		cout << "bytes[0]: " << bytes[0] << " bytes[1]: " << bytes[1] << endl;
		cout << "j = " << j << endl;
		cout << "Number of bytes sent 2: " << nBytesSent2 << endl;
		cout << "bytes2[0]: " << bytes2[0] << " bytes2[1]: " << bytes2[1] << endl;
	}
	else
		cout << "Error" << endl;
	system("PAUSE");
	return 0;
}