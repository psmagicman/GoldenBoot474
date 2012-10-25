#include <iostream>
#include <limits>
#include <stdio.h>
#include <conio.h>
#include <vector>
#include <string>

#include "MovementAlgorithm.h"
#include "global.h"
#include "Serial.h"

#include <opencv\cvaux.h>
#include <opencv2\opencv.hpp>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>

/* 
	sending data to xbee masking
	int i;
	char bytes1[2];
	bytes1[0] = i & 0xFF;
	bytes1[1] = (i >> 8) & 0xFF;

	int j;
	char bytes2[2];
	bytes2[0] = j & 0xFF;
	bytes2[1] = (j >> 8) & 0xFF;

	reassembling data in arduino
	char bytes1[2];
	char bytes2[2];
	
	int i = (bytes1[0] & (bytes1[1] << 8));
	int j = (bytes2[0] & (bytes2[1] << 8));

*/

using namespace std;

int main() {
	Robot robot;
	Obstacle obstacle;
	vector<Ball> balls;
	CSerial serial;
	//vector<Obstacle> obstacle;

	while(1) {
		cout << "Enter the x coordinate of the robot: ";
		cin >> robot.x;
		cout << "Enter the y coordinate of the robot: ";
		cin >> robot.y;
		cout << "Enter the angle the robot is facing: ";
		cin >> robot.angle;
		/*cout << "Enter the x coordinate of the ball: ";
		cin >> ball.x;
		cout << "Enter the y coordinate of the ball: ";
		cin >> ball.y;
		cout << "Enter the x coordinate of the obstacle: ";
		cin >> obstacle.x;
		cout << "Enter the y coordinate of the obstacle: ";
		cin >> obstacle.y;*/

		int numBalls;
		cout << "Enter the number of balls: ";
		cin >> numBalls;
		balls.resize(numBalls);
		for(int i = 0; i < balls.size(); i++) {
			cout << "Coordinate of Ball" << i+1 << ": " << endl;
			cout << "X: ";
			cin >> balls[i].x;
			cout << "Y: ";
			cin >> balls[i].y;
			cout << endl;
			balls[i].rad = 5;
		}

		MovementAlgorithm algos = MovementAlgorithm(robot, balls);
		int szMotorsLeft = algos.returnLeftSize();
		int szMotorsRight = algos.returnRightSize();
		if(szMotorsRight == szMotorsLeft)
			cout << "szMotorsRight == szMotorsLeft" << endl;
		else
			cout << "szMotorsRight =/= szMotorsLeft" << endl;
		for(int i = 0; i < szMotorsRight; i++) {
			cout << "Left Ticks: " << algos.returnLeftMotor()[i] << endl;
			int testLeft = algos.returnLeftMotor()[i];
			char* leftDigit = new char[5];
			if(testLeft > 0) {
				for(int j = 0; j < 5; j++) {
					leftDigit[j] = (char)(((int)'0')+testLeft%10);
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
			cout << "Right Ticks: " << algos.returnRightMotor()[i] << endl;
			int testRight = algos.returnRightMotor()[i];
			char* rightDigit = new char[5];
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
			if(serial.Open(3, 57600)) {
				int leftByteSent = serial.SendData(leftDigit, strlen(leftDigit));
				int rightByteSent = serial.SendData(rightDigit, strlen(rightDigit));
			}
			else
				cout << "ERROR" << endl;
		}
		if((cvWaitKey(10) & 255) == 27)
			break;
	}
	return 0;
}