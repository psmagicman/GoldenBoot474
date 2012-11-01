#include <iostream>
#include <limits>
#include <stdio.h>
#include <conio.h>
#include <vector>
#include <string>

#include "MovementAlgorithm.h"
#include "GLOBALS.h"
//#include "Serial.h"

#include <opencv\cvaux.h>
#include <opencv2\opencv.hpp>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>

#define OBSTACLE_RADIUS 0.164042
#define BALL_RADIUS 0.108268
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
	vector<Ball> balls;
	vector<Obstacle> obstacles;
	MovementAlgorithm algos;

		cout << "Enter the x coordinate of the robot: ";
		cin >> robot.x;
		cout << "Enter the y coordinate of the robot: ";
		cin >> robot.y;
		cout << "Enter the angle the robot is facing: ";
		cin >> robot.angle;

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
			balls[i].rad = BALL_RADIUS;
		}
		int numObs;
		cout << "Enter the number of obstacles: ";
		cin >> numObs;
		obstacles.resize(numObs);
		for(int i = 0; i < obstacles.size(); i++) {
			cout << "Coordinate of Obstacle" << i+1 << ": " << endl;
			cout << "X: ";
			cin >> obstacles[i].x;
			cout << "Y: ";
			cin >> obstacles[i].y;
			cout << endl;
			obstacles[i].rad = OBSTACLE_RADIUS;
		}
		if(numObs > 0)
			MovementAlgorithm algos = MovementAlgorithm(robot, balls, obstacles);
		else
			MovementAlgorithm algos = MovementAlgorithm(robot, balls);

		int szMotorsLeft = algos.returnLeftSize();
		int szMotorsRight = algos.returnRightSize();
		cout << "Closest ball coordinates: (" << algos.returnClosestBall().x << "," << algos.returnClosestBall().y << ")" << endl;
		for(int i = 0; i < szMotorsRight; i++) {
			cout << "Left Ticks" << i+1 << ": " << algos.returnLeftMotor()[i] << endl;
			cout << "Right Ticks" << i+1 << ": " << algos.returnRightMotor()[i] << endl;
		}
	system("PAUSE");
	return 0;
}