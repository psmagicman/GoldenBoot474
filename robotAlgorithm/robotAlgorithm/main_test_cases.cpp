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

		/*cout << "Enter the x coordinate of the robot: ";
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
		}*/
		/*int numObs = 0;
		
		if(numObs > 0)
			algos = MovementAlgorithm(robot, balls, obstacles);
		else
			algos = MovementAlgorithm(robot, balls);

		int szMotorsLeft = algos.returnLeftSize();
		int szMotorsRight = algos.returnRightSize();
		Coord2D closeBall;
		closeBall = algos.returnClosestBall();
		cout << "Closest ball coordinates: (" << closeBall.x << "," << closeBall.y << ")" << endl;
		for(int i = 0; i < szMotorsRight; i++) {
			cout << "Left Ticks" << i+1 << ": " << algos.returnLeftMotor()[i] << endl;
			cout << "Right Ticks" << i+1 << ": " << algos.returnRightMotor()[i] << endl;
		}*/
		int numBalls = 1;
		balls.resize(numBalls);
		for(int i = 0; i < 108; i++) {
			while(i >= 0 && i < 27) {
				robot.x = 0;
				robot.y = 0;
				if(i >= 0 && i < 3)
					robot.angle = 0;
				else if(i % 3 == 0)
					robot.angle = robot.angle + 45;
				if(i % 3 == 1) {
					balls[0].x = 0;
					balls[0].y = 8;
					break;
				}
				else if(i % 3 == 2) {
					balls[0].x = 8;
					balls[0].y = 0;
					break;
				}
				else {
					balls[0].x = 8;
					balls[0].y = 8;
					break;
				}
			}
			while(i >= 27 && i < 54) {
				robot.x = 0;
				robot.y = 8;
				if(i >= 27 && i < 30)
					robot.angle = 0;
				else if(i % 3 == 0)
					robot.angle = robot.angle + 45;
				if(i % 3 == 1) {
					balls[0].x = 0;
					balls[0].y = 0;
					break;
				}
				else if(i % 3 == 2) {
					balls[0].x = 8;
					balls[0].y = 0;
					break;
				}
				else {
					balls[0].x = 8;
					balls[0].y = 8;
					break;
				}
			}
			while(i >= 54 && i < 81) {
				robot.x = 8;
				robot.y = 0;
				if(i >= 0 && i < 3)
					robot.angle = 0;
				else if(i % 3 == 0)
					robot.angle = robot.angle + 45;
				if(i % 3 == 1) {
					balls[0].x = 0;
					balls[0].y = 0;
					break;
				}
				else if(i % 3 == 2) {
					balls[0].x = 0;
					balls[0].y = 8;
					break;
				}
				else {
					balls[0].x = 8;
					balls[0].y = 8;
					break;
				}
			}
			while(i >= 81 && i < 108) {
				robot.x = 8;
				robot.y = 8;
				if(i >= 81 && i < 108)
					robot.angle = 0;
				else if(i % 3 == 0)
					robot.angle = robot.angle + 45;
				if(i % 3 == 1) {
					balls[0].x = 0;
					balls[0].y = 0;
					break;
				}
				else if(i % 3 == 2) {
					balls[0].x = 8;
					balls[0].y = 0;
					break;
				}
				else {
					balls[0].x = 0;
					balls[0].y = 8;
					break;
				}
			}
			algos = MovementAlgorithm(robot, balls);
			cout << "Robot Angle: " << robot.angle;
			cout << " Final Angle: " << algos.returnAngle();
			cout << " Left Ticks: " << algos.returnLeftMotor()[0];
			cout << " Right Ticks: " << algos.returnRightMotor()[0] << endl;
		}
		/*vector<Coord2D> path;
		path = algos.returnPath();
		cout << "Size of path variable is: " << path.size() << endl;
		for(int i = 0; i < path.size(); i++) {
			cout << "Path[x] " << i+1 << " = " << path[i].x << endl;
			cout << "Path[y] " << i+1 << " = " << path[i].y << endl;
		}*/
	system("PAUSE");
	return 0;
}