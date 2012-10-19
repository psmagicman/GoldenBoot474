#include <iostream>
#include <limits>
#include <stdio.h>
#include <conio.h>
#include <vector>

#include "MovementAlgorithm.h"
#include "global.h"
#include <opencv2\opencv.hpp>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>


using namespace std;

int main() {
	Robot robot;
	Ball ball;
	Obstacle obstacle;
	/*vector<Ball> ball;
	vector<Obstacle> obstacle;*/

	cout << "Enter the x coordinate of the robot: ";
	cin >> robot.x;
	cout << "Enter the y coordinate of the robot: ";
	cin >> robot.y;
	cout << "Enter the angle the robot is facing: ";
	cin >> robot.angle;
	cout << "Enter the x coordinate of the ball: ";
	cin >> ball.x;
	cout << "Enter the y coordinate of the ball: ";
	cin >> ball.y;
	cout << "Enter the x coordinate of the obstacle: ";
	cin >> obstacle.x;
	cout << "Enter the y coordinate of the obstacle: ";
	cin >> obstacle.y;

	/*int numBalls;
	cout << "Enter the number of balls: ";
	cin >> numBalls;
	ball.resize(numBalls);
	for(int i = 0; i < ball.size(); i++) {
		cout << "Coordinate of Ball" << i << " X: ";
		cin >> ball[i].x;
		cout << " Y: ";
		cin >> ball[i].y;
		cout << endl;
		ball[i].rad = 5;
	}
	int numObs;
	cout << "Enter the number of obstacles: ";
	cin >> numObs;
	obstacle.resize(numObs);
	for(int i = 0; i < obstacle.size(); i++ ) {
		cout << "Coordinate of Obstacle" << i << " X: ";
		cin >> obstacle[i].x;
		cout << " Y: ";
		cin >> obstacle[i].y;
		cout << endl << endl;
		obstacle[i].rad = 10;
	}*/

	MovementAlgorithm algo = MovementAlgorithm(robot, ball, obstacle);
	cout << endl << "Robot distance to ball: " << algo.returnBallDist() << endl;
	cout << "Robot distance to obstacle: " << algo.returnObsDist() << endl;
	cout << "Obstacle Range: " << algo.returnObsRange() << endl;
	cout << "Robot old angle: " << robot.angle << endl;
	cout << "Robot new angle (facing ball angle): " << algo.returnBotAngle() << endl;
	if(algo.returnMoveFlag())
		cout << "Move robot..." << endl;
	else
		cout << "Robot is still turning..." << endl;

/*	vector<double> ballDist;
	ballDist.resize(numBalls);
	ballDist = algo.returnBallDist();
	for(int i = 0; i < ball.size(); i++) {
		cout << "Ball" << i << " Distance to: " << ballDist[i] << endl;
	}
	vector<double> obsDist;
	obsDist.resize(numObs);
	obsDist = algo.returnObsDist();
	for(int i = 0; i < obstacle.size(); i++) {
		cout << "Obstacle" << i << " Distance to: " << obsDist[i] << endl;
	}*/
	
	cout << endl << "Press ESC to exit...";
	while(1) {
		char key = getch();
		if(key == 27)
			break;
	}
	return 0;
}