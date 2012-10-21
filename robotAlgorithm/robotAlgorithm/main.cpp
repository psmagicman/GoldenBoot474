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
	//vector<Ball> balls;
	//vector<Obstacle> obstacle;

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
	balls.resize(numBalls);
	for(int i = 0; i < balls.size(); i++) {
		cout << "Coordinate of Ball" << i << " X: ";
		cin >> balls[i].x;
		cout << " Y: ";
		cin >> balls[i].y;
		cout << endl;
		balls[i].rad = 5;
	}*/


	MovementAlgorithm algo = MovementAlgorithm(robot, ball, obstacle);
	cout << endl << "Robot distance to ball: " << algo.returnBallDist() << endl;
	cout << "Robot distance to obstacle: " << algo.returnObsDist() << endl;
	cout << "Obstacle distance to ball: " << algo.returnball2obs() << endl;
	cout << "Obstacle Range: " << algo.returnObsRange() << endl;
	cout << "Robot old angle: " << robot.angle << endl;
	cout << "Robot new angle (facing ball angle): " << algo.returnBotAngle() << endl;
	if(algo.returnMoveFlag() && !algo.returnTurnFlag()) {
		cout << "Sending move signal to left and right motors..." << endl;
		cout << "Right motor value: " << algo.returnRightMotor() << endl;
		cout << "Left motor value: " << algo.returnLeftMotor() << endl;
		cout << "Robot will move straight..." << endl;
	}
	else {
		cout << "Robot is still turning ";
	}
	
	cout << endl << "Press ESC to exit...";
	while(1) {
		char key = getch();
		if(key == 27)
			break;
	}
	return 0;
}