/* MovementAlgorithm.cpp */

#include "MovementAlgorithm.h"
/* Constructor */
MovementAlgorithm::MovementAlgorithm(Robot robot, vector<Ball> ball, vector<Obstacle> obstacle) {
	algoRobot = robot;
	algoBall = ball;
	algoObstacle = obstacle;
}

MovementAlgorithm::~MovementAlgorithm() {}

/*vector<double> MovementAlgorithm::returnBallDist() {
	return ballDist;
}

vector<double> MovementAlgorithm::returnObsDist() {
	return obsDist;
}*/

/*void MovementAlgorithm::calcBallDist() {
	double tempX, tempY;
	ballDist.resize(algoBall.size());
	for(int i = 0; i < ballDist.size(); i++) {
		tempX = algoBall[i].x - algoRobot.x;
		tempY = algoBall[i].y - algoRobot.y;
		ballDist[i] = sqrt((pow(tempX,2)+pow(tempY,2)));
	}
}

void MovementAlgorithm::calcObsRange() {
	double tempX, tempY;
	obsDist.resize(algoObstacle.size());
	for(int i = 0; i < obsDist.size(); i++) {
		tempX = algoObstacle[i].x - algoRobot.x;
		tempY = algoObstacle[i].y - algoRobot.y;
		obsDist[i] = sqrt((pow(tempX,2)+pow(tempY,2)));
		obsRange[i] = 2*3.14159*algoObstacle[i].rad;
	}
}*/