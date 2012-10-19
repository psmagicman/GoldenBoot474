/* MovementAlgorithm.cpp */

#include "MovementAlgorithm.h"

#define PI 3.14159265

/* Constructor */
MovementAlgorithm::MovementAlgorithm(Robot robot, Ball ball, Obstacle obstacle) {
	algoRobot = robot;
	algoBall = ball;
	algoObs = obstacle;
	calcBallDist();
	calcObsRange();
	turnRobot2Ball();
	algoRobot.angle = angle;
	checkAngle(algoRobot.angle);
}

MovementAlgorithm::~MovementAlgorithm() {}

double MovementAlgorithm::returnBallDist() {
	return ballDist;
}

double MovementAlgorithm::returnObsDist() {
	return obsDist;
}

double MovementAlgorithm::returnObsRange() {
	return obsRange;
}

double MovementAlgorithm::returnBotAngle() {
	return algoRobot.angle;
}

bool MovementAlgorithm::returnMoveFlag() {
	return moveFlag;
}

void MovementAlgorithm::calcBallDist() {
	double tempX, tempY;
	tempX = algoBall.x - algoRobot.x;
	tempY = algoBall.y - algoRobot.y;
	ballDist = sqrt(pow(tempX,2)+pow(tempY,2));
}

void MovementAlgorithm::calcObsRange() {
	double tempX, tempY;
	tempX = algoObs.x - algoRobot.x;
	tempY = algoObs.y - algoRobot.x;
	obsDist = sqrt(pow(tempX,2)+pow(tempY,2));
	obsRange = algoObs.rad * 2 * PI;
}

void MovementAlgorithm::turnRobot2Ball() {
	double tempX, tempY;
	tempX = algoBall.x - algoRobot.x;
	tempY = algoBall.y - algoRobot.y;
	angle = atan2(tempY, tempX) * 180 / PI;
}

void MovementAlgorithm::checkAngle(double botAngle) {
	if(angle == botAngle)
		moveFlag = 1;
	else
		moveFlag = 0;
}
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