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
	calcball2obs();
	turnRobot2Ball();
	checkAngle(algoRobot.angle);
}

MovementAlgorithm::MovementAlgorithm(Robot robot, vector<Ball> balls) {
	int numBalls = balls.size();
	algoBalls.resize(numBalls);
	for(int i = 0; i < balls.size(); i++) {
		algoBalls[i].x = balls[i].x;
		algoBalls[i].y = balls[i].y;
		algoBalls[i].rad = balls[i].rad;
	}
	for(int j = 0; j < algoBalls.size(); j++) {
		cout << "Ball" << j << ": ";
		cout << "X: " << algoBalls[j].x << " Y: " << algoBalls[j].y;
		cout << " Rad: " << algoBalls[j].rad << endl;
	}
}

MovementAlgorithm::~MovementAlgorithm() {}

/* Getter Functions */
double MovementAlgorithm::returnBallDist() { return ballDist; }
double MovementAlgorithm::returnObsDist() { return obsDist; }
double MovementAlgorithm::returnObsRange() { return obsRange; }
double MovementAlgorithm::returnBotAngle() { return algoRobot.angle; }
double MovementAlgorithm::returnball2obs() { return ball2obs; }
bool MovementAlgorithm::returnMoveFlag() { return moveFlag; }
bool MovementAlgorithm::returnTurnFlag() { return turnFlag; }
int MovementAlgorithm::returnRightMotor() { return rightMotor; }
int MovementAlgorithm::returnLeftMotor() { return leftMotor; }

void MovementAlgorithm::calcBallDist() {
	double tempX, tempY;
	tempX = algoBall.x - algoRobot.x;
	tempY = algoBall.y - algoRobot.y;
	ballDist = sqrt(pow(tempX,2)+pow(tempY,2));
}

void MovementAlgorithm::calcObsRange() {
	double tempX, tempY;
	algoObs.rad = 10;
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
	if(angle == botAngle){
		moveFlag = 1;
		turnFlag = 0;
		rightMotor = 3;
		leftMotor = 3;
	}
	else {
		moveFlag = 0;
		turnFlag = 1;
	}
}

void MovementAlgorithm::calcball2obs() {
	double tempX, tempY;
	tempX = algoBall.x - algoObs.x;
	tempY = algoBall.y - algoObs.y;
	ball2obs = sqrt(pow(tempX,2)+pow(tempY,2));
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