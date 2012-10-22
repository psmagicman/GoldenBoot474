/* MovementAlgorithm.cpp */
/*
	each rotation consists of 64 ticks
	radius of wheel is 3.5cm
	circumference of wheel / 64 will get me 1 tick
	i.e. distance of ball to robot needs to be converted to ticks
			FOR STRAIGHT DISTANCE
			1 tick is approx. 0.34611696cm
			if distance is 100cm then 288.91967 ticks
			round this number up by 1 will get 289 ticks => 288.919967 + 1 then truncate to integer value

			FOR TURNING DISTANCE with prototype build (12cm width from motor to motor)
			3pi distance to turn 90 degrees total with both motors
			to turn 90 degrees: 3pi/0.34611696 ticks are needed for right motor = 27.2300 => 28 ticks 
								-3pi/0.34611696 ticks are needed for the left motor = -27.2300 => -28 ticks
							if angle is negative give positive ticks to the left motor and negative ticks to the right motor
							if angle is positive give positive ticks to the right motor and negative ticks to the left motor
			to turn 5 degrees:	(pi/3)/0.34611696 ticks are needed for the right motor = 3.025559774 => 4 ticks
								-(pi/3)/0.34611696 ticks are needed for the left motor = -3.025559774 => -4 ticks

*/
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
	algoRobot = robot;
	int numBalls = balls.size();
	algoBalls.resize(numBalls);
	for(int i = 0; i < balls.size(); i++) {
		algoBalls[i].x = balls[i].x;
		algoBalls[i].y = balls[i].y;
		algoBalls[i].rad = balls[i].rad;
	}

	calcMultiBall();
	compareMultiBallDist();
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

// This method will calculate the distances of the balls from the robot
// and save it into the ballsDist vector
void MovementAlgorithm::calcMultiBall() {
	double tempX, tempY;
	ballsDist.resize(algoBalls.size());
	for(int i = 0; i < ballsDist.size(); i++) {
		tempX = (double)algoBalls[i].x - (double)algoRobot.x;
		tempY = (double)algoBalls[i].y - (double)algoRobot.y;
		ballsDist[i] = (double)sqrt(pow(tempX,2)+pow(tempY,2));
		cout << "Ball" << i+1 << ": " << ballsDist[i] << endl;
	}
}

void MovementAlgorithm::compareMultiBallDist() {
	double temp;
	int ballNum = 1;
	temp = ballsDist[0];
	for(int i = 0; i < ballsDist.size(); i++) {
		if(temp > ballsDist[i]) {
			temp = ballsDist[i];
			ballNum = i+1;
		}
	}
	cout << "Ball closest to the robot is ball" << ballNum << endl;
	cout << "Ball" << ballNum << " has a distance of " << temp << " units." << endl;
	calcMultiBallAngle(ballNum);
	cout << "Robot needs to turn: " << angle << " degrees..." << endl;
}

void MovementAlgorithm::calcMultiBallAngle(int ballNum) {
	int actualBall = ballNum - 1;
	double x, y;
	x = algoBalls[actualBall].x - algoRobot.x;
	y = algoBalls[actualBall].y - algoRobot.y;
	angle = atan2(y, x) * 180 / PI;
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