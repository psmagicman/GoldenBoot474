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
#define ONE_TICK 0.34611696
#define BOT_WIDTH 6

/* Constructor */
/*MovementAlgorithm::MovementAlgorithm(Robot robot, Ball ball, Obstacle obstacle) {
	algoRobot = robot;
	algoBall = ball;
	algoObs = obstacle;
	calcBallDist();
	calcObsRange();
	calcball2obs();
	turnRobot2Ball();
	checkAngle(algoRobot.angle);
}*/

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
	checkAngle(algoRobot.angle);
}

MovementAlgorithm::~MovementAlgorithm() {}

/* Getter Functions */
double MovementAlgorithm::returnBallDist() { return finalBallDist; }
//double MovementAlgorithm::returnObsDist() { return obsDist; }
//double MovementAlgorithm::returnObsRange() { return obsRange; }
double MovementAlgorithm::returnBotAngle() { return algoRobot.angle; }
//double MovementAlgorithm::returnball2obs() { return ball2obs; }
bool MovementAlgorithm::returnMoveFlag() { return moveFlag; }
bool MovementAlgorithm::returnTurnFlag() { return turnFlag; }
int MovementAlgorithm::returnRightMotor() { return rightMotor; }
int MovementAlgorithm::returnLeftMotor() { return leftMotor; }

/*void MovementAlgorithm::calcObsRange() {
	double tempX, tempY;
	algoObs.rad = 10;
	tempX = algoObs.x - algoRobot.x;
	tempY = algoObs.y - algoRobot.x;
	obsDist = sqrt(pow(tempX,2)+pow(tempY,2));
	obsRange = algoObs.rad * 2 * PI;
}*/

void MovementAlgorithm::checkAngle(double botAngle) {
	if(botAngle > (angle-3) && botAngle < (angle+3))
		determineForward();
	else 
		determineTurning();
}

void MovementAlgorithm::determineForward() {
	ticks = calcForwardTicks();
	leftMotor = ticks;
	rightMotor = ticks;
}

// This method will determine which motor will receive positive ticks 
// and which will receive negative ticks
void MovementAlgorithm::determineTurning() {
	ticks = calcTurnTicks();
	if(angle < 0 ) {
		// left gets positive ticks
		leftMotor = ticks;
		rightMotor = -ticks;
	}
	else {
		leftMotor = -ticks;
		rightMotor = ticks;
	}
}

int MovementAlgorithm::calcForwardTicks() {
	double tempTick;
	tempTick = (finalBallDist / ONE_TICK) + 1.0;
	return (int) tempTick;
}

int MovementAlgorithm::calcTurnTicks() {
	double tempTick;
	tempTick = (angle * (PI/180) * 2 * BOT_WIDTH) + 1.0;
	return (int)tempTick;
}

/*void MovementAlgorithm::calcball2obs() {
	double tempX, tempY;
	tempX = algoBall.x - algoObs.x;
	tempY = algoBall.y - algoObs.y;
	ball2obs = sqrt(pow(tempX,2)+pow(tempY,2));
}*/

// This method will calculate the distances of the balls from the robot
// and save it into the ballsDist vector
void MovementAlgorithm::calcMultiBall() {
	double tempX, tempY;
	ballsDist.resize(algoBalls.size());
	for(int i = 0; i < ballsDist.size(); i++) {
		tempX = (double)algoBalls[i].x - (double)algoRobot.x;
		tempY = (double)algoBalls[i].y - (double)algoRobot.y;
		ballsDist[i] = (double)sqrt(pow(tempX,2)+pow(tempY,2));
		cout << "Ball" << i+1 << ": " << ballsDist[i] << "cm."<< endl;
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
	finalBallDist = temp;
	cout << "Ball closest to the robot is ball" << ballNum << endl;
	cout << "Ball" << ballNum << " has a distance of " << temp << "cm." << endl;
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