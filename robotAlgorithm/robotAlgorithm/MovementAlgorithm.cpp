/* MovementAlgorithm.cpp */
/*
	each rotation consists of 64 ticks
	radius of wheel is 3.4cm/0.111549ft
	circumference of wheel / 64 will get me 1 tick
	i.e. distance of ball to robot needs to be converted to ticks
			FOR STRAIGHT DISTANCE
			1 tick is approx. 0.333794219
			if distance is 100cm then 299.5857756 ticks
			round this number up by 1 will get 300 ticks => 299.5857756 + 1 then truncate to integer value

			FOR TURNING DISTANCE with prototype build (12cm width from motor to motor)
			3pi distance to turn 90 degrees total with both motors
			to turn 90 degrees: 3pi/0.333794219 ticks are needed for right motor = 27.2300 => 28 ticks 
								-3pi/0.333794219 ticks are needed for the left motor = -27.2300 => -28 ticks
							if angle is negative give positive ticks to the left motor and negative ticks to the right motor
							if angle is positive give positive ticks to the right motor and negative ticks to the left motor
			to turn 5 degrees:	(pi/6)/0.333794219 ticks are needed for the right motor = 1.568627453 => 2 ticks
								-(pi/6)/0.333794219 ticks are needed for the left motor = -1.568627453 => -2 ticks
	Assume the arena size is 800x800
*/
#include "MovementAlgorithm.h"

#define PI 3.14159265
#define ONE_TICK 0.7008830378306
#define BOT_WIDTH 0.19685
#define AVOID_DISTANCE 0.0984252

/* Constructor */

MovementAlgorithm::MovementAlgorithm() {

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
	checkAngle(algoRobot.angle);
}

MovementAlgorithm::MovementAlgorithm(Robot robot, vector<Ball> balls, vector<Obstacle> obstacles) {
	algoRobot = robot;
	int numBalls = balls.size();
	algoBalls.resize(numBalls);
	for(int i = 0; i < balls.size(); i++) {
		algoBalls[i].x = balls[i].x;
		algoBalls[i].y = balls[i].y;
		algoBalls[i].rad = balls[i].rad;
	}
	int numObs = obstacles.size();
	algoObs.resize(numObs);
	for(int i = 0; i < obstacles.size(); i++) {
		algoObs[i].x = obstacles[i].x;
		algoObs[i].y = obstacles[i].y;
		algoObs[i].rad = obstacles[i].rad;
	}
	calcMultiBall();
	calcMultiObsDist();
	obsCirc = calcObsRange();
	compareMultiBallDist();
	checkAngle(algoRobot.angle);
	determineObsPath();
	if(obsFlag == 1)
		cout << "Obstacle is in the path" << endl;
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
vector<int> MovementAlgorithm::returnRightMotor() { return rightMotor; }
vector<int> MovementAlgorithm::returnLeftMotor() { return leftMotor; }
int MovementAlgorithm::returnLeftSize() { return leftMotor.size(); }
int MovementAlgorithm::returnRightSize() { return rightMotor.size(); }

void MovementAlgorithm::checkAngle(double botAngle) {
	if(botAngle > (angle-3) && botAngle < (angle+3))
		determineForward();
	else {
		determineTurning();
		determineForward();
	}
}

void MovementAlgorithm::determineForward() {
	ticks = calcForwardTicks();
	leftMotor.push_back(ticks);
	rightMotor.push_back(ticks);
}

// This method will determine which motor will receive positive ticks 
// and which will receive negative ticks
void MovementAlgorithm::determineTurning() {
	ticks = calcTurnTicks();
	if(angle < 0 ) {
		// left gets positive ticks
		leftMotor.push_back(ticks);
		rightMotor.push_back(-ticks);
	}
	else {
		leftMotor.push_back(-ticks);
		rightMotor.push_back(ticks);
	}
}

int MovementAlgorithm::calcForwardTicks() {
	double tempTick;
	tempTick = (finalBallDist / ONE_TICK) + 1.0;
	cout << "Forward Ticks = " << tempTick << endl;
	return (int) tempTick;
}

int MovementAlgorithm::calcTurnTicks() {
	double tempTick;
	tempTick = 2*PI*BOT_WIDTH;
	tempTick = tempTick * angle / 360;
	tempTick = tempTick / ONE_TICK;
	tempTick = tempTick + 1.0;
	cout << "Turn Ticks = " << tempTick << endl;
	return (int)abs(tempTick);
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
		//ballsSlope[i] = tempY/tempX;
		cout << "Ball" << i+1 << ": " << ballsDist[i] << "feet."<< endl;
	}
}

// This method will calculate the distances of the obstacles from the robot
// and save it into the obsDist vector
void MovementAlgorithm::calcMultiObsDist() {
	double tempX, tempY;
	obsDist.resize(algoObs.size());
	for(int i = 0; i < obsDist.size(); i++) {
		tempX = (double)algoObs[i].x - (double)algoRobot.x;
		tempY = (double)algoObs[i].y - (double)algoRobot.y;
		obsDist[i] = (double)sqrt(pow(tempX,2)+pow(tempY,2));
		//obsSlope[i] = tempY/tempX;
		cout << "Obstacle" << i+1 << ": " << obsDist[i] << "feet." << endl;
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
	cout << "Ball" << ballNum << " has a distance of " << temp << "feet." << endl;
	calcMultiBallAngle(ballNum);
	cout << "Robot needs to turn: " << angle << " degrees..." << endl;
}

void MovementAlgorithm::calcMultiBallAngle(int ballNum) {
	actualBall = ballNum - 1;
	double x, y;
	x = algoBalls[actualBall].x - algoRobot.x;
	y = algoBalls[actualBall].y - algoRobot.y;
	angle = atan2(y, x) * 180 / PI;
}

double MovementAlgorithm::calcObsRange() {
	double temp;
	temp = 2 * PI * algoObs[0].rad;
	return temp;
}

// Determines if every obstacle that is within the path to the closest ball
// NOTE: highly inefficient method, will develop a better way to do this later
// AT THE MOMENT THIS WILL ONLY WORK FOR ONE OBSTACLE.
void MovementAlgorithm::determineObsPath() {
	obsFlag = 0;
	//int tempObsX, tempObsY;
	int countY = algoRobot.y;
	for(int x = algoRobot.x; x <= abs(algoBalls[actualBall].x); x++) {
		for(int i = 0; i < algoObs.size(); i++) {
			if((double)x >= (double)(algoObs[i].x - algoObs[i].rad - AVOID_DISTANCE) && 
				(double)x <= (double)(algoObs[i].x + algoObs[i].rad + AVOID_DISTANCE) &&
				(double)countY >= (double)(algoObs[i].y - algoObs[i].rad - AVOID_DISTANCE) &&
				(double)countY <= (double)(algoObs[i].y + algoObs[i].rad + AVOID_DISTANCE)) {
							obsFlag = 1;
							break;
			}
			else
				obsFlag = 0;
		}
		if(obsFlag == 1)
			break;
		countY++;
	}
}