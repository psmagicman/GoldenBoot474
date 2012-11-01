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
	100 units is 1 ft
*/
#include "MovementAlgorithm.h"

#define PI 3.14159265
#define ONE_TICK 0.0113
#define BOT_WIDTH 0.172244
#define AVOID_DISTANCE 0.262467

/* Constructor */

MovementAlgorithm::MovementAlgorithm() {}

MovementAlgorithm::MovementAlgorithm(Robot robot, vector<Ball> balls) {
	algoRobot = robot;
	int numBalls = balls.size();
	int numObs = 0;
	obsFlag = 0;
	algoBalls.resize(numBalls);
	for(int i = 0; i < balls.size(); i++) {
		algoBalls[i].x = balls[i].x * 100;		// coordinates are given in ft multiply by 100 to get the conversion to 800x800
		algoBalls[i].y = balls[i].y * 100;
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
		algoBalls[i].x = balls[i].x * 100;
		algoBalls[i].y = balls[i].y * 100;
		algoBalls[i].rad = balls[i].rad;
	}
	int numObs = obstacles.size();
	algoObs.resize(numObs);
	for(int i = 0; i < obstacles.size(); i++) {
		algoObs[i].x = obstacles[i].x * 100;
		algoObs[i].y = obstacles[i].y * 100;
		algoObs[i].rad = obstacles[i].rad;
	}
	calcMultiBall();
	calcMultiObsDist();
	obsCirc = calcObsRange();
	compareMultiBallDist();
	determineObsPath();
	checkAngle(algoRobot.angle);
	if(obsFlag == 1)
		cout << "Obstacle is in the path" << endl;
	else
		cout << "Obstacle is not in the path" << endl;
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
	double tempAngle;
	tempAngle = botAngle * (180/PI);
	calcMultiBallAngle();
	if(botAngle > (angle-5) && botAngle < (angle+5))
		determineForward();
	else if(obsFlag) {
		determineObsTurn();
		determineObsForward();
		determineObs2BallTurn();
		determineObs2BallForward();
	}
	else {
		cout << "Robot needs to turn: " << angle << " degrees..." << endl;
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
	if(angle < 0) {
		// left gets positive ticks
		leftMotor.push_back(ticks);
		rightMotor.push_back(-ticks);
	}
	else {
		leftMotor.push_back(-ticks);
		rightMotor.push_back(ticks);
	}
}

void MovementAlgorithm::determineObsTurn() {
	ticks = calcObsTurnTicks();
	if(angle < 0) {
		leftMotor.push_back(ticks);
		rightMotor.push_back(-ticks);
	}
	else {
		leftMotor.push_back(-ticks);
		rightMotor.push_back(ticks);
	}
}

void MovementAlgorithm::determineObsForward() {
	ticks = calcObsForwardTicks();
	leftMotor.push_back(ticks);
	rightMotor.push_back(ticks);
}

void MovementAlgorithm::determineObs2BallForward() {
	ticks = calcObs2BallForwardTicks();
	leftMotor.push_back(ticks);
	rightMotor.push_back(ticks);
}

void MovementAlgorithm::determineObs2BallTurn() {

}

int MovementAlgorithm::calcForwardTicks() {
	double tempTick;
	tempTick = ((finalBallDist/100) / ONE_TICK) + 1.0;
	cout << "Forward Ticks = " << tempTick << endl;
	return (int) tempTick;
}

int MovementAlgorithm::calcTurnTicks() {
	double tempTick;
	calcMultiBallAngle();
	tempTick = 2*PI*BOT_WIDTH;
	tempTick = tempTick * angle / 360;
	tempTick = tempTick / ONE_TICK;
	tempTick = tempTick + 1.0;
	cout << "Turn Ticks = " << tempTick << endl;
	return (int)abs(tempTick);
}

int MovementAlgorithm::calcObsTurnTicks() {
	double tempTick;
	calcMultiObsAngle();
	tempTick = 2*PI*BOT_WIDTH;
	tempTick = tempTick * angle / 360;
	tempTick = tempTick / ONE_TICK;
	tempTick = tempTick + 1.0;
	cout << "Turn ticks to avoid obstacle = " << tempTick << endl;
	return (int)abs(tempTick);
}

int MovementAlgorithm::calcObsForwardTicks() {
	double tempTick;
	tempTick = (obsDist[actualObs] / ONE_TICK) + 1.0;
	cout << "Forward ticks to avoid obstacle = " << tempTick << endl;
	return (int)tempTick;
}

int MovementAlgorithm::calcObs2BallForwardTicks() {
	double tempTick;
	double x, y, hypo;
	x = (double)algoBalls[actualBall].x - ((double)algoObs[actualBall].x + algoObs[0].rad + AVOID_DISTANCE);
	y = (double)algoBalls[actualBall].y - ((double)algoObs[actualBall].y + algoObs[0].rad + AVOID_DISTANCE);
	hypo = (double)sqrt((double)pow(x,2)+(double)pow(y,2));
	tempTick = (hypo / ONE_TICK) + 1.0;
	cout << "Forward Ticks = " << tempTick << endl;
	return (int)tempTick;
}

int MovementAlgorithm::calcObs2BallTurnTicks() {
	double tempTick;
	double x, y;
	x = (double)algoBalls[actualBall].x - ((double)algoObs[actualBall].x + algoObs[0].rad + AVOID_DISTANCE);
	y = (double)algoBalls[actualBall].y - ((double)algoObs[actualBall].y + algoObs[0].rad + AVOID_DISTANCE);
	angle = atan2(y, x) * 180 / PI;
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
	double x, y;
	double tempX, tempY;
	tempX = algoRobot.x * 100;
	tempY = algoRobot.y * 100;
	ballsDist.resize(algoBalls.size());
	for(int i = 0; i < ballsDist.size(); i++) {
		x = algoBalls[i].x - algoRobot.x;
		y = algoBalls[i].y - algoRobot.y;
		ballsDist[i] = sqrt(pow(x,2)+pow(y,2));
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
	actualBall = ballNum -1;
	_X = algoBalls[actualBall].x;
	_Y = algoBalls[actualBall].y;
}

void MovementAlgorithm::calcMultiBallAngle() {
	double x, y;
	x = algoBalls[actualBall].x - algoRobot.x;
	y = algoBalls[actualBall].y - algoRobot.y;
	if ( abs(x) < 0.01 ) {
		if (y > 0)
			angle = 90;
		else
			angle = 270;
	}
	else if(abs(y) < 0.01) {
		if(x > 0)
			angle = 0;
		else
			angle = 180;
	}
	else {
		// default angle is for quadrant 1
		angle = atan2(abs(y),abs(x)) * (180/PI);	// will always give positive angles
		if((algoBalls[actualBall].x < algoRobot.x) && (algoBalls[actualBall].y > algoRobot.y))
			angle = 180 - angle;	// angle for quadrant 2
		else if((algoBalls[actualBall].x < algoRobot.x) && (algoBalls[actualBall].y < algoRobot.y))
			angle = 270 - angle;	// angle for quadrant 3
		else if((algoBalls[actualBall].x > algoRobot.x) && (algoBalls[actualBall].y > algoRobot.y))
			angle = 360 - angle;	// angle for quadrant 4
	}
}

void MovementAlgorithm::calcMultiObsAngle() {
	double x, y;
	double tempRange;
	tempRange = algoObs[0].rad + AVOID_DISTANCE;
	x = algoObs[actualObs].x + tempRange - algoRobot.x;
	y = algoObs[actualObs].y + tempRange - algoRobot.y;
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
			if((double)x >= (double)(algoObs[i].x - algoObs[i].rad/* - AVOID_DISTANCE*/) && 
				(double)x <= (double)(algoObs[i].x + algoObs[i].rad/* + AVOID_DISTANCE*/) &&
				(double)countY >= (double)(algoObs[i].y - algoObs[i].rad/* - AVOID_DISTANCE*/) &&
				(double)countY <= (double)(algoObs[i].y + algoObs[i].rad/* + AVOID_DISTANCE*/)) {
							obsFlag = 1;
							actualObs = i;
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