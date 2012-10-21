/* MovementAlgorithm.h */
#ifndef MOVEMENTALGORITHM_H_
#define MOVEMENTALGORITHM_H_

#include <iostream>
#include <math.h>
#include <vector>

#include "global.h"

using namespace std;

class MovementAlgorithm {
public:
	/* Constructor */
	//MovementAlgorithm(Robot, vector<Ball>, vector<Obstacle>);
	MovementAlgorithm(Robot, Ball, Obstacle);
	MovementAlgorithm(Robot, vector<Ball>);
	~MovementAlgorithm();

	double returnBallDist();
	double returnObsDist();
	double returnObsRange();
	double returnBotAngle();
	double returnball2obs();
	bool returnMoveFlag();
	bool returnTurnFlag();
	int returnRightMotor();
	int returnLeftMotor();
	/*vector<double> returnBallDist();
	vector<double> returnObsDist();*/
	
private:
	Robot algoRobot;
	Ball algoBall;
	Obstacle algoObs;
	double ballDist;
	double obsDist;
	double obsRange;
	double angle;
	double ball2obs;
	bool moveFlag;
	bool turnFlag;

	int rightMotor;
	int leftMotor;
	vector<Ball> algoBalls;
	//vector<Obstacle> algoObstacle;
	vector<double> ballsDist;
	/*vector<double> obsDist;
	vector<double> obsRange;*/

	void calcBallDist();
	void calcObsRange();
	void turnRobot2Ball();
	void checkAngle(double);
	void calcball2obs();
};

#endif MOVEMENTALGORITHM_H_