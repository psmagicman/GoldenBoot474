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
	~MovementAlgorithm();

	double returnBallDist();
	double returnObsDist();
	double returnObsRange();
	double returnBotAngle();
	bool returnMoveFlag();
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
	bool moveFlag;
	/*vector<Ball> algoBall;
	vector<Obstacle> algoObstacle;
	vector<double> ballDist;
	vector<double> obsDist;
	vector<double> obsRange;*/

	void calcBallDist();
	void calcObsRange();
	void turnRobot2Ball();
	void checkAngle(double);
};

#endif MOVEMENTALGORITHM_H_