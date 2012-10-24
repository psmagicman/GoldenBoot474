/* MovementAlgorithm.h */
#ifndef MOVEMENTALGORITHM_H_
#define MOVEMENTALGORITHM_H_

#include <iostream>
#include <math.h>
#include <vector>

#include "GLOBALS.h"

using namespace std;

class MovementAlgorithm {
public:
	/* Constructor */
	//MovementAlgorithm(Robot, vector<Ball>, vector<Obstacle>);
	//MovementAlgorithm(Robot, Ball, Obstacle);
	MovementAlgorithm(Robot, vector<Ball>);
	~MovementAlgorithm();

	double returnBallDist();
	/*double returnObsDist();
	double returnObsRange();*/
	double returnBotAngle();
	//double returnball2obs();
	bool returnMoveFlag();
	bool returnTurnFlag();
	int returnRightMotor();
	int returnLeftMotor();
	/*vector<double> returnBallDist();
	vector<double> returnObsDist();*/

	double getX() {return _closestX;};
	double getY() {return _closestY;};
	
private:
	Robot algoRobot;
	Ball algoBall;
	Obstacle algoObs;
	double finalBallDist;
	double obsDist;
	double obsRange;
	double angle;
	double ball2obs;
	bool moveFlag;
	bool turnFlag;

	double _closestX;
	double _closestY;

	int rightMotor;
	int leftMotor;
	int ticks;
	vector<Ball> algoBalls;
	//vector<Obstacle> algoObstacle;
	vector<double> ballsDist;
	//vector<double> obsDist;
	//vector<double> obsRange;


	//void calcObsRange();
	void checkAngle(double);
	//void calcball2obs();
	void calcMultiBall();
	void compareMultiBallDist();
	void calcMultiBallAngle(int);
	void determineForward();
	void determineTurning();
	int calcForwardTicks();
	int calcTurnTicks();
};

#endif MOVEMENTALGORITHM_H_