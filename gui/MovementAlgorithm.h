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
	vector<int> returnRightMotor();
	vector<int> returnLeftMotor();
	int returnLeftSize();
	int returnRightSize();
	/*vector<double> returnBallDist();
	vector<double> returnObsDist();*/

	double getX() {return _X;};
	double getY() {return _Y;};
	
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
	double _X;
	double _Y;

	vector<int> rightMotor;
	vector<int> leftMotor;
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