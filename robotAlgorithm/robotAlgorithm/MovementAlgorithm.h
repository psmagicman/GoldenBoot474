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
	//MovementAlgorithm(Robot, Ball, Obstacle);
	MovementAlgorithm();
	MovementAlgorithm(Robot, vector<Ball>);
	MovementAlgorithm(Robot, vector<Ball>, vector<Obstacle>);
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
	int returnBallY();
	int returnBallX();
	//vector<Coord2D> returnPathPoints();
	/*vector<double> returnBallDist();
	vector<double> returnObsDist();*/
	
private:
	Robot algoRobot;
	double finalBallDist;
	//double obsDist;
	double obsRange;
	double angle;
	double ball2obs;
	bool moveFlag;
	bool turnFlag;
	bool obsFlag;
	int actualBall;
	int actualObs;

	vector<int> rightMotor;
	vector<int> leftMotor;
	int ticks;
	vector<Ball> algoBalls;
	vector<Obstacle> algoObs;
	vector<double> ballsDist;
	vector<double> obsDist;
	//vector<double> ballsSlope;
	//vector<double> obsSlope;
	double obsCirc;
	//vector<double> obsRange;


	
	void checkAngle(double);
	void calcMultiBall();
	void compareMultiBallDist();
	void calcMultiBallAngle();
	void calcMultiObsDist();
	void determineForward();
	void determineTurning();
	void determineObsPath();
	void determineObsTurn();
	void determineObsForward();
	void determineObs2BallTurn();
	void determineObs2BallForward();
	int calcForwardTicks();
	int calcTurnTicks();
	int calcObsTurnTicks();
	int calcObsForwardTicks();
	int calcObs2BallForwardTicks();
	int calcObs2BallTurnTicks();
	void calcMultiObsAngle();
	double calcObsRange();
};

#endif MOVEMENTALGORITHM_H_