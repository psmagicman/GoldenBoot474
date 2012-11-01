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
	MovementAlgorithm();
	MovementAlgorithm(Robot, vector<Ball>);
	MovementAlgorithm(Robot robot, vector<Ball> balls, vector<Obstacle> obstacles);
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
	Coord2D returnClosestBall();
	/*vector<double> returnBallDist();
	vector<double> returnObsDist();*/

	double getX() {return _X;};
	double getY() {return _Y;};
	
private:
	Robot algoRobot;
	Ball algoBall;
	vector<Obstacle> algoObs;
	vector<double> obsDist;
	double finalBallDist;
	double obsRange;
	double angle;
	double ball2obs;
	bool moveFlag;
	bool turnFlag;
	bool obsFlag;
	double _X;
	double _Y;
	int actualBall;
	int actualObs;
	double obsCirc;

	vector<int> rightMotor;
	vector<int> leftMotor;
	Coord2D closestBall;
	int ticks;
	vector<Ball> algoBalls;
	//vector<Obstacle> algoObstacle;
	vector<double> ballsDist;
	//vector<double> obsDist;
	//vector<double> obsRange;

	void determineObsTurn();
	void determineObsForward();
	void determineObs2BallForward();
	void determineObs2BallTurn();

	int calcObsTurnTicks();
	int calcObsForwardTicks();
	int calcObs2BallForwardTicks();
	int calcObs2BallTurnTicks();

	void determineObsPath();
	double calcObsRange();
	void calcMultiBallAngle();
	void calcMultiObsDist();
	void calcMultiObsAngle();
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