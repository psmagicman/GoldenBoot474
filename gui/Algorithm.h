#ifndef __ALGORITHM__
#define __ALGORITHM__

#include <math.h>
#include <vector>

#include "GLOBALS.h"

using namespace std;

const int _width = 800;
const int _height = 800;

class Algorithm
{
public:
	Algorithm();
	Algorithm(vector<Obstacle> obstacles);

	void analyzeField(Robot robot, vector<Ball> balls);

	int calcForwardTicks(double);
	int calcTurnTicks(double);
	/*
	Coord2D				getClosestBall();
	vector<Coord2D>		getPathToClosestBall();
	vector<Coord2D>		getPathToGoal();
	*/

	Robot				getRobot(){return _robot;};
	vector<Ball>		getBalls(){return _balls;};
	vector<Obstacle>	getObstacles(){return _obstacles;};
	
	Coord2D				getNewPointAroundObstacle(Obstacle obstacle, Coord2D beginPts, Coord2D endPts);
	vector<Coord2D>		getTangentPointOfObstacle(Obstacle obstacle, Coord2D point);

	
private:
	Robot				_robot;
	vector<Ball>		_balls;
	vector<Obstacle>	_obstacles;
	vector<vector<Coord2D> > _paths;
	vector<vector<double> > _angles;
	vector<vector<Coord2D> > _ticks;

	void analyzeObstacles();
	bool checkGTPoints(Coord2D, Coord2D, Coord2D);
	bool checkLTPoints(Coord2D, Coord2D, Coord2D); 
	void calcAngles();
};

#endif