#ifndef __ALGORITHM__
#define __ALGORITHM__

#include <math.h>
#include <vector>

#include "GLOBALS.h"

using namespace std;

const int _width = 800;
const int _height = 800;

class testAlgorithm
{
public:
	testAlgorithm();
	testAlgorithm(vector<Obstacle> obstacles);

	void analyzeField(Robot robot, vector<Ball> balls);

	/*
	Coord2D				getClosestBall();
	vector<Coord2D>		getPathToClosestBall();
	vector<Coord2D>		getPathToGoal();
	*/

	Robot				getRobot(){return _robot;};
	vector<Ball>		getBalls(){return _balls;};
	vector<Obstacle>	getObstacles(){return _obstacles;};
	

	vector<Coord2D>		getClosestPath();
	vector<Coord2D>		getClosestTick();
	vector<Coord2D>		getPathToGoal();
	vector<Coord2D>		getTickToGoal();

	
private:
	Robot				_robot;
	vector<Ball>		_balls;
	vector<Obstacle>	_obstacles;
	vector<vector<Coord2D> > _paths;
	vector<vector<Coord2D> > _ticks;

	int _closest; // Index of Closest Path/Ticks

	void analyzeObstacles(); 
	Coord2D				getNewPointAroundObstacle(Obstacle obstacle, Coord2D beginPts, Coord2D endPts);
	vector<Coord2D>		getTangentPointOfObstacle(Obstacle obstacle, Coord2D point);

	vector<Coord2D> calculateTicks(vector<Coord2D>);
	vector<Coord2D> compareTicks();
	Coord2D calcForwardTicks(double);
	Coord2D calcTurnTicks(double, Coord2D, Coord2D);
};

#endif