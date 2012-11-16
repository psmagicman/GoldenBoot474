#ifndef __ALGORITHM__
#define __ALGORITHM__

#include <math.h>
#include <vector>

#include "GLOBALS.h"

using namespace std;

const int _width = 800;
const int _height = 800;

class CAlgorithm
{
public:
	CAlgorithm();
	CAlgorithm(vector<Obstacle> obstacles);

	void analyzeField(Robot robot, vector<Ball> balls);

	Robot				getRobot(){return _robot;};
	vector<Ball>		getBalls(){return _balls;};
	vector<Obstacle>	getObstacles(){return _obstacles;};
	vector<vector<Coord2D> > getAllPaths(){return _paths;};
	
	Ball				getClosestBall(){return _closestBall;};

	vector<Coord2D>		getClosestPath();
	vector<Coord2D>		getClosestTick();
	vector<Coord2D>		getPathToGoal();
	vector<Coord2D>		getTickToGoal();
	
	vector<Coord2D>		getTangentPointOfObstacle(Obstacle obstacle, Coord2D point);
private:
	Robot				_robot;
	vector<Ball>		_balls;
	vector<Obstacle>	_obstacles;
	vector<vector<Coord2D> > _paths;
	vector<vector<Coord2D> > _ticks;

	Ball				_closestBall;

	int _closest; // Index of Closest Path/Ticks

	void analyzeObstacles(); 
	Coord2D				getNewPointAroundObstacle(Obstacle obstacle, Coord2D beginPts, Coord2D endPts);

	/*vector<Coord2D> calculateTicks(vector<Coord2D>);
	void compareTicks();
	Coord2D calcForwardTicks(double);
	Coord2D calcTurnTicks(double);
	double detFirstAngle(Coord2D, Coord2D);*/
};

#endif