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

	
	void setOpponent(Obstacle opponent);
	void analyzeField(Robot robot, vector<Ball> balls);
	void analyzeObstacles(); 
	vector<Coord2D>		getPathToGoal(Robot robot, Coord2D goal);

	Robot				getRobot(){return _robot;};
	vector<Ball>		getBalls(){return _balls;};
	vector<Obstacle>	getObstacles(){return _obstacles;};
	vector<vector<Coord2D> > getAllPaths(){return _paths;};
	
	Ball				getClosestBall(){return _closestBall;};

private:
	Robot				_robot;
	vector<Ball>		_balls;
	vector<Obstacle>	_obstacles;
	vector<Obstacle>	_originalObstacles;
	vector<vector<Coord2D> > _paths;
	vector<vector<Coord2D> > _ticks;

	Ball				_closestBall;
	Coord2D				_safetyCoord;
	bool				_safetyFlag;

	int _closest; // Index of Closest Path/Ticks

	void				calcSafety();
	Coord2D				getNewPointAroundObstacle(Obstacle obstacle, Coord2D beginPts, Coord2D endPts);
	vector<Coord2D>		getTangentPointOfObstacle(Obstacle obstacle, Coord2D point);
	vector<Coord2D>		getPathToPoint(Coord2D point, double distance = 0);
};

#endif