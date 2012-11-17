#ifndef __TICKS__
#define __TICKS__

#include <math.h>
#include <vector>
#include <iostream>

#include "GLOBALS.h"

using namespace std;

class Ticks
{
public:
	Ticks();
	Ticks(Robot);
	
	void compareTicks(vector<vector<Coord2D>>);

	vector<Coord2D> getPath();
	vector<Coord2D> getTicks();
	
private:
	int _closest;
	vector<vector<Coord2D>> _ticks;
	vector<vector<Coord2D>> _paths;
	Robot _robot;

	double detFirstAngle(Coord2D, Coord2D);
	Coord2D calcForwardTicks(double);
	Coord2D calcTurnTicks(double);
	vector<Coord2D> calculateTicks(vector<Coord2D>);
};
#endif