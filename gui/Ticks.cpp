#include "Ticks.h"

/* default constructor
 * creates a Ticks object and initialize the path index with the least amount of ticks to be -1
 */
Ticks::Ticks()
{
	_closest = -1;
}

/*
 * initializes the object with the robot starting position and the list of paths to check
 */
Ticks::Ticks(Robot robot, vector<vector<Coord2D>> paths)
{
	_closest = -1;
	_robot = robot;
	_paths = paths;
}

vector<Coord2D> Ticks::getPath()
{
	vector<Coord2D> empty;
	empty.clear();
	if(_closest >= 0)
		return _paths[_closest];
	else
		return empty;
}

vector<Coord2D> Ticks::getTicks()
{
	vector<Coord2D> empty;
	empty.clear();
	if(_closest >= 0)
		return _ticks[_closest];
	else
		return empty;
}

/*
 * compares each of the paths that are passed in and determines which path has the least amount of ticks
 */
void Ticks::compareTicks(vector<vector<Coord2D>> paths)
{
	_ticks.clear();	// clear the _ticks vector everytime this function is invoked
	// the for loop fills the _ticks vector with the number of ticks needed for each path
	for(int i = 0; i < paths.size(); i++) {
		_ticks.push_back(calculateTicks(paths[i]));
	}
	// the following code determines which path will have the least amount of ticks and set the variable _closest to the index of the path with the least total ticks
	if(_ticks.size() > 0) {
		double sum = 0;
		for(int i = 0; i < _ticks[0].size(); i++) {
			sum += abs(_ticks[0][i].x);
			_closest = 0;
		}

		for(int i = 0; i < _ticks.size(); i++) {
			double tempSum = 0;
			// calculate the total ticks of each path, then return the vector ticks that has the smallest ticks
			for(int j = 0; j < _ticks[i][j].x; j++) {
				tempSum += abs(_ticks[i][j].x);
			}
			if(sum > tempSum) {
				// does a comparison of the previous smallest sum with the current sum
				sum = tempSum;
				_closest = i;
				// remembers the index of the vector here in the path with the lowest amount of ticks are located
			}
		}
	}
}

/*
 * calculates the amount of ticks for each path that is passed into the function
 */
vector<Coord2D> Ticks::calculateTicks(vector<Coord2D> path)
{
	vector<Coord2D> ticks;
	Coord2D tempTicks;
	double firstAngle = detFirstAngle(path[1], path[0]);
	if(firstAngle > 0.01 || firstAngle < -0.01) {
		tempTicks = calcTurnTicks(firstAngle);
		ticks.push_back(tempTicks);
	}
	tempTicks = calcForwardTicks(dist(path[0].x, path[1].x, path[0].y, path[1].y));
	ticks.push_back(tempTicks);

	for(int i = 1; i < path.size()-1; i++) {
		double angleA, angleB, angleFinal, a, b, c;
		b = dist(path[i].x, path[i+1].x, path[i].y, path[i+1].y);
		angleA = angleRelative(path[i-1], path[i]) * (180/PI);
		angleB = angleRelative(path[i+1], path[i]) * (180/PI);
		if(angleA < angleB)
			angleA = angleA + 360;
		angleFinal = 180 - (angleA - angleB);
		if(angleA < angleB)
			angleFinal = -angleFinal;
		tempTicks = calcTurnTicks(angleFinal);
		ticks.push_back(tempTicks);
		tempTicks = calcForwardTicks(b);
		ticks.push_back(tempTicks);
	}
	return ticks;
}

/*
 * calculates the number of ticks needed to go to the destination
 * these ticks are only for the forward motion
 */
Coord2D Ticks::calcForwardTicks(double distance)
{
	Coord2D tempTicks;
	double tempTick;
	tempTick = ((distance/50)/ONE_TICK);
	tempTick = tempTick * 2.0;
	// x is left y is right
	tempTicks.x = tempTick;
	tempTicks.y = tempTick;
	return tempTicks;
}

/*
 * calculates the number of ticks needed to turn towards the passed in angle
 * these ticks are only for the turning motion
 */
Coord2D Ticks::calcTurnTicks(double angle)
{
	Coord2D tempTicks;
	double tempTick;
	tempTick = 2*PI*BOT_WIDTH;
	tempTick = tempTick*angle/360;
	tempTick = tempTick/ONE_TICK;
	tempTick = tempTick * 2.0;
	tempTick = abs(tempTick);
	// x is left y is right
	if(angle > 0.0 && angle < 180.0) {
		// left turn
		tempTicks.x = - tempTick;
		tempTicks.y = tempTick;
	}
	else {
		// right turn
		tempTicks.x = tempTick;
		tempTicks.y = -tempTick;
	}
	return tempTicks;
}

/*
 * determines the angle required for the first turn
 */
double Ticks::detFirstAngle(Coord2D targetPt, Coord2D basePt)
{
	double tempAngle = angleRelative(targetPt, basePt);
	tempAngle = tempAngle - _robot.angle;
	tempAngle = tempAngle * (180/PI);
	// if tempAngle is negative but in the 1st or 2nd quadrant we add 360 to it
	// else if tempAngle is positive but in the 3rd or 4th quadrant we subtract 360 from it
	// otherwise do nothing to change tempAngle
	// this is because we want to make sure that whatever angle we get will be between -180 ~ 180 degrees
	// this will allow us to make sure that the robot does not turn a less optimal direction
	if(tempAngle < -180)
		tempAngle = tempAngle + 360;
	else if(tempAngle > 180)
		tempAngle = tempAngle - 360;
	return tempAngle;
}