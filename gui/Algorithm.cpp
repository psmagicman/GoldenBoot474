#include "Algorithm.h"

CAlgorithm::CAlgorithm()
{
	_closest = -1;
}

CAlgorithm::CAlgorithm(vector<Obstacle> obstacles)
{
	_obstacles = obstacles;
	for (int i = 0; i < _obstacles.size(); i++) {
		_obstacles[i].rad = _safetyRadius;
	}
	analyzeObstacles();
	_closest = -1;
}

vector<Coord2D> CAlgorithm::getClosestPath()
{
	vector<Coord2D> empty;
	empty.clear();
	if (_closest >= 0)
		return _paths[_closest];
	else
		return empty;
}

vector<Coord2D> CAlgorithm::getClosestTick()
{
	vector<Coord2D> empty;
	empty.clear();
	if (_closest >= 0)
		return _ticks[_closest];
	else
		return empty;
}

vector<Coord2D> CAlgorithm::getPathToGoal()
{
	vector<Coord2D> empty;
	empty.clear();

	return empty;
}

vector<Coord2D> CAlgorithm::getTickToGoal()
{
	vector<Coord2D> empty;
	empty.clear();

	return empty;
}

void CAlgorithm::analyzeField(Robot robot, vector<Ball> balls)
{
	_closest = -1;
	_robot = robot;
	_balls = balls;
	_paths.clear();
	
	Coord2D beginPts; // Beginning Coordinates of a path
	Coord2D endPts; // Ending Coordinates of a path

	// Cycle through all the Balls, and construct a Path to each ball
	for (int iBall = 0; iBall < _balls.size(); iBall++) {
		vector<Coord2D> path;
		path.clear();
		/*
		bool validBall = true;
		for (int i = 0; i < _obstacles.size(); i++) {
			if ( dist(_balls[iBall].x, _obstacles[i].x, _balls[iBall].y, _obstacles[i].y) <= _obstacles[i].rad) {
				validBall = false;
				break;
			}
		}
		if (!validBall) {
			_paths.push_back(path);
			continue;
		}
		*/
		// Set Beginning Coordinates to Robot
		beginPts.x = _robot.x;
		beginPts.y = _robot.y;
		path.push_back(beginPts);
		endPts.x = _balls[iBall].x;
		endPts.y = _balls[iBall].y;
		path.push_back(endPts);
		// If there are obstacles in the field, detect if there are any obstacles in the way
		if (_obstacles.size() > 0) {
			// Loop through the path (Look at pop_back and push_back at the bottom. If there are obstacles detected along the way, then a newPath will be pushed
			for (int iPath = 0; iPath < path.size()-1; iPath++) {
				// Select Obstacles that are between the Robot to the Ball
				beginPts = path[iPath]; // Reassign Beginning Coordinates to the last saved path coordinate
				// Create a Obstacle variable that is far outside of the arena
				Obstacle obstaclePts;
				obstaclePts.x = -_width;
				obstaclePts.y = -_height;
				for (int iObstacle = 0; iObstacle < _obstacles.size(); iObstacle++) {
					Coord2D tempObstaclePts;
					tempObstaclePts.x = _obstacles[iObstacle].x;
					tempObstaclePts.y = _obstacles[iObstacle].y;
					double angleObstaclePath = cosineLaw(
													dist(beginPts.x, endPts.x, beginPts.y, endPts.y), 
													dist(beginPts.x, tempObstaclePts.x, beginPts.y, tempObstaclePts.y),
													dist(endPts.x, tempObstaclePts.x, endPts.y, tempObstaclePts.y)
												)*180/PI;
					double distObstaclePath = distFromLine(tempObstaclePts, endPts, beginPts);
					if (	angleObstaclePath < 90 && // Check if the Angle between the Obstacle and the Path is less than 90 degrees. If it is, that means it can intercept with the path
							distObstaclePath < _obstacles[iObstacle].rad && // Check if the distance of the Obstacle to the line is less than the radius of the Obstacle
							dist(obstaclePts.x, beginPts.x, obstaclePts.y, beginPts.y) > dist(_obstacles[iObstacle].x, beginPts.x, _obstacles[iObstacle].y, beginPts.y) // Check if the newer obstacle is closer
						) 
					{
						obstaclePts = _obstacles[iObstacle]; // Closest Obstacle and it's length to the path is less than its radius
					}
				}
				// If an obstacle is detected to be in-between the path, add a new point to the path
				if (obstaclePts.x >= 0 && obstaclePts.y >= 0) {
					Coord2D newPath = getNewPointAroundObstacle(obstaclePts,beginPts,endPts);
					path[path.size()-1] = newPath;
					path.push_back(endPts);
				}
			}
		}
		_paths.push_back(path);
	}
	compareTicks();
}

void CAlgorithm::analyzeObstacles()
{
	vector<Obstacle> tempObstacles;
	for (int i = 0; i < _obstacles.size() && _obstacles[i].rad != 0; i++) 
	{
		vector<Obstacle> combinedObstacles;
		combinedObstacles.push_back(_obstacles[i]);
		// Combine Obstacles that does not have enough gap in-between for the robot to fit through
		for (int j = 0; j < _obstacles.size(); j++) 
		{
			if (i != j) {
				double obstacleDistance = dist(_obstacles[i].x, _obstacles[j].x, _obstacles[i].y, _obstacles[j].y);
				// If the distance between 2 obstacles are smaller than the size of robot, add the obstacle to to-be combined list
				if (obstacleDistance < (_obstacles[i].rad + _obstacles[j].rad))
				{
					combinedObstacles.push_back(_obstacles[j]);
					_obstacles[j].rad = 0;
				}
			}
		}
		Obstacle combinedObstacle;
		combinedObstacle.x = 0;
		combinedObstacle.y = 0;
		// Find Center Point of Close-Obstacles
		for (int j = 0; j < combinedObstacles.size(); j++) 
		{
			combinedObstacle.x += combinedObstacles[j].x;
			combinedObstacle.y += combinedObstacles[j].y;
		}
		combinedObstacle.x /= combinedObstacles.size();
		combinedObstacle.y /= combinedObstacles.size();

		// Find Radius of new Obstacle
		// New Radius = largest distance between new center with all associated obstacle centers plus obstacle radius
		combinedObstacle.rad = 0;
		for (int j = 0; j < combinedObstacles.size(); j++) 
		{
			if (combinedObstacle.rad < dist(combinedObstacles[j].x, combinedObstacle.x, combinedObstacles[j].y, combinedObstacle.y)) {
				combinedObstacle.rad = dist(combinedObstacles[j].x, combinedObstacle.x, combinedObstacles[j].y, combinedObstacle.y); 
			}
		}
		combinedObstacle.rad += combinedObstacles[0].rad;
		tempObstacles.push_back(combinedObstacle);
	}
	_obstacles = tempObstacles;
}

Coord2D CAlgorithm::getNewPointAroundObstacle(Obstacle obstacle, Coord2D beginPts, Coord2D endPts)
{
	Coord2D newPoint;
	// beginPts is the starting position
	// endPts is the ball position
	// TODO: use getTangentPointObstacle.
	// Logic:
	//		1. Find the Tangent points between BeginPts to Obstacle - Choose the point that is closest to EndPts
	//		2. Find the Tangent points between EndPts to Obstacle - Choose the point that is closest to BeginPts
	//		3. Construct a line-equation for 1. and 2., and find their intercept. Their intercept is the new point
	vector<Coord2D> tempBeginPoint;
	vector<Coord2D> tempEndPoint;
	tempBeginPoint = getTangentPointOfObstacle(obstacle, beginPts);
	tempEndPoint = getTangentPointOfObstacle(obstacle, endPts);

	// Step 1
	Coord2D tempNewPoint1; // Stores the Tangent Coord for begin point
	if ( dist(endPts.x, tempBeginPoint[0].x, endPts.y, tempBeginPoint[0].y) < dist(endPts.x, tempBeginPoint[1].x, endPts.y, tempBeginPoint[1].y) )
		tempNewPoint1 = tempBeginPoint[0];
	else
		tempNewPoint1 = tempBeginPoint[1];

	// Step 2
	Coord2D tempNewPoint2; // Stores the tangent Coord for end point
	if ( dist(beginPts.x, tempEndPoint[0].x, beginPts.y, tempEndPoint[0].y) < dist(beginPts.x, tempEndPoint[1].x, beginPts.y, tempEndPoint[1].y) )
		tempNewPoint2 = tempEndPoint[0];
	else
		tempNewPoint2 = tempEndPoint[1];
	
	// slope from the endPts to step 1
	double rise1, run1, slope1;
	rise1 = tempNewPoint1.y - beginPts.y;
	run1 = tempNewPoint1.x - beginPts.x;
	slope1 = rise1/run1;
	// slope from the beginPts to step 2
	double rise2, run2, slope2;
	rise2 = tempNewPoint2.y - endPts.y;
	run2 = tempNewPoint2.x - endPts.x;
	slope2 = rise2/run2;
	// equation of a line y = mx + b 
	double b1, b2;
	b1 = tempNewPoint1.y - (slope1*tempNewPoint1.x);
	b2 = tempNewPoint2.y - (slope2*tempNewPoint2.x);
	double newX, newY;
	// m1*x + b1 = m2*x + b2
	// b2 - b1 = x(m1 - m2)
	// x = (b2 - b1)/(m1 - m2)
	newX = (b2 - b1)/(slope1 - slope2);
	newY = slope1*newX + b1;
	newPoint.x = newX;
	newPoint.y = newY;
	return newPoint;
}

// getTangetPointOfObstacle - Returns the two points on the circle that is tangent to the point
vector<Coord2D> CAlgorithm::getTangentPointOfObstacle(Obstacle obstacle, Coord2D point)
{
	double lenBetweenObstaclePath = obstacle.rad*1.1;
	double lenBetweenPointObstacle = dist(point.x, obstacle.x, point.y, obstacle.y);
	double lenBetweenPointPath = sqrtf(pow(lenBetweenPointObstacle,2) - pow(lenBetweenObstaclePath,2));
	double angleObstaclePath = acos( (pow(lenBetweenPointPath,2) + pow(lenBetweenPointObstacle,2) - pow(lenBetweenObstaclePath,2))/(2*lenBetweenPointPath*lenBetweenPointObstacle));

	double xDiff = obstacle.x - point.x;
	double yDiff = obstacle.y - point.y;
	double angleObstacle = 0;
	if (abs(yDiff) < 0.01) {
		if (xDiff > 0) {
			angleObstacle = 0;
		} else {
			angleObstacle = PI;
		}
	} else if (abs(xDiff) < 0.01) {
		if (yDiff > 0) {
			angleObstacle = PI/2;
		} else {
			angleObstacle = 3*PI/2;
		}
	} else {
		angleObstacle = atan(abs(yDiff/xDiff));
		if (yDiff > 0 && xDiff < 0) { // 2nd Quadrant
			angleObstacle = PI - angleObstacle;
		}
		else if (yDiff < 0 && xDiff < 0) { // 3rd Quadrant
			angleObstacle = PI + angleObstacle;
		} 
		else if (yDiff < 0 && xDiff > 0) { // 4th Quadrant
			angleObstacle = 2*PI - angleObstacle;
		}
	}
	
	vector<Coord2D> newPoints;
	Coord2D newPoint;
	double rise = 0;
	double run = 0;

	rise = lenBetweenPointPath*sin(angleObstacle + angleObstaclePath);
	run = lenBetweenPointPath*cos(angleObstacle + angleObstaclePath);
	newPoint.x = run+point.x;
	newPoint.y = rise+point.y;
	newPoints.push_back(newPoint);

	rise = lenBetweenPointPath*sin(angleObstacle - angleObstaclePath);
	run = lenBetweenPointPath*cos(angleObstacle - angleObstaclePath);
	newPoint.x = run+point.x;
	newPoint.y = rise+point.y;
	newPoints.push_back(newPoint);

	return newPoints;
}

Coord2D CAlgorithm::calcForwardTicks(double dist)
{
	Coord2D tempTicks;
	double tempTick;
	tempTick = ((dist/100)/ONE_TICK);
	tempTick * 2.0;
	tempTicks.x = tempTick;
	tempTicks.y = tempTick;
	return tempTicks;
}

Coord2D CAlgorithm::calcTurnTicks(double angle, Coord2D cPt, Coord2D nPt)
{
	Coord2D tempTicks;
	double tempTick;
	tempTick = 2*PI*BOT_WIDTH;
	tempTick = tempTick*angle/360;
	tempTick = tempTick/ONE_TICK;
	tempTick = tempTick * 2.0;
	tempTick = abs(tempTick);
	if((angleRelative(nPt, cPt)*(180/PI)) > 0 && (angleRelative(nPt, cPt)*(180/PI)) < 180) {			
		// left turn
		tempTicks.x = tempTick;
		tempTicks.y = -tempTick;
	}
	else {
		// right turn
		tempTicks.x = -tempTick;
		tempTicks.y = tempTick;
	}
	return tempTicks;
}

/*
 * compares the total amount of ticks for each path
 */
void CAlgorithm::compareTicks()
{
	_ticks.clear();
	for(int i = 0; i < _paths.size(); i++) {
		_ticks.push_back(calculateTicks(_paths[i]));
	}
	if (_ticks.size() > 0) {
		double sum = 0;
		for (int i = 0; i < _ticks[0].size(); i++) {
			sum += abs(_ticks[0][i].x);
			_closest = 0;
		}

		for(int i = 1; i < _ticks.size(); i++) {
			double tempSum = 0;
			// calculate the total ticks of each path, then return the vector ticks that has the smallest ticks
			for(int j = 0; j < _ticks[i].size(); j++) {
					tempSum += abs(_ticks[i][j].x);
			}
			if(sum > tempSum) {
				// does a comparison of the previous smallest sum with the current sum
				sum = tempSum;
				_closest = i;
				_closestBall = _balls[_closest];
				// remembers the index of the vector here the path with the lowest amount of ticks are located
			}
		}
	}
}

/*
 * calculates the ticks of the path passed into the function
 */
vector<Coord2D> CAlgorithm::calculateTicks(vector<Coord2D> path)
{
	vector<Coord2D> ticks;
	Coord2D tempTicks;
	double opp = dist(path[0].x, path[1].x, path[0].y, path[1].y);
	double adj = dist(path[0].x, path[1].x, path[0].y, path[0].y);
	double firstAngle = angleWithOrigin(path[1]);
	firstAngle = (firstAngle - _robot.angle)*(180/PI);
	tempTicks = calcTurnTicks(firstAngle, path[0], path[1]);
	ticks.push_back(tempTicks);
	tempTicks = calcForwardTicks(dist(path[0].x, path[1].x, path[0].y, path[1].y));
	ticks.push_back(tempTicks);

	for(int i = 1; i < path.size()-1; i++) {
		double angle, a, b, c;
		a = dist(path[i].x,path[i-1].x,path[i].y,path[i-1].y);
		b = dist(path[i].x,path[i+1].x,path[i].y,path[i+1].y);
		c = dist(path[i-1].x,path[i+1].x,path[i-1].y,path[i+1].y);
		if(i < path.size()-1) {
			angle = cosineLaw(a, b, c);
			angle = 180 - angle;
			tempTicks = calcTurnTicks(angle, path[i], path[i+1]);
			ticks.push_back(tempTicks);
		}
		tempTicks = calcForwardTicks(a);
		ticks.push_back(tempTicks);
	}
	return ticks;
}