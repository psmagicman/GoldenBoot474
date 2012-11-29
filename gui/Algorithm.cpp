#include "Algorithm.h"

CAlgorithm::CAlgorithm()
{
	_closest = -1;
}

CAlgorithm::CAlgorithm(vector<Obstacle> obstacles)
{
	_originalObstacles = obstacles;
	for (int i = 0; i < _originalObstacles.size(); i++) {
		_originalObstacles[i].rad = _safetyRadius;
	}
	_originalObstacles.resize(_originalObstacles.size()+1);
	_originalObstacles[_originalObstacles.size()-1].rad = 0;
	analyzeObstacles();
	_closest = -1;
}

vector<Coord2D> CAlgorithm::getPathToGoal(Robot robot, Coord2D goal)
{
	_robot = robot;	
	if (goal.y > FINAL_HEIGHT/2) {
		goal.y -= 40;
		goal.x -= 15;
	} else {
		goal.y += 40;
		goal.x += 15;
	}

	calcSafety();
	vector<Coord2D> path = getPathToPoint(goal,0);

	if (goal.y > FINAL_HEIGHT/2) {
		goal.y += 10;
	} else {
		goal.y -= 10;
	}
	path.push_back(goal);
	return path;
}

void CAlgorithm::setOpponent(Obstacle opponent)
{
	opponent.rad = _robotRadius*2;
	_originalObstacles[_originalObstacles.size()-1] = opponent;
}

void CAlgorithm::analyzeField(Robot robot, vector<Ball> balls)
{
	_closest = -1;
	_robot = robot;
	_balls = balls;
	_paths.clear();

	calcSafety();

	// Remove Balls that are outside of "Dangerous Fields"
	for (int i = 0; i < _balls.size(); i++) {
		for (int j = 0; j < _obstacles.size(); j++) {
			if ( dist(_balls[i].x, _obstacles[j].x, _balls[i].y, _obstacles[j].y) <= _obstacles[j].rad) {
				_balls.erase(_balls.begin()+i);
				i--;
				break;
			}
		}
		if (i < _balls.size()) {
			int distanceFromWall = 15;
			if (_balls[i].x < distanceFromWall || _balls[i].x > FINAL_WIDTH - distanceFromWall || _balls[i].y < distanceFromWall || _balls[i].y > FINAL_HEIGHT - distanceFromWall) {
				_balls.erase(_balls.begin()+i);
				i--;
			}
		}
	}
	// Cycle through all the Valid Balls, and construct a Path to each ball
	for (int iBall = 0; iBall < _balls.size(); iBall++) {
		Coord2D ballPoint;
		ballPoint.x = _balls[iBall].x;
		ballPoint.y = _balls[iBall].y;
		_paths.push_back(getPathToPoint(ballPoint,35));
	}
}

vector<Coord2D> CAlgorithm::getPathToPoint(Coord2D point, double distance)
{
	Coord2D beginPts; // Beginning Coordinates of a path
	Coord2D endPts; // Ending Coordinates of a path

	beginPts.x = _robot.x;
	beginPts.y = _robot.y;
	endPts.x = point.x;
	endPts.y = point.y;

	vector<Coord2D> path;
	path.push_back(beginPts);
	if (_safetyFlag) {
		path.push_back(_safetyCoord);
	}
	path.push_back(endPts);

	// If there are obstacles in the field, detect if there are any obstacles in the way
	if (_obstacles.size() > 0) {
		// Loop through the path (Look at pop_back and push_back at the bottom. If there are obstacles detected along the way, then a newPath will be pushed
		int iPath = 0;
		if (_safetyFlag) iPath = 1;
		for (; iPath < path.size()-1; iPath++) {
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
						dist(beginPts.x, endPts.x, beginPts.y, endPts.y) > dist(beginPts.x, _obstacles[iObstacle].x, beginPts.y, _obstacles[iObstacle].y) &&
						dist(obstaclePts.x, beginPts.x, obstaclePts.y, beginPts.y) > dist(_obstacles[iObstacle].x, beginPts.x, _obstacles[iObstacle].y, beginPts.y) // Check if the newer obstacle is closer
					) 
				{
					obstaclePts = _obstacles[iObstacle]; // Closest Obstacle and it's length to the path is less than its radius
				}
			}
			// If an obstacle is detected to be in-between the path, add a new point to the path
			if (obstaclePts.x >= 0 && obstaclePts.y >= 0) {
				Coord2D newPath;
				if (_safetyFlag) 
					newPath = getNewPointAroundObstacle(obstaclePts,path[1],endPts);
				else 
					newPath = getNewPointAroundObstacle(obstaclePts,beginPts,endPts);
				path[path.size()-1] = newPath;
				path.push_back(endPts);
			}
		}
	}

	double angleBeforeLastPoint = angleRelative(path[path.size()-1], path[path.size()-2]);
	double lastLength = dist(path[path.size()-1].x, path[path.size()-2].x, path[path.size()-1].y, path[path.size()-2].y);
	if (lastLength > distance) {
		endPts.x = endPts.x - distance*cos(angleBeforeLastPoint);
		endPts.y = endPts.y - distance*sin(angleBeforeLastPoint);
		path[path.size()-1] = endPts;
	} else {
		endPts.x = endPts.x - (lastLength-1)*cos(angleBeforeLastPoint);
		endPts.y = endPts.y - (lastLength-1)*cos(angleBeforeLastPoint);
		path[path.size()-1] = endPts;
	}
	return path;
}

void CAlgorithm::analyzeObstacles()
{
	vector<Obstacle> tempObstacles;
	_obstacles = _originalObstacles;
	for (int i = 0; i < _obstacles.size(); i++) 
	{
		if (_obstacles[i].rad > 0) {
			vector<Obstacle> combinedObstacles;
			combinedObstacles.push_back(_obstacles[i]);
			// Combine Obstacles that does not have enough gap in-between for the robot to fit through
			for (int j = 0; j < _obstacles.size(); j++) 
			{
				if (i != j && _obstacles[j].rad > 0) {
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
	if ( dist(tempNewPoint1.x, tempEndPoint[0].x, tempNewPoint1.y, tempEndPoint[0].y) < dist(tempNewPoint1.x, tempEndPoint[1].x, tempNewPoint1.y, tempEndPoint[1].y) )
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

void CAlgorithm::calcSafety()
{
	int obstacleIndex = -1;
	for (int i = 0; i < _obstacles.size(); i++) {
		if (dist(_robot.x, _obstacles[i].x, _robot.y, _obstacles[i].y) < (_obstacles[i].rad + _robotRadius))
		{
			obstacleIndex = i;
			break;
		}
	}

	if (obstacleIndex > -1) {
		double safetyRadius = _obstacles[obstacleIndex].rad;
		double slope = abs((_robot.y - _obstacles[obstacleIndex].y) / (_robot.x - _obstacles[obstacleIndex].x));
		if (_robot.x < _obstacles[obstacleIndex].x ) _safetyCoord.x = _obstacles[obstacleIndex].x-safetyRadius*1.2*cos(slope);
		else if (abs(_robot.x - _obstacles[obstacleIndex].x) < 5) _safetyCoord.x = _obstacles[obstacleIndex].x;
		else _safetyCoord.x = _obstacles[obstacleIndex].x+safetyRadius*1.2*cos(slope);
		
		if (_robot.y < _obstacles[obstacleIndex].y ) _safetyCoord.y = _obstacles[obstacleIndex].y-safetyRadius*1.2*sin(slope);
		else if (abs(_robot.y - _obstacles[obstacleIndex].y) < 5) _safetyCoord.y = _obstacles[obstacleIndex].y;
		else _safetyCoord.y = _obstacles[obstacleIndex].y+safetyRadius*1.2*sin(slope);
		_safetyFlag = true;
	} else {
		_safetyFlag = false;
	}
}