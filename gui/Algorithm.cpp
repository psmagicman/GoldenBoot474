#include "Algorithm.h"

Algorithm::Algorithm()
{
}

Algorithm::Algorithm(vector<Obstacle> obstacles)
{
	_obstacles = obstacles;
	for (int i = 0; i < _obstacles.size(); i++) {
		_obstacles[i].rad = _obstacleRadius*2;
	}
	analyzeObstacles();
}

/*
Coord2D Algorithm::getClosestBall()
{
}

vector<Coord2D> Algorithm::getPathToClosestBall()
{
}

vector<Coord2D> Algorithm::getPathToGoal()
{
}
*/

void Algorithm::analyzeField(Robot robot, vector<Ball> balls)
{
	_robot = robot;
	_balls = balls;
	_paths.clear();
	
	Coord2D beginPts; // Beginning Coordinates of a path
	Coord2D endPts; // Ending Coordinates of a path

	// Set Beginning Coordinates to Robot
	beginPts.x = _robot.x;
	beginPts.y = _robot.y;

	// Cycle through all the Balls, and construct a Path to each ball
	for (int iBall = 0; iBall < _balls.size(); iBall++) {
		vector<Coord2D> path;
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
					if ( cosineLaw(
							dist(beginPts.x, endPts.x, beginPts.y, endPts.y), 
							dist(beginPts.x, tempObstaclePts.x, beginPts.y, tempObstaclePts.y),
							dist(endPts.x, tempObstaclePts.x, endPts.y, tempObstaclePts.y)
							) < PI/2 && // Check if the Angle between the Obstacle and the Path is less than 90 degrees. If it is, that means it can intercept with the path
							distFromLine(tempObstaclePts, endPts, beginPts) < _obstacles[iObstacle].rad && // Check if the distance of the Obstacle to the line is less than the radius of the Obstacle
							dist(obstaclePts.x, beginPts.x, obstaclePts.y, beginPts.y) > dist(_obstacles[iObstacle].x, beginPts.x, _obstacles[iObstacle].y, beginPts.y) // Check if the newer obstacle is closer
						) 
					{
						obstaclePts = _obstacles[iObstacle]; // Closest Obstacle and it's length to the path is less than its radius
					}
				}
				// If an obstacle is detected to be in-between the path, add a new point to the path
				if (obstaclePts.x > 0 && obstaclePts.y > 0) {
					Coord2D newPath = getNewPointAroundObstacle(obstaclePts,beginPts,endPts);
					path[path.size()-1] = newPath;
					path.push_back(endPts);
				}
			}
		}
		_paths.push_back(path);
	}
}

void Algorithm::analyzeObstacles()
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

Coord2D Algorithm::getNewPointAroundObstacle(Obstacle obstacle, Coord2D beginPts, Coord2D endPts)
{
	Coord2D newPoint;
	// TODO: use getTangentPointObstacle.
	// Logic:
	//		1. Find the Tangent points between BeginPts to Obstacle - Choose the point that is closest to EndPts
	//		2. Find the Tangent points between EndPts to Obstacle - Choose the point that is closest to BeginPts
	//		3. Construct a line-equation for 1. and 2., and find their intercept. Their intercept is the new point
	return newPoint;
}

// getTangetPointOfObstacle - Returns the two points on the circle that is tangent to the point
vector<Coord2D> Algorithm::getTangentPointOfObstacle(Obstacle obstacle, Coord2D point)
{
	double lenBetweenObstaclePath = obstacle.rad;
	double lenBetweenPointObstacle = dist(point.x, obstacle.x, point.y, obstacle.y);
	double lenBetweenPointPath = sqrtf(pow(lenBetweenPointObstacle,2) - pow(lenBetweenObstaclePath,2));
	double angleObstaclePath = acos( (pow(lenBetweenPointPath,2) + pow(lenBetweenPointObstacle,2) - pow(lenBetweenObstaclePath,2))/(2*lenBetweenPointPath*lenBetweenPointObstacle));

	double xDiff = obstacle.x - point.x;
	double yDiff = obstacle.y - point.y;
	double angleObstacle = 0;
	if (yDiff < 0.01) {
		if (xDiff > 0) {
			angleObstacle = 0;
		} else {
			angleObstacle = PI;
		}
	} else if (xDiff < 0.01) {
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