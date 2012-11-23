#ifndef __GLOBALS__
#define __GLOBALS__

#include <math.h>
#include <string>
#include <sstream>

using namespace std;

#define WIDTH	640
#define HEIGHT	480
#define FINAL_WIDTH 400
#define FINAL_HEIGHT 400

#define PI	3.14159265359

#define FT_TO_CM 30.48

#define BAUD_RATE 57600

#define BOT_WIDTH 0.172244
#define ONE_TICK 0.0113

#define EMERGENCY_ACT	-2
#define	EMERGENCY_STOP	-1
#define TASKS_READY		0
#define STP1_REQUEST	1
#define STP1_RESPOND	2
#define BALL_REQUEST	3
#define	BALL_RESPOND	4
#define STP2_REQUEST	5
#define STP2_RESPOND	6
#define GOAL_REQUEST	7
#define GOAL_RESPOND	8

#define NO_ERRORS		0
#define	ERR_PATHING		1
#define ERR_OBSTACLE	2
#define ERR_BALL		3

extern const int calibrationSize;

extern const double _ballRadius;
extern const double _robotRadius;
extern const double _obstacleRadius;
extern const double _safetyRadius;

struct Coord2D {
	double x;
	double y;
};

struct Robot {
	double x;
	double y;
	double angle;
};

struct Ball {
	double x;
	double y;
};

struct Obstacle {
	double x;
	double y;
	double rad;
};

extern double dist(double x1, double x2, double y1, double y2);
extern double distFromLine(Coord2D A, Coord2D B, Coord2D C);
extern double angleWithOrigin(Coord2D A);
extern double angleRelative(Coord2D targetPt, Coord2D basePt);
extern double angleRelative2(Coord2D targetPt, Coord2D basePt);
extern double cosineLaw(double lenA, double lenB, double lenC);
extern string itoa(double value);
extern string itoa(int value);

#endif