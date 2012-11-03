#ifndef __GLOBALS__
#define __GLOBALS__

#define WIDTH	640
#define HEIGHT	480
#define FINAL_WIDTH 400
#define FINAL_HEIGHT 400

#define FT_TO_CM 30.48

#define BAUD_RATE 57600

extern const int _numOfWebcams;
extern const int calibrationSize;

extern const double _ballRadius;
extern const double _robotRadius;
extern const double _obstacleRadius;

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
	int rad;
};

struct Obstacle {
	double x;
	double y;
	double rad;
};

#endif