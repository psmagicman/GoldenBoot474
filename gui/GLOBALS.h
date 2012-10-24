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

// Arena
extern double arenaHmin;
extern double arenaSmin;
extern double arenaVmin;

extern double arenaHmax;
extern double arenaSmax;
extern double arenaVmax;

// Ball
extern double ballHmin;
extern double ballSmin;
extern double ballVmin;

extern double ballHmax;
extern double ballSmax;
extern double ballVmax;

// Obstacles - BLUE
extern double obstaclesHmin1;
extern double obstaclesSmin1;
extern double obstaclesVmin1;

extern double obstaclesHmax1;
extern double obstaclesSmax1;
extern double obstaclesVmax1;

// Obstacles - YELLOW
extern double obstaclesHmin2;
extern double obstaclesSmin2;
extern double obstaclesVmin2;

extern double obstaclesHmax2;
extern double obstaclesSmax2;
extern double obstaclesVmax2;

// Robot - RED
extern double robotHmin1;
extern double robotSmin1;
extern double robotVmin1;

extern double robotHmax1;
extern double robotSmax1;
extern double robotVmax1;

// Robot - GREEN
extern double robotHmin2;
extern double robotSmin2;
extern double robotVmin2;

extern double robotHmax2;
extern double robotSmax2;
extern double robotVmax2;

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
	int rad;
};

#endif