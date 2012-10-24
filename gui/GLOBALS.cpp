#include "GLOBALS.h"

const int _numOfWebcams = 2;
const int calibrationSize = 10;

const double _ballRadius = 0.2*FINAL_WIDTH/8;
const double _robotRadius = 0.25*FINAL_WIDTH/8;
const double _obstacleRadius = 0.4*FINAL_WIDTH/8;

// Arena
double arenaHmin = 0;
double arenaSmin = 0;
double arenaVmin = 0;

double arenaHmax = 256;
double arenaSmax = 256;
double arenaVmax = 75;

// Ball
double ballHmin = 20;
double ballSmin = 100;
double ballVmin = 100;

double ballHmax = 40;
double ballSmax = 255;
double ballVmax = 255;

// Obstacles - BLUE
double obstaclesHmin1 = 100;
double obstaclesSmin1 = 100;
double obstaclesVmin1 = 100;

double obstaclesHmax1 = 130;
double obstaclesSmax1 = 255;
double obstaclesVmax1 = 255;

// Obstacles - YELLOW
double obstaclesHmin2 = 20;
double obstaclesSmin2 = 100;
double obstaclesVmin2 = 100;

double obstaclesHmax2 = 30;
double obstaclesSmax2 = 255;
double obstaclesVmax2 = 255;

// Robot - RED
double robotHmin1 = 170;
double robotSmin1 = 150;
double robotVmin1 = 100;

double robotHmax1 = 180;
double robotSmax1 = 255;
double robotVmax1 = 255;

// Robot - GREEN
double robotHmin2 = 45;
double robotSmin2 = 100;
double robotVmin2 = 100;

double robotHmax2 = 90;
double robotSmax2 = 255;
double robotVmax2 = 255;