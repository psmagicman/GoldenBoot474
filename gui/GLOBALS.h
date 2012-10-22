#ifndef __GLOBALS__
#define __GLOBALS__

#define WIDTH	640
#define HEIGHT	480
#define FINAL_WIDTH 400
#define FINAL_HEIGHT 400

const int _numOfWebcams = 2;
const int calibrationSize = 100;

// Arena
const double arenaHmin = 0;
const double arenaSmin = 0;
const double arenaVmin = 0;

const double arenaHmax = 256;
const double arenaSmax = 256;
const double arenaVmax = 75;

// Ball
const double ballHmin = 32;
const double ballSmin = 75;
const double ballVmin = 75;

const double ballHmax = 40;
const double ballSmax = 255;
const double ballVmax = 255;

// Obstacles
const double obstaclesHmin = 32;
const double obstaclesSmin = 75;
const double obstaclesVmin = 75;

const double obstaclesHmax = 40;
const double obstaclesSmax = 255;
const double obstaclesVmax = 255;

// Robot
const double robotHmin = 32;
const double robotSmin = 75;
const double robotVmin = 75;

const double robotHmax = 40;
const double robotSmax = 255;
const double robotVmax = 255;

#endif