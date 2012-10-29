#ifndef GLOBAL_H_
#define GLOBAL_H_

struct Robot {
	int x;
	int y;
	double angle;
};

struct Ball {
	int x;
	int y;
	int rad;
};

struct Obstacle {
	int x;
	int y;
	double rad;
};

struct Coord2D {
	int x;
	int y;
};

#endif GLOBAL_H_