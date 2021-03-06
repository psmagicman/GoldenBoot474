#include "GLOBALS.h"

const int calibrationSize = 10;

const double _ballRadius = 0.11*FINAL_WIDTH/8;
const double _robotRadius = 0.4*FINAL_WIDTH/8;
const double _obstacleRadius = 0.35*FINAL_WIDTH/8;
const double _safetyRadius = _obstacleRadius + _robotRadius;

double dist(double x1, double x2, double y1, double y2)
{
	return sqrtf(
		pow(x1 - x2,2) +
		pow(y1 - y2,2)
		);
}

double distFromLine(Coord2D A, Coord2D B, Coord2D C)
{
	double dotProduct = (A.x-C.x)*(B.x-C.x) + (A.y-C.y)*(B.y-C.y);
	double lenA = dist(A.x, C.x, A.y, C.y);
	double lenB = dist(B.x, C.x, B.y, C.y);
	return sqrtf(lenA*lenA - abs(dotProduct/lenB)*abs(dotProduct/lenB)); // Scalar Projection
}

double angleWithOrigin(Coord2D A)
{
	if (abs(A.y) < 0.01) {
		if (A.x > 0) {
			return 0;
		} else {
			return PI;
		}
	} else if (abs(A.x) < 0.01) {
		if (A.y > 0) {
			return PI/2;
		} else {
			return 3*PI/2;
		}
	} else {
		double angle = atan(abs(A.y)/abs(A.x));
		if (A.y > 0 && A.x < 0) {
			angle = PI-angle;
		}
		else if (A.y < 0 && A.x < 0) {
			angle = PI+angle;
		}
		else if (A.y < 0 && A.x > 0) {
			angle = 2*PI-angle;
		}
		return angle;
	}
	return 0;
}

double angleRelative(Coord2D targetPt, Coord2D basePt)
{
	double xDiff = targetPt.x - basePt.x;
	double yDiff = targetPt.y - basePt.y;
	if (abs(yDiff) < 0.01) {
		if (xDiff > 0) {
			return 0;
		} else {
			return PI;
		}
	} else if (abs(xDiff) < 0.01) {
		if (yDiff > 0) {
			return PI/2;
		} else {
			return 3*PI/2;
		}
	} else {
		double angle = atan(abs(yDiff)/abs(xDiff));
		if (yDiff > 0 && xDiff < 0) {
			angle = PI-angle;
		}
		else if (yDiff < 0 && xDiff < 0) {
			angle = PI+angle;
		}
		else if (yDiff < 0 && xDiff > 0) {
			angle = 2*PI-angle;
		}
		return angle;
	}
	return 0;
}

double angleRelative2(Coord2D targetPt, Coord2D basePt)
{
	double xDiff = targetPt.x - basePt.x;
	double yDiff = targetPt.y - basePt.y;
	yDiff = - yDiff;
	if (abs(yDiff) < 0.01) {
		if (xDiff > 0) {
			return 0;
		} else {
			return PI;
		}
	} else if (abs(xDiff) < 0.01) {
		if (yDiff > 0) {
			return PI/2;
		} else {
			return 3*PI/2;
		}
	} else {
		double angle = atan(abs(yDiff)/abs(xDiff));
		if (yDiff > 0 && xDiff < 0) {
			angle = PI-angle;
		}
		else if (yDiff < 0 && xDiff < 0) {
			angle = PI+angle;
		}
		else if (yDiff < 0 && xDiff > 0) {
			angle = 2*PI-angle;
		}
		return angle;
	}
	return 0;
}

double cosineLaw(double lenA, double lenB, double lenC)
{
	return acos( (lenA*lenA + lenB*lenB - lenC*lenC)/(2*lenA*lenB) );
}

string itoa(double value)
{
	stringstream ss;
	ss << value;
	return ss.str();
}

string itoa(int value)
{
	stringstream ss;
	ss << value;
	return ss.str();
}