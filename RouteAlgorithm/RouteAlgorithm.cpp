#include <opencv\cvaux.h>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>
#include <opencv2\opencv.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include "GLOBALS.h"
#include "Coord2D.h"

using namespace std;

Coord2D robot;
vector<Coord2D> balls;
vector<Coord2D> obstacles;

double dist (Coord2D A, Coord2D B)
{
	return sqrtf(
		(A.x - B.x)*(A.x - B.x) +
		(A.y - B.y)*(A.y - B.y)
		);
}

double distFromRobot(Coord2D object)
{
return sqrtf(
		(robot.x - object.x)*(robot.x - object.x) +
		(robot.y - object.y)*(robot.y - object.y)
		);
}

void swap(vector<Coord2D> &array, int a, int b)
{
	Coord2D temp = array[a];
	array[a] = array[b];
	array[b] = temp;
}

int SplitArray(vector<Coord2D> &array, Coord2D pivot, int startIndex, int endIndex)
{
	int leftBoundary = startIndex;
	int rightBoundary = endIndex;

	while (leftBoundary < rightBoundary)
	{
		while (
			distFromRobot(pivot) < distFromRobot(array[rightBoundary]) &&
			rightBoundary > leftBoundary
			)
		{
			rightBoundary--;
		}
		swap(array, leftBoundary, rightBoundary);
		
		while (
			distFromRobot(pivot) >= distFromRobot(array[leftBoundary]) &&
			leftBoundary < rightBoundary
			)
		{
			leftBoundary--;
		}
		swap(array, rightBoundary, leftBoundary);
	}
	return leftBoundary;
}

void QuickSort(vector<Coord2D> &array, int startIndex, int endIndex)
{
	Coord2D pivot = array[startIndex];
	int splitPoint;

	if (endIndex > startIndex) 
	{
		splitPoint = SplitArray(array, pivot, startIndex, endIndex);

		array[splitPoint] = pivot;

		QuickSort(array, startIndex, splitPoint-1);
		QuickSort(array, splitPoint+1, endIndex);
	}
}

void sortBalls()
{
	QuickSort(balls,0,balls.size()-1);
	for (int i = 0; i < balls.size(); i++) {
		cout << distFromRobot(balls[i]) << endl;
	}
}

void main()
{
	cout << "Robot X: ";
	cin >> robot.x;
	cout << "Robot Y: ";
	cin >> robot.y;

	int ballSize;
	cout << "Number of Balls: ";
	cin >> ballSize;
	balls.resize(ballSize);
	for( int i = 0; i < balls.size(); i++) {
		cout << "Ball#" << i << " X:";
		cin >> balls[i].x;
		cout << "Ball#" << i << " Y:";
		cin >> balls[i].y;
	}

	int obstacleSize;
	cout << "Number of Obstacles: ";
	cin >> obstacleSize;
	obstacles.resize(obstacleSize);
	for( int i = 0; i < obstacles.size(); i++) {
		cout << "Obstacle#" << i << " X:";
		cin >> obstacles[i].x;
		cout << "Obstacle#" << i << " Y:";
		cin >> obstacles[i].y;
	}

	sortBalls();

	CvSize size = cvSize(400,400);
	IplImage * image = cvCreateImage(size, IPL_DEPTH_8U, 3);

	cvCircle(image, cvPoint(robot.x, robot.y), _robotRadius, CV_RGB(0,0,255));
	for (int i = 0; i < balls.size(); i++) {
		cvCircle(image, cvPoint(balls[i].x, balls[i].y), _ballRadius, CV_RGB(0,255,0));
	}
	for (int i = 0; i < obstacles.size(); i++) {
		cvCircle(image, cvPoint(obstacles[i].x, obstacles[i].y), _obstacleRadius, CV_RGB(255,0,0));
	}

	cvNamedWindow("Weee", CV_WINDOW_AUTOSIZE);
	cvShowImage("Weee",image);

	while (true) {
		if (cvWaitKey(10) == 27) {
			break;
		}
	}
	cvDestroyAllWindows();
}