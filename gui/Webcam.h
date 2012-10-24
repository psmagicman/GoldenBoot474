#ifndef WEBCAM
#define WEBCAM

#include <opencv\cvaux.h>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>
#include <opencv2\opencv.hpp>

#include <vector>
#include <map>
#include <math.h>

#include "GLOBALS.h"

using namespace std;
using namespace cv;

class Webcam
{
public:
	Webcam(int ID);
	~Webcam();
	
	void calibrate(int index);
	void beginCalibrate();
	void finishCalibrate();
	bool isCalibrated() {return _calibrated;};
	void resetCalibrate();

	void calculateNormal(bool arena, bool balls, bool obstacles, bool robot, bool draw = false);
	void calculateThreshold(int type);

	bool capture();
	IplImage * getNormal() {return _normal;};
	IplImage * getHSV() {return _hsv;};
	IplImage * getThreshold() {return _threshold;};
	IplImage * getFinal();

	vector<Point2f> getBalls() {return _realBallPts;};
	vector<Point2f> getObstacles() {return _realObstaclesPts;};
	vector<Point2f> getRobots() {return _realRobotPts;};
	
	void release();

private:
	int _ID;
	CvSize _size;
	CvSize _finalSize;
	
	CvCapture * _capture;
	CvMemStorage * _storage;

	IplImage * _normal;
	IplImage * _hsv;
	IplImage * _threshold;
	IplImage * _final;

	IplImage * _threshold1;
	IplImage * _threshold2;

	CvScalar _arenaMin;
	CvScalar _arenaMax;
	CvScalar _ballMin;
	CvScalar _ballMax;
	CvScalar _obstaclesMin1;
	CvScalar _obstaclesMax1;
	CvScalar _obstaclesMin2;
	CvScalar _obstaclesMax2;
	CvScalar _robotMin1;
	CvScalar _robotMax1;
	CvScalar _robotMin2;
	CvScalar _robotMax2;

	Mat _homography;
	
	// Calibration Variables
	bool _calibrated;
	int * topLeftXarray;
	int * topLeftYarray;
	int * topRightXarray;
	int * topRightYarray;
	int * botLeftXarray;
	int * botLeftYarray;
	int * botRightXarray;
	int * botRightYarray;
	map<string, CvPoint> topPts;
	map<string, CvPoint> botPts;

	vector<CvPoint> _ballPts;
	vector<CvPoint> _obstaclesPts;
	vector<CvPoint> _robotPts;

	vector<Point2f> _realBallPts;
	vector<Point2f> _realObstaclesPts;
	vector<Point2f> _realRobotPts;

	void init();

	void QuickSort(int* array, int startIndex, int endIndex);
	int SplitArray(int* array, int pivotValue, int startIndex, int endIndex);
	void swap(int &a, int &b);
};

#endif