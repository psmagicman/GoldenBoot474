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
	
	void undistort();
	void calibrate(int index);
	void clearCalibrate();
	void finishCalibrate();
	bool isCalibrated();
	void reset();

	void calculateNormal(bool arena, bool balls, bool obstacles, bool robot);
	void calculateThreshold(int type);

	bool capture();
	IplImage * getNormal();
	IplImage * getHSV();
	IplImage * getThreshold();
	IplImage * getFinal();
	
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

	CvScalar _arenaMin;
	CvScalar _arenaMax;
	CvScalar _ballMin;
	CvScalar _ballMax;
	CvScalar _obstaclesMin;
	CvScalar _obstaclesMax;
	CvScalar _robotMin;
	CvScalar _robotMax;

	CvMat * _perspectiveMat;
	Mat _homography;
	CvPoint2D32f* _c1;
	CvPoint2D32f* _c2;
	
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

	double _topLen;
	double _Xratio;
	double _Yratio;
	double _Y_A;
	double _Y_B;
	double _Y_C;
	int _Xcenter;
	int _Ycenter;

	void init();
	int convertX(int X, int Y);
	int convertY(int Y);

	void QuickSort(int* array, int startIndex, int endIndex);
	int SplitArray(int* array, int pivotValue, int startIndex, int endIndex);
	void swap(int &a, int &b);
};

#endif