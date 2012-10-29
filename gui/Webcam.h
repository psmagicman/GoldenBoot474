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
#include "QuickSort.h"

using namespace std;
using namespace cv;

class Webcam
{
public:
	Webcam(int ID, bool home);
	~Webcam();
	
	bool capture();

	void calibrate(int index);
	void finishCalibrate();

	bool isCalibrated() {return _calibrated;};
	void resetCalibrate();

	void setThresholds(map<string, vector<int> > thresholds);
	void calculateNormal(bool arena, bool balls, bool obstacles, bool robot, bool draw = false);
	void calculateThreshold(string type);
	void calculateFinal();

	void changeThreshold(string type, int input[8]);

	IplImage * getNormal() {return _normal;};
	IplImage * getHSV() {return _hsv;};
	IplImage * getThreshold() {return _threshold;};
	IplImage * getFinal();

	vector<Point2f> getBalls() {return _ballPts;};
	vector<Point2f> getRobot() {return _robotPts;};
	vector<Point2f> getObstacles() {return _obstaclesPts;};
	double getRobotAngle() {return _robotAngle;};
	
	void release();

	// Arena
	int _arenaHmin;
	int _arenaSmin;
	int _arenaVmin;
	int _arenaHmax;
	int _arenaSmax;
	int _arenaVmax;
	int _arenaAmin;
	int _arenaAmax;
	// Ball
	int _ballHmin;
	int _ballSmin;
	int _ballVmin;
	int _ballHmax;
	int _ballSmax;
	int _ballVmax;
	int _ballAmin;
	int _ballAmax;
	// Obstacles - YELLOW
	int _obstacles1Hmin;
	int _obstacles1Smin;
	int _obstacles1Vmin;
	int _obstacles1Hmax;
	int _obstacles1Smax;
	int _obstacles1Vmax;
	int _obstacles1Amin;
	int _obstacles1Amax;
	// Obstacles - Blue
	int _obstacles2Hmin;
	int _obstacles2Smin;
	int _obstacles2Vmin;
	int _obstacles2Hmax;
	int _obstacles2Smax;
	int _obstacles2Vmax;
	int _obstacles2Amin;
	int _obstacles2Amax;
	// Robot - FRONT (Green)
	int _robot1Hmin;
	int _robot1Smin;
	int _robot1Vmin;
	int _robot1Hmax;
	int _robot1Smax;
	int _robot1Vmax;
	int _robot1Amin;
	int _robot1Amax;
	// Robot - BACK (RED)
	int _robot2Hmin;
	int _robot2Smin;
	int _robot2Vmin;
	int _robot2Hmax;
	int _robot2Smax;
	int _robot2Vmax;
	int _robot2Amin;
	int _robot2Amax;

private:
	int _ID;
	bool _main; // Is this the main camera?
	CvSize _size; // Size of Capture Image
	CvSize _finalSize; // Size of Processed (Final) Image

	CvCapture * _capture;
	CvMemStorage * _storage;

	IplImage * _normal;
	IplImage * _hsv;
	IplImage * _threshold;
	IplImage * _final;

	QuickSort myQuickSort;

	int _robotDist; // Used to threshold the distance between the two robot colours

	CvScalar _thresholdMin;
	CvScalar _thresholdMax;

	Mat _homography;
	
	// Calibration Variables
	bool _calibrated;
	int * _topLeftXarray;
	int * _topLeftYarray;
	int * _topRightXarray;
	int * _topRightYarray;
	int * _botLeftXarray;
	int * _botLeftYarray;
	int * _botRightXarray;
	int * _botRightYarray;
	map<string, Point2f> _topArenaPts;
	map<string, Point2f> _botArenaPts;

	vector<Point2f> _ballPts;
	vector<Point2f> _obstaclesPts;
	vector<Point2f> _robotPts;
	vector<Point2f> _robotFrontPts;
	vector<Point2f> _robotBackPts;
	double _robotAngle;

	void init();
	double dist(double x1, double x2, double y1, double y2);
};

#endif