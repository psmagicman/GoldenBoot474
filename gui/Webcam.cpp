#include "Webcam.h"

Webcam::Webcam(int ID)
{
	_ID = ID;
	init();
}

Webcam::~Webcam()
{
	cvReleaseCapture(&_capture);
}

void Webcam::init()
{
	_size = cvSize(WIDTH, HEIGHT);
	_finalSize = cvSize(FINAL_WIDTH, FINAL_HEIGHT); // 8ft x 8ft

	_hsv = cvCreateImage(_size, IPL_DEPTH_8U, 3);
	_threshold = cvCreateImage(_size, IPL_DEPTH_8U, 1);
	_final = cvCreateImage(_finalSize, IPL_DEPTH_8U, 3);

	//_capture = cvCaptureFromCAM(_ID);
	_normal = cvLoadImage("test.jpg", CV_LOAD_IMAGE_COLOR);

	_ballMin = cvScalar(ballHmin, ballSmin, ballVmin);
	_ballMax = cvScalar(ballHmax, ballSmax, ballSmax);

	_arenaMin = cvScalar(arenaHmin, arenaSmin, arenaVmin);
	_arenaMax = cvScalar(arenaHmax, arenaSmax, arenaVmax);

	_obstaclesMin = cvScalar(obstaclesHmin, obstaclesSmin, obstaclesVmin);
	_obstaclesMax = cvScalar(obstaclesHmax, obstaclesSmax, obstaclesVmax);

	_robotMin = cvScalar(robotHmin, robotSmin, robotVmin);
	_robotMax = cvScalar(robotHmax, robotSmax, robotVmax);

	topLeftXarray = new int[calibrationSize];
	topLeftYarray = new int[calibrationSize];
	topRightXarray = new int[calibrationSize];
	topRightYarray = new int[calibrationSize];

	botLeftXarray = new int[calibrationSize];
	botLeftYarray = new int[calibrationSize];
	botRightXarray = new int[calibrationSize];
	botRightYarray = new int[calibrationSize];

	topPts["Left"].x = 0;
	topPts["Left"].y = 0;
	topPts["Right"].x = 0;
	topPts["Right"].y = 0;
	
	botPts["Left"].x = 0;
	botPts["Left"].y = 0;
	botPts["Right"].x = 0;
	botPts["Right"].y = 0;
	
	_perspectiveMat = cvCreateMat(3,3,CV_32FC1);

	_calibrated = false;
}

bool Webcam::capture()
{
	//_normal = cvQueryFrame(_capture);
	cvCvtColor(_normal, _hsv, CV_BGR2HSV);
	_storage = cvCreateMemStorage(0);
	if (_normal && _hsv)
		return true;
	else
		return false;
	return true;
}

void Webcam::release()
{
	cvReleaseMemStorage(&_storage);
}

void Webcam::reset()
{
	_calibrated = false;
}

void Webcam::undistort()
{
}

void Webcam::calibrate(int index)
{
	if (capture()) {
		calculateThreshold(0);
		// Detect Lines
		CvSeq * lines = cvHoughLines2(_threshold, _storage, CV_HOUGH_PROBABILISTIC, 1, 90*CV_PI/180, 100, 200, 0);
	
		CvPoint * topPts = new CvPoint[2];
		topPts[0].x = _normal->width;
		topPts[0].y = _normal->height;
		topPts[1].x = 0;
		topPts[1].y = _normal->height;

		CvPoint * botPts = new CvPoint[2];
		botPts[0].x = _normal->width;
		botPts[0].y = 0;
		botPts[1].x = 0;
		botPts[1].y = 0;

		for (int i = 0; i < lines->total; i++) {
			CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
			if (line[0].y < _threshold->height/2 && line[1].y < _threshold->height/2) {
				if (topPts[0].x > line[0].x) {
					topPts[0].x = line[0].x;
					topPts[0].y = line[0].y;
				}
				if (topPts[1].x < line[1].x) {
					topPts[1].x = line[1].x;
					topPts[1].y = line[1].y;
				}
			}
		}

		botPts[0] = topPts[1];
		botPts[1] = topPts[0];

		for (int i = 0; i < lines->total; i++) {
			CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
			if (line[0].y > _threshold->height/2 && line[1].y > _threshold->height/2) {
				if (botPts[0].y < line[0].y || botPts[1].y < line[1].y) {
					if ((line[0].x > topPts[0].x+10) && (botPts[0].x > line[0].x)) botPts[0].x = line[0].x;
					if (botPts[0].y < line[0].y) botPts[0].y = line[0].y;
					if ((line[1].x < topPts[1].x-10) && (botPts[1].x < line[1].x)) botPts[1].x = line[1].x;
					if (botPts[1].y < line[1].y) botPts[1].y = line[1].y;
				
				}
			}
		}
		
		topLeftXarray[index] = topPts[0].x;
		topLeftYarray[index] = topPts[0].y;
		topRightXarray[index] = topPts[1].x;
		topRightYarray[index] = topPts[1].y;

		botLeftXarray[index] = botPts[0].x;
		botLeftYarray[index] = botPts[0].y;
		botRightXarray[index] = botPts[1].x;
		botRightYarray[index] = botPts[1].y;

		release();
	}
}

void Webcam::clearCalibrate()
{
	delete [] topLeftXarray;
	delete [] topLeftYarray;
	delete [] topRightXarray;
	delete [] topRightYarray;

	delete [] botLeftXarray;
	delete [] botLeftYarray;
	delete [] botRightXarray;
	delete [] botRightYarray;

	topLeftXarray = new int[calibrationSize];
	topLeftYarray = new int[calibrationSize];
	topRightXarray = new int[calibrationSize];
	topRightYarray = new int[calibrationSize];

	botLeftXarray = new int[calibrationSize];
	botLeftYarray = new int[calibrationSize];
	botRightXarray = new int[calibrationSize];
	botRightYarray = new int[calibrationSize];
}

void Webcam::finishCalibrate()
{
	QuickSort(topLeftXarray, 0 , calibrationSize-1);
	QuickSort(topLeftYarray, 0 , calibrationSize-1);
	QuickSort(topRightXarray, 0 , calibrationSize-1);
	QuickSort(topRightYarray, 0 , calibrationSize-1);
	
	QuickSort(botLeftXarray, 0 , calibrationSize-1);
	QuickSort(botLeftYarray, 0 , calibrationSize-1);
	QuickSort(botRightXarray, 0 , calibrationSize-1);
	QuickSort(botRightYarray, 0 , calibrationSize-1);

	topPts["Left"].x = topLeftXarray[calibrationSize/2];
	topPts["Left"].y = topLeftYarray[calibrationSize/2];
	topPts["Right"].x = topRightXarray[calibrationSize/2];
	topPts["Right"].y = topRightYarray[calibrationSize/2];
	
	botPts["Left"].x = botLeftXarray[calibrationSize/2];
	botPts["Left"].y = botLeftYarray[calibrationSize/2];
	botPts["Right"].x = botRightXarray[calibrationSize/2];
	botPts["Right"].y = botRightYarray[calibrationSize/2];

	_topLen = abs(topPts["Right"].x - topPts["Left"].x);
	double midLen = abs((double)(topPts["Right"].y+topPts["Left"].y)/2.0 - (double)(botPts["Right"].y+botPts["Right"].y)/2.0);
	double midLen2 = _topLen * 7.0 / 8.0;

	_Yratio = (midLen2 / midLen) * (FINAL_HEIGHT / _topLen);
	_Xratio = FINAL_WIDTH / _topLen;
	
	vector<Point2f> cameraPts;
	vector<Point2f> fieldPts;

	cameraPts.push_back(cvPoint(topPts["Left"].x, topPts["Left"].y));
	cameraPts.push_back(cvPoint(topPts["Right"].x, topPts["Right"].y));
	cameraPts.push_back(cvPoint(botPts["Left"].x, botPts["Left"].y));
	cameraPts.push_back(cvPoint(botPts["Right"].x, botPts["Right"].y));

	fieldPts.push_back(cvPoint(0,0));
	fieldPts.push_back(cvPoint(FINAL_WIDTH,0));
	fieldPts.push_back(cvPoint(FINAL_WIDTH*2.5/8.0,FINAL_HEIGHT*7.0/8.0));
	fieldPts.push_back(cvPoint(FINAL_WIDTH*5.5/8.0,FINAL_HEIGHT*7.0/8.0));

	_homography = findHomography(cameraPts, fieldPts);

	_calibrated = true;
}

bool Webcam::isCalibrated()
{
	return _calibrated;
}

void Webcam::calculateNormal(bool arena, bool balls, bool obstacles, bool robot)
{
	if (arena) {
		cvLine(_normal, topPts["Left"], topPts["Right"], CV_RGB(0,0,255), 1, CV_AA, 0);
		cvLine(_normal, botPts["Left"], botPts["Right"], CV_RGB(0,0,255), 1, CV_AA, 0);
	}

	if (balls) {
		calculateThreshold(1);
		CvSeq* circles = cvHoughCircles(_threshold, _storage, CV_HOUGH_GRADIENT, 5, 50, 200, 30, 1,10);
		_ballPts.clear();
		_ballPts.resize(circles->total);
		for( int i = 0; i < _ballPts.size(); i++) {
			float * p = (float*)cvGetSeqElem(circles,i);
			cvCircle(_normal, cvPoint(cvRound(p[0]), cvRound(p[1])), 3, CV_RGB(0,255,0), -1, 8, 0);
			cvCircle(_normal, cvPoint(cvRound(p[0]), cvRound(p[1])), cvRound(p[2]), CV_RGB(0,255,0), 2, 8, 0);
			_ballPts[i].x = p[0];
			_ballPts[i].y = p[1];
		}
	}

	if (obstacles) {
		calculateThreshold(2);
	}

	if (robot) {
		calculateThreshold(3);
	}
}

void Webcam::calculateThreshold(int type)
{
	if (type == 0) { // Arena
		cvInRangeS(_hsv, _arenaMin, _arenaMax, _threshold);
	} else if (type == 1) { // Balls
		cvInRangeS(_hsv, _ballMin, _ballMax, _threshold);
	} else if (type == 2) { // Obstacles
		cvInRangeS(_hsv, _obstaclesMin, _obstaclesMax, _threshold);
	} else if (type == 3) { // Robot
		cvInRangeS(_hsv, _robotMin, _robotMax, _threshold);
	} else { // No Threshold
		cvInRangeS(_hsv, cvScalar(255,255,255), cvScalar(255,255,255), _threshold);
	}
	cvSmooth(_threshold, _threshold, CV_GAUSSIAN, 7, 7);
}

IplImage *  Webcam::getNormal()
{
	return _normal;
}

IplImage *	Webcam::getHSV()
{
	return _hsv;
}

IplImage * Webcam::getThreshold()
{
	return _threshold;
}

IplImage * Webcam::getFinal()
{
	CvPoint * corners = new CvPoint[4];
	corners[0].x = 0;
	corners[0].y = 0;
	corners[1].x = _final->width;
	corners[1].y = 0;
	corners[2].x = _final->width;
	corners[2].y = _final->height;
	corners[3].x = 0;
	corners[3].y = _final->height;
	for (int i = 0 ; i < 3; i++) {
		cvLine(_final, corners[i], corners[i+1], CV_RGB(0,0,255), 3, CV_AA, 0);
	}
	cvLine(_final, corners[3], corners[0], CV_RGB(0,0,255), 3, CV_AA, 0);

	CvPoint * topGoal = new CvPoint[4];
	topGoal[0].x = _final->width*2.5/8.0;
	topGoal[0].y = 0;
	topGoal[1].x = _final->width*2.5/8.0;
	topGoal[1].y = _final->height / 8;
	topGoal[2].x = _final->width*5.5/8.0;
	topGoal[2].y = _final->height / 8;
	topGoal[3].x = _final->width*5.5/8.0;
	topGoal[3].y = 0;
	for (int i = 0; i < 3; i++) {
		cvLine(_final, topGoal[i], topGoal[i+1], CV_RGB(0,0,255), 3, CV_AA, 0);
	}

	CvPoint * bottomGoal = new CvPoint[4];
	bottomGoal[0].x = _final->width*5.5/8.0;
	bottomGoal[0].y = _final->height;
	bottomGoal[1].x = _final->width*5.5/8.0;
	bottomGoal[1].y = _final->height*7/8;
	bottomGoal[2].x = _final->width*2.5/8.0;
	bottomGoal[2].y = _final->height*7/8;
	bottomGoal[3].x = _final->width*2.5/8.0;
	bottomGoal[3].y = _final->height;
	for (int i = 0; i < 3; i++) {
		cvLine(_final, bottomGoal[i], bottomGoal[i+1], CV_RGB(0,0,255), 3, CV_AA, 0);
	}
	
	calculateNormal(0,1,0,0);
	
	if (_calibrated) {
		vector<Point2f> camBallPts;
		vector<Point2f> realBallPts;
	
		for (int i = 0; i < _ballPts.size(); i++) {
			camBallPts.push_back(cvPoint(_ballPts[i].x, _ballPts[i].y));
			realBallPts.push_back(cvPoint(0, 0));
		}

		perspectiveTransform((Mat)camBallPts, (Mat)realBallPts, _homography);

		for (int i = 0; i < realBallPts.size(); i++) {
			cvCircle(_final, realBallPts[i], 5, CV_RGB(0,255,0), 2, 8, 0);
		}
	}

	return _final;
}

int Webcam::convertX(int X, int Y)
{
	return 0;
}

int Webcam::convertY(int Y)
{
	double correctDistFromTop = abs(Y-(topPts["Right"].y + topPts["Left"].y)/2);
	return correctDistFromTop*_Yratio;
}

void Webcam::swap(int &a, int &b)
{
	int temp;
	temp = a;
	a = b;
	b = temp;
}

int Webcam::SplitArray(int* array, int pivot, int startIndex, int endIndex)
{
	int leftBoundary = startIndex;
	int rightBoundary = endIndex;
	
	while(leftBoundary < rightBoundary)			   
	{
		 while( pivot < array[rightBoundary]		
				&& rightBoundary > leftBoundary)	
		 {
			  rightBoundary--;						
		 }
		 swap(array[leftBoundary], array[rightBoundary]);
		 
		 while( pivot >= array[leftBoundary]		  
				&& leftBoundary < rightBoundary)	  
		 {
			  leftBoundary++;						
		 }
		 swap(array[rightBoundary], array[leftBoundary]);
	}
	return leftBoundary;							  
}

void Webcam::QuickSort(int* array, int startIndex, int endIndex)
{
	int pivot = array[startIndex];					
	int splitPoint;
	
	if(endIndex > startIndex)						
	{
		splitPoint = SplitArray(array, pivot, startIndex, endIndex);												
		array[splitPoint] = pivot;
		QuickSort(array, startIndex, splitPoint-1); 
		QuickSort(array, splitPoint+1, endIndex);	
	}
}