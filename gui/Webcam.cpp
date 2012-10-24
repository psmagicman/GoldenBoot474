#include "Webcam.h"

Webcam::Webcam(int ID)
{
	_ID = ID;
	init();
}

Webcam::~Webcam()
{
	release();
	cvReleaseCapture(&_capture);
}

void Webcam::init()
{
	_size = cvSize(WIDTH, HEIGHT);
	_finalSize = cvSize(FINAL_WIDTH, FINAL_HEIGHT); // 8ft x 8ft
	_hsv = cvCreateImage(_size, IPL_DEPTH_8U, 3);
	_threshold = cvCreateImage(_size, IPL_DEPTH_8U, 1);
	_final = cvCreateImage(_finalSize, IPL_DEPTH_8U, 3);
	_threshold1 = cvCreateImage(_size, IPL_DEPTH_8U, 1);
	_threshold2 = cvCreateImage(_size, IPL_DEPTH_8U, 1);

	_capture = cvCaptureFromCAM(_ID);
	
	// Create Threshold Limits 
	_ballMin = cvScalar(ballHmin, ballSmin, ballVmin);
	_ballMax = cvScalar(ballHmax, ballSmax, ballSmax);
	_arenaMin = cvScalar(arenaHmin, arenaSmin, arenaVmin);
	_arenaMax = cvScalar(arenaHmax, arenaSmax, arenaVmax);
	_obstaclesMin1 = cvScalar(obstaclesHmin1, obstaclesSmin1, obstaclesVmin1);
	_obstaclesMax1 = cvScalar(obstaclesHmax1, obstaclesSmax1, obstaclesVmax1);
	_obstaclesMin2 = cvScalar(obstaclesHmin2, obstaclesSmin2, obstaclesVmin2);
	_obstaclesMax2 = cvScalar(obstaclesHmax2, obstaclesSmax2, obstaclesVmax2);
	_robotMin1 = cvScalar(robotHmin1, robotSmin1, robotVmin1);
	_robotMax1 = cvScalar(robotHmax1, robotSmax1, robotVmax1);
	_robotMin2 = cvScalar(robotHmin2, robotSmin2, robotVmin2);
	_robotMax2 = cvScalar(robotHmax2, robotSmax2, robotVmax2);

	_calibrated = false;
}

bool Webcam::capture()
{
	//_normal = cvQueryFrame(_capture);
	_normal = cvLoadImage("test.jpg", CV_LOAD_IMAGE_COLOR);
	if (_normal) cvCvtColor(_normal, _hsv, CV_BGR2HSV);
	if (_normal && _hsv) {
		_storage = cvCreateMemStorage(0);
		return true;
	} else {
		return false;
	}
	return false;
}

void Webcam::release()
{
	cvReleaseMemStorage(&_storage);
}

void Webcam::resetCalibrate()
{
	_calibrated = false;
}

void Webcam::calibrate(int index)
{
	if (capture()) {
		// Detect Arena
		calculateThreshold(0);
	
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
		
		CvSeq * lines = cvHoughLines2(_threshold, _storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 100, 200, 0);
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
			else if (line[0].y > _threshold->height/2 && line[1].y > _threshold->height/2) {
				if (botPts[0].y < line[0].y || botPts[1].y < line[1].y) {
					if (botPts[0].x > line[0].x) {
						botPts[0].x = line[0].x;
						botPts[0].y = line[0].y;
					} 
					if (botPts[1].x < line[1].x) {
						botPts[1].x = line[1].x;
						botPts[1].y = line[1].y;
					}
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

void Webcam::beginCalibrate()
{
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

	// Take Median Values to avoid Errors
	topPts["Left"].x = topLeftXarray[calibrationSize/2]; 
	topPts["Left"].y = topLeftYarray[calibrationSize/2];
	topPts["Right"].x = topRightXarray[calibrationSize/2];
	topPts["Right"].y = topRightYarray[calibrationSize/2];
	
	botPts["Left"].x = botLeftXarray[calibrationSize/2];
	botPts["Left"].y = botLeftYarray[calibrationSize/2];
	botPts["Right"].x = botRightXarray[calibrationSize/2];
	botPts["Right"].y = botRightYarray[calibrationSize/2];

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
	
	// Find Homography
	_homography = findHomography(cameraPts, fieldPts);
	
	delete [] topLeftXarray;
	delete [] topLeftYarray;
	delete [] topRightXarray;
	delete [] topRightYarray;

	delete [] botLeftXarray;
	delete [] botLeftYarray;
	delete [] botRightXarray;
	delete [] botRightYarray;

	_calibrated = true;
}

void Webcam::calculateNormal(bool arena, bool balls, bool obstacles, bool robot, bool draw)
{
	if (arena) {
		if (_calibrated) {
			cvLine(_normal, topPts["Left"], topPts["Right"], CV_RGB(0,0,255), 1, CV_AA, 0);
			cvLine(_normal, botPts["Left"], botPts["Right"], CV_RGB(0,0,255), 1, CV_AA, 0);
		} else {
			calculateThreshold(0);
			CvSeq * lines = cvHoughLines2(_threshold, _storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 100, 200, 0);
			for (int i = 0; i < lines->total; i++) {
				CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
				if (draw) cvLine(_normal, line[0], line[1], CV_RGB(0,0,255), 1, CV_AA, 0);
			}
		}
	}

	if (balls) {
		calculateThreshold(1);
		CvSeq * contours;
		cvFindContours(_threshold, _storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
		_ballPts.clear();
		for (;contours != 0; contours = contours->h_next)
		{
			if (cvContourArea(contours) > 200 && cvContourArea(contours) < 1200) {
				CvMoments moment;
				cvMoments(contours, &moment, 0);
				double m_00 = cvGetSpatialMoment( &moment, 0, 0);
				double m_10 = cvGetSpatialMoment( &moment, 1, 0);
				double m_01 = cvGetSpatialMoment( &moment, 0, 1);
				_ballPts.push_back(cvPoint(m_10/m_00, m_01/m_00));
				if (draw) {
					cvCircle(_normal, _ballPts.back(), 1, CV_RGB(0,255,0));
					cvDrawContours(_normal, contours, CV_RGB(0,255,0), CV_RGB(0,255,0), -1, 1, 8, cvPoint(0,0));
				}
			}
		}
	}

	if (obstacles) {
		calculateThreshold(2);
		CvSeq * contours;
		cvFindContours(_threshold, _storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
		_obstaclesPts.clear();
		for (;contours != 0; contours = contours->h_next)
		{
			if (cvContourArea(contours) > 2000) {
				if (draw) cvDrawContours(_normal, contours, CV_RGB(255,0,0), CV_RGB(255,0,0), -1, 1, 8, cvPoint(0,0));			
				CvMoments moment;
				cvMoments(contours, &moment, 0);
				double m_00 = cvGetSpatialMoment( &moment, 0, 0);
				double m_10 = cvGetSpatialMoment( &moment, 1, 0);
				double m_01 = cvGetSpatialMoment( &moment, 0, 1);
				_obstaclesPts.push_back(cvPoint(m_10/m_00, m_01/m_00+sqrtf(cvContourArea(contours)/3)));
			}
		}
	}

	if (robot) {
		calculateThreshold(3);
		CvSeq * contours;
		cvFindContours(_threshold, _storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
		_robotPts.clear();
		vector<double> contourCenterX;
		vector<double> contourCenterY;
		for (;contours != 0; contours = contours->h_next)
		{
			if (cvContourArea(contours) > 300) {
				if (draw) cvDrawContours(_normal, contours, CV_RGB(255,0,0), CV_RGB(255,0,0), -1, 1, 8, cvPoint(0,0));
				CvMoments moment;
				cvMoments(contours, &moment, 0);
				double m_00 = cvGetSpatialMoment( &moment, 0, 0);
				double m_10 = cvGetSpatialMoment( &moment, 1, 0);
				double m_01 = cvGetSpatialMoment( &moment, 0, 1);
				contourCenterX.push_back(m_10/m_00);
				contourCenterY.push_back(m_01/m_00);
			}
		}
		if (contourCenterX.size() > 1) {
			for (int i = 0; i < contourCenterX.size(); i++) {
				for (int j = 0; j < contourCenterX.size(); j++) {
					double dist = sqrtf(pow(contourCenterX[i] - contourCenterX[j],2) + pow(contourCenterY[i] - contourCenterY[j],2));
					if ( i != j && dist < 1000) {
						_robotPts.push_back(cvPoint((contourCenterX[i]+contourCenterX[j])/2, (contourCenterY[i]+contourCenterY[j])/2));
						if (draw) cvCircle(_normal, _robotPts.back(), 1, CV_RGB(255,0,0));
					}
				}
			}
		}
	}
}

void Webcam::calculateThreshold(int type)
{
	if (type == 0) { // Arena
		cvInRangeS(_hsv, _arenaMin, _arenaMax, _threshold);
	} else if (type == 1) { // Balls
		cvInRangeS(_hsv, _ballMin, _ballMax, _threshold);		
	} else if (type == 2) { // Obstacles
		cvInRangeS(_hsv, _obstaclesMin1, _obstaclesMax1, _threshold1);
		cvInRangeS(_hsv, _obstaclesMin2, _obstaclesMax2, _threshold2);
		cvOr(_threshold1, _threshold2, _threshold);
	} else if (type == 3) { // Robot
		cvInRangeS(_hsv, _robotMin1, _robotMax1, _threshold1);
		cvInRangeS(_hsv, _robotMin2, _robotMax2, _threshold2);
		cvOr(_threshold1, _threshold2, _threshold);
	} else { // No Threshold
		cvInRangeS(_hsv, cvScalar(255,255,255), cvScalar(255,255,255), _threshold);
	}
	cvSmooth(_threshold, _threshold, CV_GAUSSIAN, 5, 5);
}

IplImage * Webcam::getFinal()
{
	cvZero(_final);
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
	
	// If Calibrated, Draw Objects
	if (_calibrated) {
		calculateNormal(1,1,1,1);

		_realBallPts.clear();
		vector<Point2f> camBallPts;	
		for (int i = 0; i < _ballPts.size(); i++) {
			camBallPts.push_back(cvPoint(_ballPts[i].x, _ballPts[i].y));
			_realBallPts.push_back(cvPoint(0, 0));
		}
		perspectiveTransform((Mat)camBallPts, (Mat)_realBallPts, _homography);
		for (int i = 0; i < _realBallPts.size(); i++) {
			cvCircle(_final, _realBallPts[i], _ballRadius, CV_RGB(0,255,0), 2, 8, 0);
		}
		
		_realRobotPts.clear();
		vector<Point2f> camRobotPts;
		for (int i = 0; i < _robotPts.size(); i++) {
			camRobotPts.push_back(_robotPts[i]);
			_realRobotPts.push_back(cvPoint(0,0));
		}
		perspectiveTransform((Mat)camRobotPts, (Mat)_realRobotPts, _homography);
		for (int i = 0; i < _realRobotPts.size(); i++) {
			cvCircle(_final, _realRobotPts[i], _robotRadius, CV_RGB(0,0,255), 2, 8, 0);
		}

		_realObstaclesPts.clear();
		vector<Point2f> camObstaclesPts;
		for (int i = 0; i < _obstaclesPts.size(); i++) {
			camObstaclesPts.push_back(_obstaclesPts[i]);
			_realObstaclesPts.push_back(cvPoint(0,0));
		}
		perspectiveTransform((Mat)camObstaclesPts, (Mat)_realObstaclesPts, _homography);
		for (int i = 0; i < _realObstaclesPts.size(); i++) {
			cvCircle(_final, _realObstaclesPts[i], _obstacleRadius, CV_RGB(255,0,0), 2, 8, 0);
		}
	}

	return _final;
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