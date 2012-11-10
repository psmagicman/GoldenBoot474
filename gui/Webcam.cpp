#include "Webcam.h"

Webcam::Webcam(int ID)
{
	_ID = ID;
	init();
}

Webcam::~Webcam()
{
	if (capture()) {
		release();
		cvReleaseCapture(&_capture);
	}
}

void Webcam::init()
{
	_size = cvSize(WIDTH, HEIGHT);
	_finalSize = cvSize(FINAL_WIDTH, FINAL_HEIGHT); // 8ft x 8ft
	_hsv = cvCreateImage(_size, IPL_DEPTH_8U, 3);
	_threshold = cvCreateImage(_size, IPL_DEPTH_8U, 1);
	_final = cvCreateImage(_finalSize, IPL_DEPTH_8U, 3);

	_capture = cvCaptureFromCAM(_ID);
	
	// _arena
	_arenaHmin = 0;
	_arenaSmin = 0;
	_arenaVmin = 0;
	_arenaHmax = 255;
	_arenaSmax = 255;
	_arenaVmax = 80;
	_arenaAmin = 200;
	_arenaAmax = 0;
	// _ball
	_ballHmin = 30;
	_ballSmin = 150;
	_ballVmin = 50;
	_ballHmax = 40;
	_ballSmax = 255;
	_ballVmax = 255;
	_ballAmin = 200;
	_ballAmax = 1250;
	// _obstacles - YELLOW
	_obstacles1Hmin = 20;
	_obstacles1Smin = 100;
	_obstacles1Vmin = 100;
	_obstacles1Hmax = 40;
	_obstacles1Smax = 255;
	_obstacles1Vmax = 255;
	_obstacles1Amin = 1000;
	_obstacles1Amax = 10000;
	// _obstacles - BLUE
	_obstacles2Hmin = 100;
	_obstacles2Smin = 175;
	_obstacles2Vmin = 100;
	_obstacles2Hmax = 120;
	_obstacles2Smax = 255;
	_obstacles2Vmax = 255;
	_obstacles2Amin = 200;
	_obstacles2Amax = 4000;
	// _robot - GREEN
	_robot1Hmin = 60;
	_robot1Smin = 100;
	_robot1Vmin = 100;
	_robot1Hmax = 80;
	_robot1Smax = 255;
	_robot1Vmax = 255;
	_robot1Amin = 100;
	_robot1Amax = 1000;
	// _robot - RED
	_robot2Hmin = 170;
	_robot2Smin = 150;
	_robot2Vmin = 150;
	_robot2Hmax = 180;
	_robot2Smax = 255;
	_robot2Vmax = 255;
	_robot2Amin = 100;
	_robot2Amax = 1000;
	_robotDist = 15;
	
	_topLeftXarray = new int[calibrationSize];
	_topLeftYarray = new int[calibrationSize];
	_topRightXarray = new int[calibrationSize];
	_topRightYarray = new int[calibrationSize];
	_botLeftXarray = new int[calibrationSize];
	_botLeftYarray = new int[calibrationSize];
	_botRightXarray = new int[calibrationSize];
	_botRightYarray = new int[calibrationSize];

	_calibrated = false;
	_calibratedObstacles = false;
}

bool Webcam::capture()
{
	_normal = cvQueryFrame(_capture);
	//_normal = cvLoadImage("test.jpg", CV_LOAD_IMAGE_COLOR);
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
	_calibratedObstacles = false;
}

void Webcam::calibrate(int index)
{
	if (capture()) {
		// Detect Arena
		calculateThreshold("Arena");
	
		_topLeftXarray[index] = _normal->width;	_topLeftYarray[index] = _normal->height;
		_topRightXarray[index] = 0;				_topRightYarray[index] = _normal->height;
		_botLeftXarray[index] = _normal->width;	_botLeftYarray[index] = 0;
		_botRightXarray[index] = 0;				_botRightYarray[index] = 0;

		CvSeq * lines = cvHoughLines2(_threshold, _storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 100, _arenaAmin, _arenaAmax);
		for (int i = 0; i < lines->total; i++) {
			CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
			// If Line is top half of image, assign left-most to topLeft, and right-most to topRight
			if (line[0].y < _threshold->height/2 && line[1].y < _threshold->height/2) {
				if (_topLeftXarray[index] > line[0].x) {
					_topLeftXarray[index] = line[0].x;
					_topLeftYarray[index] = line[0].y;
				}
				if (_topRightXarray[index] < line[1].x) {
					_topRightXarray[index] = line[1].x;
					_topRightYarray[index] = line[1].y;
				}
			}
			// If line is bottom half of image, assign left-most to botLeft, and right-most to botRight
			else if (line[0].y > _threshold->height/2 && line[1].y > _threshold->height/2) {
				if (_botLeftXarray[index] > line[0].x) {
					_botLeftXarray[index] = line[0].x;
					_botLeftYarray[index] = line[0].y;
				} 
				if (_botRightXarray[index] < line[1].x) {
					_botRightXarray[index] = line[1].x;
					_botRightYarray[index] = line[1].y;
				}
			}
		}
		release();
	}
}

void Webcam::finishCalibrate()
{
	// Sort all Values through Quicksort
	myQuickSort.sort(_topLeftXarray, 0 , calibrationSize-1);
	myQuickSort.sort(_topLeftYarray, 0 , calibrationSize-1);
	myQuickSort.sort(_topRightXarray, 0 , calibrationSize-1);
	myQuickSort.sort(_topRightYarray, 0 , calibrationSize-1);
	myQuickSort.sort(_botLeftXarray, 0 , calibrationSize-1);
	myQuickSort.sort(_botLeftYarray, 0 , calibrationSize-1);
	myQuickSort.sort(_botRightXarray, 0 , calibrationSize-1);
	myQuickSort.sort(_botRightYarray, 0 , calibrationSize-1);

	// Take Median Values to eliminate Errors
	_topArenaPts["Left"].x = _topLeftXarray[calibrationSize/2]; 
	_topArenaPts["Left"].y = _topLeftYarray[calibrationSize/2];
	_topArenaPts["Right"].x = _topRightXarray[calibrationSize/2];
	_topArenaPts["Right"].y = _topRightYarray[calibrationSize/2];
	_botArenaPts["Left"].x = _botLeftXarray[calibrationSize/2];
	_botArenaPts["Left"].y = _botLeftYarray[calibrationSize/2];
	_botArenaPts["Right"].x = _botRightXarray[calibrationSize/2];
	_botArenaPts["Right"].y = _botRightYarray[calibrationSize/2];

	// Create Homography Matrix
	vector<Point2f> cameraPts;
	vector<Point2f> fieldPts;
	cameraPts.push_back(_topArenaPts["Left"]);
	cameraPts.push_back(_topArenaPts["Right"]);
	cameraPts.push_back(_botArenaPts["Left"]);
	cameraPts.push_back(_botArenaPts["Right"]);
	fieldPts.push_back(cvPoint(0,0));
	fieldPts.push_back(cvPoint(FINAL_WIDTH,0));
	fieldPts.push_back(cvPoint(FINAL_WIDTH*2.5/8.0,FINAL_HEIGHT*7.0/8.0));
	fieldPts.push_back(cvPoint(FINAL_WIDTH*5.5/8.0,FINAL_HEIGHT*7.0/8.0));
	_homography = findHomography(cameraPts, fieldPts);

	// Calibration COMPLETE
	_calibrated = true;
}

void Webcam::calculateObstacles()
{
	if (_calibrated && capture()) {
		calculateThreshold("Obstacles1");
		vector<Point2f> obstaclesBotPts;
		cvFindContours(_threshold, _storage, &_obstaclesBotContour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
		for (;_obstaclesBotContour != 0; _obstaclesBotContour = _obstaclesBotContour->h_next)
		{
			if (cvContourArea(_obstaclesBotContour) > _obstacles1Amin && cvContourArea(_obstaclesBotContour) < _obstacles1Amax) {
				CvMoments moment;
				cvMoments(_obstaclesBotContour, &moment, 0);
				double m_00 = cvGetSpatialMoment( &moment, 0, 0);
				double m_10 = cvGetSpatialMoment( &moment, 1, 0);
				double m_01 = cvGetSpatialMoment( &moment, 0, 1);
				obstaclesBotPts.push_back(cvPoint(m_10/m_00, m_01/m_00));
			}
		}

		calculateThreshold("Obstacles2");
		vector<Point2f> obstaclesTopPts;
		vector<double> obstaclesTopHeight;
		cvFindContours(_threshold, _storage, &_obstaclesTopContour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
		for (;_obstaclesTopContour != 0; _obstaclesTopContour = _obstaclesTopContour->h_next)
		{
			if (cvContourArea(_obstaclesTopContour) > _obstacles2Amin && cvContourArea(_obstaclesTopContour) < _obstacles2Amax) {
				CvMoments moment;
				cvMoments(_obstaclesTopContour, &moment, 0);
				double m_00 = cvGetSpatialMoment( &moment, 0, 0);
				double m_10 = cvGetSpatialMoment( &moment, 1, 0);
				double m_01 = cvGetSpatialMoment( &moment, 0, 1);
				obstaclesTopPts.push_back(cvPoint(m_10/m_00, m_01/m_00));
				obstaclesTopHeight.push_back(sqrtf(cvContourArea(_obstaclesTopContour)/2));
			}
		}

		_obstaclesPtsCam.clear();
		for (int i = 0; i < obstaclesTopPts.size(); i++) {
			for (int j = 0; j < obstaclesBotPts.size(); j++) {
				if (
					abs(obstaclesTopPts[i].y - obstaclesBotPts[j].y) < 100	&&
					abs(obstaclesTopPts[i].x - obstaclesBotPts[j].x) < 50 
					) 
				{
					if (dist(obstaclesTopPts[i].x , obstaclesBotPts[j].x, obstaclesTopPts[i].y, obstaclesBotPts[j].y) < 75) {
						_obstaclesPtsCam.push_back(Point2f(
							2*obstaclesBotPts[j].x - obstaclesTopPts[i].x,
							2*obstaclesBotPts[j].y - obstaclesTopPts[i].y - obstaclesTopHeight[i]
							));
					}
				}
			}
		}
		_obstaclesPts = _obstaclesPtsCam;
		perspectiveTransform((Mat)_obstaclesPtsCam, (Mat)_obstaclesPts, _homography);

		_calibratedObstacles = true;
	}
}

void Webcam::calculateNormal(bool arena, bool balls, bool obstacles, bool robot, bool draw)
{
	if (arena) {
		if (_calibrated) {
			cvLine(_normal, _topArenaPts["Left"], _topArenaPts["Right"], CV_RGB(0,0,255), 1, CV_AA, 0);
			cvLine(_normal, _botArenaPts["Left"], _botArenaPts["Right"], CV_RGB(0,0,255), 1, CV_AA, 0);
		} else {
			calculateThreshold("Arena");
			CvSeq * lines = cvHoughLines2(_threshold, _storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 100, _arenaAmin, _arenaAmax);
			if (draw) {
				for (int i = 0; i < lines->total; i++) {
					CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
					cvLine(_normal, line[0], line[1], CV_RGB(0,0,255), 1, CV_AA, 0);
				}
			}
		}
	}

	if (balls) {
		calculateThreshold("Balls");
		CvSeq * contours;
		cvFindContours(_threshold, _storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
		_ballPts.clear();
		for (;contours != 0; contours = contours->h_next)
		{
			if (cvContourArea(contours) > _ballAmin && cvContourArea(contours) < _ballAmax) {
				CvMoments moment;
				cvMoments(contours, &moment, 0);
				double m_00 = cvGetSpatialMoment( &moment, 0, 0);
				double m_10 = cvGetSpatialMoment( &moment, 1, 0);
				double m_01 = cvGetSpatialMoment( &moment, 0, 1);
				_ballPts.push_back(cvPoint(m_10/m_00, m_01/m_00 + sqrtf(cvContourArea(contours)/(double)CV_PI)/2.0 ));
				if (draw) {
					cvCircle(_normal, _ballPts.back(), 1, CV_RGB(0,255,0));
					cvDrawContours(_normal, contours, CV_RGB(0,255,0), CV_RGB(0,255,0), -1, 1, 8, cvPoint(0,0));
				}
			}
		}
		delete contours;
	}

	if (obstacles) {
		// Detect Obstacles
		if (!_calibratedObstacles) {
			calculateThreshold("Obstacles1");
			CvSeq * contoursBot;
			cvFindContours(_threshold, _storage, &contoursBot, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
			for (;contoursBot != 0; contoursBot = contoursBot->h_next)
			{
				if (cvContourArea(contoursBot) > _obstacles1Amin && cvContourArea(contoursBot) < _obstacles1Amax) {
					if (draw) cvDrawContours(_normal, contoursBot, CV_RGB(255,0,0), CV_RGB(255,0,0), -1, 1, 8, cvPoint(0,0));
				}
			}
			delete contoursBot;

			calculateThreshold("Obstacles2");
			CvSeq * contoursTop;
			cvFindContours(_threshold, _storage, &contoursTop, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
			for (;contoursTop != 0; contoursTop = contoursTop->h_next)
			{
				if (cvContourArea(contoursTop) > _obstacles2Amin && cvContourArea(contoursTop) < _obstacles2Amax) {
					if (draw) cvDrawContours(_normal, contoursTop, CV_RGB(128,0,0), CV_RGB(255,0,0), -1, 1, 8, cvPoint(0,0));
				}
			}
			delete contoursTop;
		}
		else {
			if (draw) {
				for (int i = 0; i < _obstaclesPtsCam.size(); i++) {
					cvDrawCircle(_normal, _obstaclesPtsCam[i], 1, CV_RGB(255,0,0));
				}
			}
		}
	}

	if (robot) { // Detect Robot only if this is the main camera
		// Find Front of Robot
		calculateThreshold("Robot1");
		CvSeq * contoursFront;
		cvFindContours(_threshold, _storage, &contoursFront, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
		_robotFrontPts.clear();
		for (;contoursFront != 0; contoursFront = contoursFront->h_next)
		{
			if (cvContourArea(contoursFront) > _robot1Amin && cvContourArea(contoursFront) < _robot1Amax) {
				if (draw) cvDrawContours(_normal, contoursFront, CV_RGB(0,0,0), CV_RGB(0,0,0), -1, 1, 8, cvPoint(0,0));
				CvMoments moment;
				cvMoments(contoursFront, &moment, 0);
				double m_00 = cvGetSpatialMoment( &moment, 0, 0);
				double m_10 = cvGetSpatialMoment( &moment, 1, 0);
				double m_01 = cvGetSpatialMoment( &moment, 0, 1);
				_robotFrontPts.push_back(cvPoint(m_10/m_00,(m_01/m_00+20)));
			}
		}
		delete contoursFront;

		// Find Back of Robot
		calculateThreshold("Robot2");
		CvSeq * contoursBack;
		cvFindContours(_threshold, _storage, &contoursBack, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
		_robotBackPts.clear();
		for (;contoursBack != 0; contoursBack = contoursBack->h_next)
		{
			if (cvContourArea(contoursBack) > _robot2Amin && cvContourArea(contoursBack) < _robot2Amax) {
				if (draw) cvDrawContours(_normal, contoursBack, CV_RGB(128,128,128), CV_RGB(128,128,128), -1, 1, 8, cvPoint(0,0));
				CvMoments moment;
				cvMoments(contoursBack, &moment, 0);
				double m_00 = cvGetSpatialMoment( &moment, 0, 0);
				double m_10 = cvGetSpatialMoment( &moment, 1, 0);
				double m_01 = cvGetSpatialMoment( &moment, 0, 1);
				_robotBackPts.push_back(cvPoint(m_10/m_00,(m_01/m_00+20)));
			}
		}
		delete contoursBack;
	}
}

void Webcam::calculateThreshold(string type)
{
	if (type == "Arena") { // Arena
		_thresholdMin = cvScalar(_arenaHmin, _arenaSmin, _arenaVmin);
		_thresholdMax = cvScalar(_arenaHmax, _arenaSmax, _arenaVmax);
	} else if (type == "Balls") { // Balls;		
		_thresholdMin = cvScalar(_ballHmin, _ballSmin, _ballVmin);
		_thresholdMax = cvScalar(_ballHmax, _ballSmax, _ballVmax);
	} else if (type == "Obstacles1") { // Obstacles - Yellow
		_thresholdMin = cvScalar(_obstacles1Hmin, _obstacles1Smin, _obstacles1Vmin);
		_thresholdMax = cvScalar(_obstacles1Hmax, _obstacles1Smax, _obstacles1Vmax);
	} else if (type == "Obstacles2") { // Obstacles - Blue
		_thresholdMin = cvScalar(_obstacles2Hmin, _obstacles2Smin, _obstacles2Vmin);
		_thresholdMax = cvScalar(_obstacles2Hmax, _obstacles2Smax, _obstacles2Vmax);
	} else if (type == "Robot1") { // Robot - FRONT
		_thresholdMin = cvScalar(_robot1Hmin, _robot1Smin, _robot1Vmin);
		_thresholdMax = cvScalar(_robot1Hmax, _robot1Smax, _robot1Vmax);
	} else if (type == "Robot2") { // Robot - BACK
		_thresholdMin = cvScalar(_robot2Hmin, _robot2Smin, _robot2Vmin);
		_thresholdMax = cvScalar(_robot2Hmax, _robot2Smax, _robot2Vmax);
	} else { // No Threshold
		_thresholdMin = cvScalar(0,0,0);
		_thresholdMax = cvScalar(0,0,0);
	}
	cvInRangeS(_hsv, _thresholdMin, _thresholdMax, _threshold);
	cvSmooth(_threshold, _threshold, CV_GAUSSIAN, 5, 5);
}

void Webcam::calculateFinal()
{
	// If Calibrated, Calculate Objects
	if (_calibrated) {
		calculateNormal(0,1,1,1);
		
		// Transform the Balls to plane
		if (_ballPts.size() > 0) perspectiveTransform((Mat)_ballPts, (Mat)_ballPts, _homography);

		// Transform the Robot to Plane, and find middle point + angle
		_robotPts.clear();
		if (_robotFrontPts.size() > 0 && _robotBackPts.size() > 0) {
			perspectiveTransform((Mat)_robotFrontPts, (Mat)_robotFrontPts, _homography);
			perspectiveTransform((Mat)_robotBackPts, (Mat)_robotBackPts, _homography);
			for (int i = 0; i < _robotFrontPts.size(); i++) {
				for (int j = 0; j < _robotBackPts.size(); j++) {
					double distTemp = dist(_robotFrontPts[i].x, _robotBackPts[j].x, _robotFrontPts[i].y, _robotBackPts[j].y);
					if (distTemp < _robotDist) {
						_robotPts.push_back(_robotFrontPts[i]);
						_robotPts.push_back(_robotBackPts[j]);
						for (int k = 0; k < _robotPts.size(); k++) {
							_robotPts[k].y = _robotPts[k].y + distTemp;
						}
						if (abs(_robotPts[0].x - _robotPts[1].x) < 0.5) {
							if (_robotPts[0].y < _robotPts[1].y) // Facing Upwards
								_robotAngle = CV_PI/2;
							else // Facing Downwards
								_robotAngle = CV_PI*3/2;
						}
						else if (abs(_robotPts[0].y - _robotPts[1].y) < 0.5) {
							if (_robotPts[0].x > _robotPts[1].x)
								_robotAngle = 0;
							else
								_robotAngle = CV_PI;
						}
						else {
							_robotAngle = atan(abs(_robotPts[0].y - _robotPts[1].y)/abs(_robotPts[0].x - _robotPts[1].x));
							if (_robotPts[0].y < _robotPts[1].y && _robotPts[0].x < _robotPts[1].x) 
								_robotAngle = CV_PI - _robotAngle;
							else if (_robotPts[0].y > _robotPts[1].y && _robotPts[0].x < _robotPts[1].x) 
								_robotAngle = CV_PI + _robotAngle;
							else if (_robotPts[0].y > _robotPts[1].y && _robotPts[0].x > _robotPts[1].x) 
								_robotAngle = 2*CV_PI - _robotAngle;
						}
						break;
					}
				}
			}
		}
	}
}

IplImage * Webcam::getFinal()
{
	cvZero(_final);

	// Draw Arena
	CvPoint * corners = new CvPoint[4];
	corners[0].x = 0;				corners[0].y = 0;
	corners[1].x = _final->width;	corners[1].y = 0;
	corners[2].x = _final->width;	corners[2].y = _final->height;
	corners[3].x = 0;				corners[3].y = _final->height;
	for (int i = 0 ; i < 3; i++) {
		cvLine(_final, corners[i], corners[i+1], CV_RGB(0,0,255), 3, CV_AA, 0);
	}
	cvLine(_final, corners[3], corners[0], CV_RGB(0,0,255), 3, CV_AA, 0);

	CvPoint * topGoal = new CvPoint[4];

	topGoal[0].x = _final->width*2.5/8.0;	topGoal[0].y = 0;
	topGoal[1].x = _final->width*2.5/8.0;	topGoal[1].y = _final->height / 8;
	topGoal[2].x = _final->width*5.5/8.0;	topGoal[2].y = _final->height / 8;
	topGoal[3].x = _final->width*5.5/8.0;	topGoal[3].y = 0;
	for (int i = 0; i < 3; i++) {
		cvLine(_final, topGoal[i], topGoal[i+1], CV_RGB(0,0,255), 3, CV_AA, 0);
	}

	CvPoint * bottomGoal = new CvPoint[4];
	bottomGoal[0].x = _final->width*5.5/8.0;	bottomGoal[0].y = _final->height;
	bottomGoal[1].x = _final->width*5.5/8.0;	bottomGoal[1].y = _final->height*7/8;
	bottomGoal[2].x = _final->width*2.5/8.0;	bottomGoal[2].y = _final->height*7/8;
	bottomGoal[3].x = _final->width*2.5/8.0;	bottomGoal[3].y = _final->height;
	for (int i = 0; i < 3; i++) {
		cvLine(_final, bottomGoal[i], bottomGoal[i+1], CV_RGB(0,0,255), 3, CV_AA, 0);
	}
	
	// If Calibrated, Draw Objects
	if (_calibrated) {
		calculateFinal();
		for (int i = 0; i < _ballPts.size(); i++) {
			cvCircle(_final, _ballPts[i], _ballRadius, CV_RGB(0,255,0), 2, 8, 0);
		}

		if (_robotPts.size() == 2) {
			cvCircle(_final, _robotPts[0], _robotRadius, CV_RGB(0,0,255), 2, 8, 0);
			cvCircle(_final, _robotPts[1], _robotRadius, CV_RGB(0,0,128), 2, 8, 0);
		}
		for (int i = 0; i < _obstaclesPts.size(); i++) {
			cvCircle(_final, _obstaclesPts[i], _obstacleRadius, CV_RGB(255,0,0), 2, 8, 0);
		}
	}
	return _final;
}

double Webcam::dist(double x1, double x2, double y1, double y2)
{
	return sqrtf(
		pow(x1 - x2,2) +
		pow(y1 - y2,2)
		);
}