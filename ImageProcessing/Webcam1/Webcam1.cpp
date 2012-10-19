#include <opencv\cvaux.h>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>
#include <opencv2\opencv.hpp>

#include <iostream>
#include <string>
#include <vector>

#include "../GLOBALS.h"

using namespace std;
using namespace cv;

const string WebcamName = "Webcam1";
const string WebcamFilter = "Filter1";
const int ID = 0;

int main()
{
	/*
	// CALIBURATION
	int numBoards = 10;
	int numCornersHor = 6;
	int numCornersVer = 9;

	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);

	VideoCapture capture = VideoCapture(ID);

	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;

	vector<Point2f> corners;
	int successes = 0;

	Mat image;
	Mat gray_image;
	capture >> image;

	vector<Point3f> obj;
	for (int j = 0; j < numSquares; j++) {
		obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));
	}

	while (successes < numBoards) {
		cvtColor(image, gray_image, CV_BGR2GRAY);
		bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found) {
			cornerSubPix(gray_image, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, board_sz, corners, found);
		}
		imshow("win1",image);
		imshow("win2",gray_image);
		capture >> image;
		int key = waitKey(1);
		if (key == 27) {
			return 0;
		}

		if (key == ' ' && found != 0) {
			image_points.push_back(corners);
			object_points.push_back(obj);
			printf("Snap stored!\n");
			successes++;
			if (successes >= numBoards)
				break;
		}

		Sleep(50);
	}
	

	Mat intrinsic = Mat(3,3,CV_32FC1);
	Mat distCoeffs;
	vector<Mat> rvecs;
	vector<Mat> tvecs;

	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(1)[1] = 1;

	calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

	Mat imageUndistorted;

	while (1)
	{
		capture >> image;
		undistort(image, imageUndistorted, intrinsic, distCoeffs);

		imshow("win1",image);
		imshow("win2",imageUndistorted);

		waitKey(1);
	}
	*/	

	/*
	cvSetCaptureProperty(capture, 320, 640);
	cvSetCaptureProperty(capture, 240, 360);
	*/

	
	CvSize size = cvSize(640, 480);
	CvSize finalSize = cvSize(800,800);
	CvCapture *capture = cvCaptureFromCAM(ID);
	cvNamedWindow(WebcamName.c_str(), CV_WINDOW_AUTOSIZE);
	CvScalar hsv_min = cvScalar(greenHueMin, greenSatMin, greenValMin);
	CvScalar hsv_max = cvScalar(greenHueMax, greenSatMax, greenValMax);
	CvScalar black_min = cvScalar(blackHueMin, blackSatMin, blackValMin);
	CvScalar black_max = cvScalar(blackHueMax, blackSatMax, blackValMax);

	IplImage* hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3);
	IplImage* thresholded = cvCreateImage(size, IPL_DEPTH_8U, 1);
	IplImage* thresholded2 = cvCreateImage(size, IPL_DEPTH_8U, 1);
	IplImage* final = cvCreateImage(finalSize, IPL_DEPTH_8U, 3);

	while (true) {
		IplImage *frame = cvQueryFrame(capture);
		cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
		cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
		cvInRangeS(hsv_frame, black_min, black_max, thresholded2);
		CvMemStorage *storage = cvCreateMemStorage(0);
		cvSmooth(thresholded, thresholded, CV_GAUSSIAN, 7, 7);
		cvSmooth(thresholded2, thresholded2, CV_GAUSSIAN, 5, 5);

		// Draw Lines
		
	 // Draw Arena Lines
		CvSeq * lines = cvHoughLines2(thresholded2, storage, CV_HOUGH_PROBABILISTIC, 1, 90*CV_PI/180, 100, 200, 0);
		//cout << lines->total << endl;
	
		CvPoint * topPts = new CvPoint[2];
		topPts[0].x = frame->width;
		topPts[0].y = frame->height;
		topPts[1].x = 0;
		topPts[1].y = frame->height;

		CvPoint * botPts = new CvPoint[2];
		botPts[0].x = frame->width;
		botPts[0].y = 0;
		botPts[1].x = 0;
		botPts[1].y = 0;

		for (int i = 0; i < lines->total; i++) {
			CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
			//cout << line[0].x << " " << line[0].y << " " << line[1].x << " " << line[1].y << endl;
			if (line[0].y < frame->height/2 && line[1].y < frame->height/2) {

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
			//cout << line[0].x << " " << line[0].y << " " << line[1].x << " " << line[1].y << endl;
			if (line[0].y > frame->height/2 && line[1].y > frame->height/2) {
				if (botPts[0].y < line[0].y || botPts[1].y < line[1].y) {
					if ((line[0].x > topPts[0].x+10) && (botPts[0].x > line[0].x)) botPts[0].x = line[0].x;
					if (botPts[0].y < line[0].y) botPts[0].y = line[0].y;
					if ((line[1].x < topPts[1].x-10) && (botPts[1].x < line[1].x)) botPts[1].x = line[1].x;
					if (botPts[1].y < line[1].y) botPts[1].y = line[1].y;
				}
			}
			cvLine(frame, line[0], line[1], CV_RGB(0,255,255), 1, CV_AA, 0);
		}

		//cout << endl;
		cvLine(frame, topPts[0], topPts[1], CV_RGB(0,0,255), 1, CV_AA, 0);
		cvLine(frame, botPts[0], botPts[1], CV_RGB(0,0,255), 1, CV_AA, 0);

		double topLen = sqrtf(
			((topPts[0].x - topPts[1].x) * (topPts[0].x - topPts[1].x)) +
			((topPts[0].y - topPts[1].y) * (topPts[0].y - topPts[1].y))
			);
		// 540

		double midLen = abs(topPts[0].y - botPts[0].y);
		// 446
		
		double midLen2 = topLen * 7.0 / 8.0;


		double _theta = acos(midLen / midLen2);

		double Yratio = final->height / frame->height;
		double Xratio = final->width / frame->width;
		
		double leftX = topPts[1].x - (topPts[1].x - topPts[0].x)*5.5/8.0;
		double leftSlope = midLen / (botPts[0].x - leftX);
		double leftIntercept = botPts[0].y - leftSlope*botPts[0].x;
		
		double rightX = topPts[1].x - (topPts[1].x - topPts[0].x)*2.5/8.0;
		double rightSlope = midLen / (botPts[1].x - rightX);
		double rightIntercept = botPts[1].y - rightSlope*botPts[1].x;

		CvPoint * leftTest = new CvPoint[2];
		leftTest[0].x = leftX;
		leftTest[0].y = topPts[1].y;
		leftTest[1].x = botPts[0].x;
		leftTest[1].y = botPts[0].y;
		cvLine(frame,leftTest[0], leftTest[1], CV_RGB(0,0,255));
		
		CvPoint * rightTest = new CvPoint[2];
		rightTest[0].x = rightX;
		rightTest[0].y = topPts[1].y;
		rightTest[1].x = botPts[1].x;
		rightTest[1].y = botPts[1].y;
		cvLine(frame,rightTest[0], rightTest[1], CV_RGB(0,0,255));

		double centerX = (rightIntercept - leftIntercept) / (leftSlope - rightSlope);
		double centerY = centerX * rightSlope + rightIntercept;

		//cout << leftSlope << " " << rightSlope << endl;
		//cout << centerX << " " << centerY << endl;
		// Draw Circles
		CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 4, 50, 200, 30, 3,10);
		CvPoint * circlePts = new CvPoint[circles->total];
		for( int i = 0; i < circles->total; i++) {
			float * p = (float*)cvGetSeqElem(circles,i);
			cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])), 3, CV_RGB(0,255,0), -1, 8, 0);
			cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])), cvRound(p[2]), CV_RGB(0,255,0), 2, 8, 0);
			circlePts[i].x = ((p[0] - topPts[0].x) - (p[1]-topPts[0].y)*(p[0]-centerX)/(p[1]-centerY))*Xratio;
			circlePts[i].y = (p[1]-topPts[0].y)/cos(_theta)*Yratio;
			cout << p[0] << " " << circlePts[i].x << "\t" << p[1] << " " << circlePts[i].y << endl;
		}

		CvPoint * corners = new CvPoint[4];
		corners[0].x = 0;
		corners[0].y = 0;
		corners[1].x = final->width;
		corners[1].y = 0;
		corners[2].x = final->width;
		corners[2].y = final->height;
		corners[3].x = 0;
		corners[3].y = final->height;
		for (int i = 0 ; i < 3; i++) {
			cvLine(final, corners[i], corners[i+1], CV_RGB(0,0,255), 3, CV_AA, 0);
		}
		cvLine(final, corners[3], corners[0], CV_RGB(0,0,255), 3, CV_AA, 0);

		CvPoint * topGoal = new CvPoint[4];
		topGoal[0].x = final->width*2.5/8.0;
		topGoal[0].y = 0;
		topGoal[1].x = final->width*2.5/8.0;
		topGoal[1].y = final->height / 8;
		topGoal[2].x = final->width*5.5/8.0;
		topGoal[2].y = final->height / 8;
		topGoal[3].x = final->width*5.5/8.0;
		topGoal[3].y = 0;
		for (int i = 0; i < 3; i++) {
			cvLine(final, topGoal[i], topGoal[i+1], CV_RGB(0,0,255), 3, CV_AA, 0);
		}

		CvPoint * bottomGoal = new CvPoint[4];
		bottomGoal[0].x = final->width*5.5/8.0;
		bottomGoal[0].y = final->height;
		bottomGoal[1].x = final->width*5.5/8.0;
		bottomGoal[1].y = final->height*7/8;
		bottomGoal[2].x = final->width*2.5/8.0;
		bottomGoal[2].y = final->height*7/8;
		bottomGoal[3].x = final->width*2.5/8.0;
		bottomGoal[3].y = final->height;
		for (int i = 0; i < 3; i++) {
			cvLine(final, bottomGoal[i], bottomGoal[i+1], CV_RGB(0,0,255), 3, CV_AA, 0);
		}

		CvPoint * midLine = new CvPoint[2];
		midLine[0].x = 0;
		midLine[0].y = final->height/2;
		midLine[1].x = final->width;
		midLine[1].y = final->height/2;
		cvLine(final, midLine[0], midLine[1], CV_RGB(0,0,255), 3, CV_AA, 0);

		for(int i = 0; i < circles->total; i++) {
			cvCircle(final, circlePts[i], 3, CV_RGB(0,255,0), 2, 8, 0);
		}

		cvNamedWindow("final",CV_WINDOW_AUTOSIZE);
		cvShowImage("final",final);

		
		cvShowImage(WebcamName.c_str(),frame);

		//cvNamedWindow("threshold2", CV_WINDOW_AUTOSIZE);
		//cvShowImage("threshold2", thresholded2);
		//cvNamedWindow(WebcamFilter.c_str(), CV_WINDOW_AUTOSIZE);
		//cvShowImage(WebcamFilter.c_str(), thresholded);
		

		cvReleaseMemStorage(&storage);

		delete [] topPts;
		delete [] botPts;
		delete [] corners;
		delete [] circlePts;
		Sleep(10);
		if (cvWaitKey(10) == 27) {
			break;
		}
	}

	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
	return 0;
}
