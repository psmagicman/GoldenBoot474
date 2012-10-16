#include <opencv\cvaux.h>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>
#include <opencv2\opencv.hpp>

#include <iostream>
#include <string>

#include "../GLOBALS.h"

using namespace std;

const string WebcamName = "Webcam1";
const string WebcamFilter = "Filter1";
const int ID = 0;

int main()
{
	/*
	cvSetCaptureProperty(capture, 320, 640);
	cvSetCaptureProperty(capture, 240, 360);
	*/

	CvSize size = cvSize(640, 480);
	CvCapture *capture = cvCaptureFromCAM(ID);
	cvNamedWindow(WebcamName.c_str(), CV_WINDOW_AUTOSIZE);
	CvScalar hsv_min = cvScalar(greenHueMin, greenSatMin, greenValMin);
	CvScalar hsv_max = cvScalar(greenHueMax, greenSatMax, greenValMax);
	CvScalar black_min = cvScalar(blackHueMin, blackSatMin, blackValMin);
	CvScalar black_max = cvScalar(blackHueMax, blackSatMax, blackValMax);
	IplImage* hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3);
	IplImage* thresholded = cvCreateImage(size, IPL_DEPTH_8U, 1);
	IplImage* thresholded2 = cvCreateImage(size, IPL_DEPTH_8U, 1);

	while (true) {
		IplImage *frame = cvQueryFrame(capture);
		cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
		cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
		cvInRangeS(hsv_frame, black_min, black_max, thresholded2);
		CvMemStorage *storage = cvCreateMemStorage(0);
		cvSmooth(thresholded, thresholded, CV_GAUSSIAN, 7, 7);
		cvSmooth(thresholded2, thresholded2, CV_GAUSSIAN, 5, 5);

		// Draw Circles
		CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 4, 50, 200, 30, 0, 30);
		for( int i = 0; i < circles->total; i++) {
			float * p = (float*)cvGetSeqElem(circles,i);
			cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])), 3, CV_RGB(0,255,0), -1, 8, 0);
			cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])), cvRound(p[2]), CV_RGB(0,255,0), 2, 8, 0);
		}

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
			//cvLine(frame, line[0], line[1], CV_RGB(0,255,255), 1, CV_AA, 0);
		}
		//cout << endl;
		cvLine(frame, topPts[0], topPts[1], CV_RGB(0,0,255), 1, CV_AA, 0);
		cvLine(frame, botPts[0], botPts[1], CV_RGB(0,0,255), 1, CV_AA, 0);


		cvNamedWindow("threshold2", CV_WINDOW_AUTOSIZE);
		cvShowImage("threshold2", thresholded2);
		cvShowImage(WebcamName.c_str(),frame);
		cvShowImage(WebcamFilter.c_str(), thresholded);
		cvReleaseMemStorage(&storage);

		//delete [] topPts;
		//delete [] botPts;
		Sleep(10);
		if (cvWaitKey(10) == 27) {
			break;
		}
	}

	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
	return 0;
}
