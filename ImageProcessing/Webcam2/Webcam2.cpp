#include <opencv\cvaux.h>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>
#include <opencv2\opencv.hpp>

#include <iostream>
#include <string>

#include "../GLOBALS.h"

using namespace std;

const string WebcamName = "Webcam2";
const string WebcamFilter = "Filter2";
const int ID = 1;

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
	IplImage* hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3);
	IplImage* thresholded = cvCreateImage(size, IPL_DEPTH_8U, 1);

	while (true) {
		cout << greenHueMin << " " << greenHueMax << endl;

		IplImage *frame = cvQueryFrame(capture);
		cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
		cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
		CvMemStorage *storage = cvCreateMemStorage(0);
		cvSmooth(thresholded, thresholded, CV_GAUSSIAN, 7, 7);
		CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2, thresholded->height/4, 250, 100);
		for( int i = 0; i < circles->total; i++) {
			float * p = (float*)cvGetSeqElem(circles,i);
			cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])), 3, CV_RGB(0,255,0), -1, 8, 0);
			cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])), cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0);
		}
		cvShowImage(WebcamName.c_str(),frame);
		cvShowImage(WebcamFilter.c_str(), thresholded);
		cvReleaseMemStorage(&storage);


		Sleep(1);
		if (cvWaitKey(10) == 27) {
			break;
		}
	}

	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
	return 0;
}
