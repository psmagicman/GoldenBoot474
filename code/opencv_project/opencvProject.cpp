// Need to do some citation here...
// Site: http://www.lirtex.com
//WebPage: http://www.lirtex.com/robotics/fast-object-tracking-robot-computer-vision/
// opencv_test.cpp : Defines the entry point for the console application.
//

#include <opencv\cvaux.h>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>
#include <opencv2\opencv.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>


int main() {
	// Default capture size - 640x480
	CvSize size = cvSize(640, 480);

	// Initialize capturing live feed from camera
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	// No device? Print error and then quit
	if(!capture) {
		fprintf(stderr, "ERROR: capture is NULL \n");
		getchar();
		return -1;
	}
	
	// Create a window in which the captured images will be presented
	cvNamedWindow("Camera", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("EdgeDetection", CV_WINDOW_AUTOSIZE);

	// Detect a red ball
	//CvScalar hsv_min = cvScalar(80, 240, 180, 0);
	//CvScalar hsv_max = cvScalar(120, 240, 30, 0);

	// Detect a green ball
	CvScalar hsv_min = cvScalar(30, 100, 100, 0);
	CvScalar hsv_max = cvScalar(40, 255, 255, 0);

	IplImage* hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3);
	IplImage* thresholded = cvCreateImage(size, IPL_DEPTH_8U, 1);
	IplImage* thresholded2 = cvCreateImage(size, IPL_DEPTH_8U, 1);

	// Show the image captured from the camera in the window and repeat
	while(1) {
		// Get one frame
		IplImage* frame = cvQueryFrame(capture);

		// If we can't grab a frame then quit
		if(!frame) {
			fprintf(stderr, "ERROR: frame is null...\n");
			getchar();
			break;
		}

		// Effects Start

		// Convert color space to HSV as it is much easier to filter colors in the HSV color-space
		cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
		// Filter out colors which are out of range
		cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
		// Memory for hough circles
		CvMemStorage* storage = cvCreateMemStorage(0);
		// hough detector works better with some smoothing of the image
		cvSmooth(thresholded,thresholded, CV_GAUSSIAN, 9, 9);
		CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2, thresholded->height/4, 100, 50, 10, 400);
		for(int i=0; i<circles->total; i++) {
			float* p = (float*)cvGetSeqElem(circles, i);
			printf("Ball! x=%f y=%f\n\r", p[0], p[1], p[2]);
			cvCircle(frame, cvPoint(cvRound(p[0]),cvRound(p[1])), 3, CV_RGB(0,255,0), -1, 8, 0);
			cvCircle(frame, cvPoint(cvRound(p[0]),cvRound(p[1])), cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0);
		}

		// Effects End

		// Showing the image from the camera
		cvShowImage("Camera", frame);
		cvShowImage("HSV", hsv_frame);
		cvShowImage("After Color Filtering", thresholded);
		cvReleaseMemStorage(&storage);

		// Do not release the frame!
		// If ESC key is pressed, Key=0x10001B.
		// remove higher bits using AND operator
		if((cvWaitKey(10) & 255) == 27)
			break;
	}
	

	// Release the capture device housekeeping
	cvReleaseCapture(&capture);
	cvDestroyWindow("Camera");
	return 0;
}