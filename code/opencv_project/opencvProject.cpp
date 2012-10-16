/*
*
* Functionality:
*	Demostrates the use of two webcams simultaneously 
*
* Citation:
*	http://apparentchaos.wordpress.com/2012/02/28/display-two-live-webcams-with-opencv/
* Author of original code:
*	Apparent Chaos
*
* Julien's note:
*	The code might have a problem with the USB bandwith in which the webcams may request more bandwidth than the USB can supply.
*	This is a hardware thing with USB 2.0, will need to find a way to reduce the amount of data being requested at the same time.
*	This is also noted by the original author of the code.
*	I did not encounter any errors for the amount of testing I have done, but be aware of the possibility that there can be a problem.
*	Better safe than sorry
**/

/*
#include <opencv\cvaux.h>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>
#include <opencv2\opencv.hpp>

#include <iostream>

using namespace std;

int main() {
	// Camera 1
	CvCapture *capture1 = cvCaptureFromCAM(1);
	cvSetCaptureProperty(capture1, 320, 1280);		// Set width 640; *note: The 160 can also be 320
	cvSetCaptureProperty(capture1, 240, 720);		// Set height 480; *note: The 120 can also be 240
																		// make sure if you are using 160, you are also using 120
																		// the second parameter of both calls need to maintain a 4 to 3 ratio
	/*CvCapture *capture2 = cvCaptureFromCAM(1);
	cvSetCaptureProperty(capture2, 320, 1280);		// Set width 640; *note: The 160 can also be 320
	cvSetCaptureProperty(capture2, 240, 720);		// Set height 480; *note: The 120 can also be 240
	*/

	/*
	// prints ERROR for the camera(s) that do not have a frame captured
	if(!cvQueryFrame(capture1) || !cvQueryFrame(capture2)) {
		if(!capture1)
			cout << "ERROR: Camera 1 is NULL" << endl << endl;
		else if(!capture2)
			cout << "ERROR: Camera 2 is NULL" << endl << endl;
		else
			cout << "ERROR: Both cameras are NULL" << endl << endl;
		// cleanup and terminate if an error is thrown
		cvReleaseCapture(&capture1);
		cvReleaseCapture(&capture2);
		return -1;
	}
	else
		cout << endl << "Success! Camera 1 and Camera 2 is captured" << endl << endl;
		*/
/*

	cvNamedWindow("Camera1", CV_WINDOW_AUTOSIZE);		// create a mount for Camera 1
	//cvNamedWindow("Camera2", CV_WINDOW_AUTOSIZE);		// create a mount for Camera 2

	while(1) {
		IplImage *frame1 = cvQueryFrame(capture1);	// capture frame from Camera 1
		//IplImage *frame2 = cvQueryFrame(capture2);	// capture frame from Camera 2
																			
		// outputs ERROR for the frame(s) that cannot capture a frame from their respective camera
		// not likely to happen, but good to throw errors when it is possible
		/*
		if(5!frame1 || !frame2) {
			if(!frame1)
				cout << endl << "ERROR: frame1 is NULL" << endl << endl;
			else if(!frame2)
				cout << endl << "ERROR: frame2 is NULL" << endl << endl;
			else
				cout << endl << "ERROR: both frames are NULL" << endl << endl;
			// breaks out of the forever loop when error is thrown
			break;
		}

		

		cvShowImage("Camera1", frame1);	// show feed for Camera 1
		Sleep(10);
		//cvShowImage("Camera2", frame2);	// show feed for Camera 2
		//Sleep(10);

		// press ESC to terminate
		if((cvWaitKey(10) & 255) == 27) 
			break;
	}

	// release the cameras for later use
	// basic cleanup procedure
	cvReleaseCapture(&capture1);
	//cvReleaseCapture(&capture2);
	cvDestroyWindow("Camera1");	// closes window for Camera 1
	//cvDestroyWindow("Camera2");	// closes window for Camera 2
	return 0;
}

*/
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
	CvCapture* capture = cvCaptureFromCAM(0);

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
	CvScalar hsv_min = cvScalar(30, 50, 50, 0);
	CvScalar hsv_max = cvScalar(40, 256, 256, 0);

	IplImage* hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3);
	IplImage* thresholded = cvCreateImage(size, IPL_DEPTH_8U, 1);
	IplImage* thresholded2 = cvCreateImage(size, IPL_DEPTH_8U, 1);

	float point[5][3] =
	{{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};

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
		cvSmooth(thresholded,thresholded, CV_GAUSSIAN, 7, 7);
		CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2, thresholded->height/4, 250, 100);
		
		for(int i=0; i<circles->total; i++) {
			float* p = (float*)cvGetSeqElem(circles, i);
			for (int j = 1; j < 5; j++) {
				point[j-1][0] = point[j][0];
				point[j-1][1] = point[j][1];
				point[j-1][2] = point[j][2];
			}
			point[4][0] = p[0];
			point[4][1] = p[1];
			point[4][2] = p[2];

			float pointX = 0;
			float pointY = 0;
			float pointZ = 0;
			for (int j = 0; j < 5; j++) {
				pointX += point[j][0];
				pointY += point[j][1];
				pointZ += point[j][2];
			}
			pointX = pointX/5;
			pointY = pointY/5;
			pointZ = pointZ/5;
			printf("Ball! x=%f y=%f\n\r", pointX, pointY, pointZ);
			cvCircle(frame, cvPoint(cvRound(pointX),cvRound(pointY)), 3, CV_RGB(0,255,0), -1, 8, 0);
			cvCircle(frame, cvPoint(cvRound(pointX),cvRound(pointY)), cvRound(pointZ), CV_RGB(255,0,0), 3, 8, 0);
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

		Sleep(10);
	}
	

	// Release the capture device housekeeping
	cvReleaseCapture(&capture);
	cvDestroyWindow("Camera");
	return 0;
}