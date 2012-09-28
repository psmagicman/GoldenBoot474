// Made changes to Juliens code in order to implement shape detection  
// Site: http://programing-tutorial.blogspot.ca/2009/11/image-proseesing-extacting-shapes-using.html
// Revision date : 27/09/2012

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
	CvSeq*  contours;
	// No device? Print error and then quit
	if(!capture) {
		fprintf(stderr, "ERROR: capture is NULL \n");
		getchar();
		return -1;
	}
	
	// Create a window in which the captured images will be presented
	cvNamedWindow("Camera", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("EdgeDetection", CV_WINDOW_AUTOSIZE);

	// Detect a red ball
	//CvScalar hsv_min = cvScalar(150, 84, 130, 0);
	//CvScalar hsv_max = cvScalar(358, 256, 255, 0);

	// Detect an orange ball
	//CvScalar hsv_min = cvScalar(0, 50, 170, 0);
	//CvScalar hsv_max = cvScalar(10, 180, 256, 0);
	//CvScalar hsv_min2 = cvScalar(170, 50, 170, 0);
	//CvScalar hsv_max2 = cvScalar(256, 180, 256, 0);

	//cvScalar(blue,green,red,alpha)

	//IplImage* hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3);
	//IplImage* thresholded = cvCreateImage(size, IPL_DEPTH_8U, 1);
	//IplImage* thresholded2 = cvCreateImage(size, IPL_DEPTH_8U, 1);

	IplImage* img = cvCreateImage(size, IPL_DEPTH_8U, 1);
	IplImage* gray = cvCreateImage( size, IPL_DEPTH_8U, 1 );
    cvCvtColor(img,gray,CV_BGR2GRAY );

	uchar *data;
    data=(uchar* )img->imageData;

    uchar *data_gray;
    data_gray=(uchar* )gray->imageData;

    int  step_g=gray->widthStep;
    int  step=img->widthStep;
    int channels=img->nChannels;

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
		//cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
		// Filter out colors which are out of range
		//cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
		//cvInRangeS(hsv_frame, hsv_min2, hsv_max2, thresholded2);
		//cvOr(thresholded, thresholded2, thresholded);
		
		
		// Memory for hough circles
		CvMemStorage* storage = cvCreateMemStorage(0);
		// hough detector works better with some smoothing of the image
		//cvSmooth(thresholded,thresholded, CV_GAUSSIAN, 9, 9);
		for(int i=0;640;i=i++)
     {
         for(int j=0;480;j++)
         {
             
             
             /////////////exracting  white
             if( (data[i*step+j*channels+0]>200) && (data[i*step+j*channels+1]>200) && (data[i*step+j*channels+2]>220) )
                 data_gray[i*step_g+j] = 255;


            else 
                data_gray[i*step_g+j] = 0;

         }
         
     }

   // find counters in the gray image - object detection
    cvFindContours( gray, storage, &contours, sizeof(CvContour),
                    CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE , cvPoint(0,0) );

    

    double area;
    double p;
    double metric_s,metric_c;

// loop through all counturs    

for( ; contours != 0; contours = contours->h_next )
    {
        area=fabs( cvContourArea(contours,CV_WHOLE_SEQ ) );
        p=fabs( cvArcLength( contours, CV_WHOLE_SEQ, -1) );

        // determine metric for circle
        metric_c=area*4*3.14/(p*p);
        metric_s=area*16/(p*p);

        if(metric_c > 0.75f && metric_s > 1.0f)
            printf(" circle \n");

        else if(metric_s > 0.9f && metric_c < 0.8f )
            printf(" square \n");

        else
            printf(" nothing ");


        printf("area = %lf perimeter=%lf metric_c=%lf metric_s=%lf\n",p,area,metric_c,metric_s);



    }

		//CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2, thresholded->height/4, 100, 50, 10, 400);
		//for(int i=0; i<circles->total; i++) {
		//	float* p = (float*)cvGetSeqElem( circles, i);
			//printf("Ball! x=%f y=%f\n\r", p[0], p[1], p[2]);
			//cvCircle(frame, cvPoint(cvRound(p[0]),cvRound(p[1])), 3, CV_RGB(0,255,0), -1, 8, 0);
			//cvCircle(frame, cvPoint(cvRound(p[0]),cvRound(p[1])), cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0);
		}

		// Effects End

		// Showing the image from the camera
		//cvShowImage("Camera", frame);
		//cvShowImage("HSV", hsv_frame);
		//cvShowImage("After Color Filtering", thresholded);

	     // cvShowImage("Camera", frame);
		  //cvShowImage("HSV", hsv_frame);
		//cvReleaseMemStorage(&storage);
		cvReleaseImage(&img);
        cvReleaseImage(&gray);

		// Do not release the frame!
		// If ESC key is pressed, Key=0x10001B.
		// remove higher bits using AND operator
	//	{if((cvWaitKey(10) & 255) == 27)
	//		break;
	//	}
	
	// Release the capture device housekeeping
	cvReleaseCapture(&capture);
	cvDestroyWindow("Camera");
	return 0;
}