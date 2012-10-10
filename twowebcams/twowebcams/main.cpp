/**
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
#include <opencv\cvaux.h>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>
#include <opencv2\opencv.hpp>

#include <iostream>

using namespace std;

int main() {
	// Camera 1
	CvCapture *capture1 = cvCreateCameraCapture(0);
	cvSetCaptureProperty(capture1, 160, 640);		// Set width 640; *note: The 160 can also be 320
	cvSetCaptureProperty(capture1, 120, 480);		// Set height 480; *note: The 120 can also be 240
																		// make sure if you are using 160, you are also using 120
																		// the second parameter of both calls need to maintain a 4 to 3 ratio

	CvCapture *capture2 = cvCaptureFromCAM(2);
	cvSetCaptureProperty(capture2, 160, 640);		// Set width 640; *note: The 160 can also be 320
	cvSetCaptureProperty(capture2, 120, 480);		// Set height 480; *note: The 120 can also be 240

	// prints ERROR for the camera(s) that do not have a frame captured
	if(!cvQueryFrame(capture1) || !cvQueryFrame(capture2)) {
		if(!capture1)
			cout << "ERROR: Camera 1 is NULL" << endl << endl;
		elseif(!capture2)
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



	cvNamedWindow("Camera1", CV_WINDOW_AUTOSIZE);		// create a mount for Camera 1
	cvNamedWindow("Camera2", CV_WINDOW_AUTOSIZE);		// create a mount for Camera 2

	while(1) {
		IplImage *frame1 = cvQueryFrame(capture1);	// capture frame from Camera 1
																			// might be a good place to add something here to slow down the rate of data being requested
		IplImage *frame2 = cvQueryFrame(capture2);	// capture frame from Camera 2
																			// here is a good place too possibly; maybe add both?
																			
		// outputs ERROR for the frame(s) that cannot capture a frame from their respective camera
		// not likely to happen, but good to throw errors when it is possible
		if(!frame1 || !frame2) {
			if(!frame1)
				cout << endl << "ERROR: frame1 is NULL" << endl << endl;
			elseif(!frame2)
				cout << endl << "ERROR: frame2 is NULL" << endl << endl;
			else
				cout << endl << "ERROR: both frames are NULL" << endl << endl;
			// breaks out of the forever loop when error is thrown
			break;
		}
		

		cvShowImage("Camera1", frame1);	// show feed for Camera 1
		cvShowImage("Camera2", frame2);	// show feed for Camera 2

		// press ESC to terminate
		if((cvWaitKey(10) & 255) == 27) 
			break;
	}

	// release the cameras for later use
	// basic cleanup procedure
	cvReleaseCapture(&capture1);
	cvReleaseCapture(&capture2);
	cvDestroyWindow("Camera1");	// closes window for Camera 1
	cvDestroyWindow("Camera2");	// closes window for Camera 2
	return 0;
}