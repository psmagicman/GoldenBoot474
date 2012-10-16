#include "Webcam.h"

Webcam::Webcam(int ID)
{
	_ID = ID;
	
	stringstream out;
	out << ID;
	_name = "Webcam" + out.str();
}

int Webcam::WebcamStats(void *args)
{
	CvCapture *capture = cvCaptureFromCAM(_ID);
	cvSetCaptureProperty(capture, 320, 1280);		// Set width 640; *note: The 160 can also be 320
	cvSetCaptureProperty(capture, 240, 720);
	cvNamedWindow(_name.c_str(), CV_WINDOW_AUTOSIZE);
	while (1) {
		IplImage *frame = cvQueryFrame(capture);
		if (!frame) {
			break;
		}
		cvShowImage(_name.c_str(), frame);

		if (this->TerminateStatus()) {
			cvReleaseCapture(&capture);
			cvDestroyWindow(_name.c_str());
			return 0;
		}
		Sleep(10);
	}
	cvReleaseCapture(&capture);
	cvDestroyWindow(_name.c_str());
	return 0;
}

int Webcam::main()
{
	ClassThread<Webcam> WebcamThread(this,&Webcam::WebcamStats,ACTIVE,NULL);
	cout << _name << endl;

	system("pause");
	WebcamThread.RequestTerminate();
	return 0;
}