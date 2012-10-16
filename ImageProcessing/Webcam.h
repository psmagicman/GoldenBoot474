#ifndef __WEBCAM__
#define __WEBCAM__

#include <sstream>
#include <iostream>
#include <string>

#include <opencv\cvaux.h>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>
#include <opencv2\opencv.hpp>

#include "GLOBALS.h"
#include "CThread.h"

using namespace std;

class Webcam : public ActiveClass
{
public:
	Webcam(int ID);
	int WebcamStats(void *args);

private:
	int		_ID;
	string	_name;

	int main();
};

#endif