#ifndef __THRESHOLDFILE__
#define __THRESHOLDFILE__

#include <QtGui>
#include <string>
#include <sstream>

#include "XMLReader.h"
#include "Webcam.h"

using namespace std;

class ThresholdFile
{
public:
	ThresholdFile();
	ThresholdFile(Webcam * cam, QString path);

	void load();
	void save();

private:
	Webcam * _cam;
	XMLReader _xml;

	QString _path;	

	void loadData();
	void saveData();
	string itoa(int value);
};

#endif