#ifndef ARDUINOCOMM
#define ARDUINOCOMM

#include <QtGui>

#include "GLOBALS.h"

class ArduinoCOMM{
public:
	ArduinoCOMM(QString path);
	~ArduinoCOMM();

	void write(char * output);
	QString read();

private:
	QProcess _arduino;
};

#endif