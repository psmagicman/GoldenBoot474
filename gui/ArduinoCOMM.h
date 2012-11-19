#ifndef __ARDUINOCOMM__
#define __ARDUINOCOMM__

#include <string>
#include <QtGui>
#include <qobject.h>

using namespace std;

class ArduinoCOMM
{
public:
	ArduinoCOMM(QString path);
	~ArduinoCOMM();

	QString read();
	void write(string input);

private:
	QProcess _arduino;
};

#endif