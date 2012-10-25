#include "ArduinoCOMM.h"

ArduinoCOMM::ArduinoCOMM(QString path)
{
	_arduino.start(path);
}

ArduinoCOMM::~ArduinoCOMM()
{
	_arduino.close();
}

QString ArduinoCOMM::read()
{
	QByteArray data = _arduino.readAllStandardOutput();
	return QString::fromLocal8Bit(data);
}

void ArduinoCOMM::write(char * output)
{
	_arduino.write(output);
}
