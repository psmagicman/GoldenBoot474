#include "ArduinoCOMM.h"

ArduinoCOMM::ArduinoCOMM(QString path, int port)
{
	QStringList arguments;
	arguments << QString::number(port);
	_arduino.start(path, arguments);
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

void ArduinoCOMM::write(string output)
{
	_arduino.write((output+"\n").c_str());
}
