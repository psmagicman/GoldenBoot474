#include "gui.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	GUI w;
	w.showMaximized();
	return a.exec();
}
