#ifndef GUI_H
#define GUI_H

#include <opencv\cvaux.h>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>
#include <opencv2\opencv.hpp>

#include <QtGui>
#include "ui_gui.h"


#include "Webcam.h"

class GUI : public QMainWindow
{
	Q_OBJECT

public:
	GUI(QWidget *parent = 0, Qt::WFlags flags = 0);
	~GUI();

private slots:
	void display();
	void on_leftCalibrate_triggered();
	void on_rightCalibrate_triggered();
	void on_leftReset_triggered();
	void on_rightReset_triggered();

private:
	Ui::GUIClass ui;

	Webcam * _cam1;
	Webcam * _cam2;

	QLabel * _progressLabel;
	QProgressBar * _progressBar;
	QTimer * _timer;
	
	IplImage * _image;
	IplImage * _topImage;

	void init();
	void displayImage(IplImage * webcamFeed, QLabel * location, int type = 0);
	void displayFinal();
	void log(QString text);
};

#endif // GUI_H
