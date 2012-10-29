#ifndef GUI_H
#define GUI_H

#include <stdlib.h>

#include <opencv\cvaux.h>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>
#include <opencv2\opencv.hpp>

#include <QtGui>
#include "ui_gui.h"

#include "ArduinoCOMM.h"
#include "MovementAlgorithm.h"
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

	void writeLeftThreshold();
	void writeRightThreshold();

private:
	Ui::GUIClass ui;

	Webcam * _cam1;
	Webcam * _cam2;

	QLabel * _progressLabel;
	QProgressBar * _progressBar;
	QTimer * _timer;

	ArduinoCOMM * _arduino;
	
	bool test;

	IplImage * _image;
	IplImage * _topImage;

	vector<Point2f> _balls;
	vector<Point2f> _obstacles;
	vector<Point2f> _robot;
	double _robotAngle;

	void init();
	void displayImage(IplImage * webcamFeed, QLabel * location, int type = 0);
	void displayFinal(IplImage * webcamFeed, QLabel * location);
	void displayMain();
	void detectObstacles(); // Detect if there are any obstacles in front of the robot

	vector<Point2f> combinePts(vector<Point2f> pts1, vector<Point2f> pts2);
	void readLeftThreshold();
	void readRightThreshold();
	
	double dist(double x1, double x2, double y1, double y2);
	void log(QString text);
};

#endif // GUI_H
