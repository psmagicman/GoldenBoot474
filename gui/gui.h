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
#include "Webcam.h"
#include "ThresholdFile.h"
#include "Algorithm.h"
#include "Ticks.h"
#include "XMLReader.h"

const int _commTime = 500;

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
	void on_leftObstacles_triggered();
	void on_rightObstacles_triggered();
	void on_leftReset_triggered();
	void on_rightReset_triggered();

	void on_leftsave_triggered() {_thresholdLeft->save();};
	void on_rightsave_triggered() {_thresholdRight->save();};
	void on_leftload_triggered() {_thresholdLeft->load();};
	void on_rightload_triggered() {_thresholdRight->load();};
	void on_leftArenaSave_triggered();
	void on_rightArenaSave_triggered();

	void writeLeftThreshold();
	void writeRightThreshold();

	void clear() {_task1 = false; _task2 = false; _task3 = false; _final = false; _state = 0; stopRobot(); log("CLEAR");};
	void task1() {_task1 = true; _ballsToScore = 1; _ballsScored = 0; _state = 0; log("Doing Task1 ...");};
	void task2() {_task2 = true; _ballsToScore = 1; _ballsScored = 0; _state = 0; log("Doing Task2 ...");};
	void task3() {_task3 = true; _ballsToScore = 1; _ballsScored = 0; _state = 0; log("Doing Task3 ...");};
	void final() {_final = true; _ballsToScore = 3; _ballsScored = 0; _state = 0; log("Doing Final ...");};

	void readProcess();

	void testGrab(){writeProcess("GRAB");};
	void testStop(){stopRobot();};
	void testSend();
	void testKick(){writeProcess("KICK");};
	void testRun(){writeProcess("RUN");};

private:
	Ui::GUIClass ui;

	Webcam *		_cam1;
	Webcam *		_cam2;

	ThresholdFile * _thresholdLeft;
	ThresholdFile * _thresholdRight;
	XMLReader * _arenaLeft;
	XMLReader * _arenaRight;

	QProcess *		_arduino;
	QLabel *		_progressLabel;
	QTimer *		_timer;
	QTime			_time;
	QString			_progressText;
	int				_prevTime;
	int				_prevFPS;
	int				_prevTaskTime;
	
	CAlgorithm		_algorithm;
	Ticks			_ticks;
	
	bool _obstaclesProcessed;
	bool _task1;
	bool _task2;
	bool _task3;
	bool _final;
	
	bool _run;
	int _state;
	int _prevState;
	int _errorState;
	// State 0 : Stoppin Robot
	// State 1 : Waiting for Stop Confirmation
	// State 2 : Looking for Ball
	// State 3 : Looking for Ball Acquired Confirmation
	// State 4 : Looking for Goal
	// State 5 : Looking for Goal Confirmation
	int _pathIndex;

	int _ballsToScore;
	int _ballsScored;

	IplImage * _image;
	IplImage * _topImage;

	vector<Point2f>		_balls;
	Point2f				_targetBall;
	vector<Point2f>		_obstacles;
	vector<Point2f>		_robot;
	Point2f				_prevRobot;
	vector<Point2f>		_opponent;
	vector<Coord2D>		_path;
	double				_robotAngle;
	vector<double>		_robotAngles;
	Coord2D				_goal;

	Robot				_algoRobot;
	vector<Ball>		_algoBalls;
	vector<Obstacle>	_algoObstacles;

	void init();
	void initVariables();
	void initLeftArena();
	void initRightArena();
	void processRobot();
	void processBalls();
	void displayImage(IplImage * webcamFeed, QLabel * location, int type = 0);
	void displayFinal(IplImage * webcamFeed, QLabel * location);
	void displayMain();
	void detectProblems(); // Detect if there are any obstacles in front of the robot or out of projectory

	void doTask1();
	void doTask2();
	void doTask3();
	void doFinal();
	void stopRobot();
	bool taskInit();
	void restartTask();

	bool calcPathToBall();
	bool sendBallCommand();

	bool calcPathToGoal();
	bool sendGoalCommand();

	vector<Point2f> combinePts(vector<Point2f> pts1, vector<Point2f> pts2, double distLimit);
	vector<Point2f> combineRobotPts(vector<Point2f> pts1, vector<Point2f> pts2);
	void readLeftThreshold();
	void readRightThreshold();
	
	void writeProcess(string command);
	void log(QString text);
};

#endif // GUI_H
