#include "gui.h"

GUI::GUI(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);

	// Start Recording!
	init();
	_timer->start(50);
}

GUI::~GUI()
{
	_arduino->write("EXIT");
	_arduino->~ArduinoCOMM();
	_cam1->~Webcam();
	_cam2->~Webcam();
}

void GUI::init()
{
	// Initialize Status Bar
	_progressBar = new QProgressBar;
	_progressBar->setAlignment(Qt::AlignCenter);
	_progressLabel = new QLabel;
	_progressLabel->setAlignment(Qt::AlignRight);
	log("MAIN PROGRAM - Initializing ...");
	ui.statusBar->addPermanentWidget(_progressLabel, WIDTH);
	ui.statusBar->addPermanentWidget(_progressBar, WIDTH/2);
	_progressBar->setValue(100);
	
	// Connect Buttons
	connect(ui.leftCalibrate, SIGNAL(clicked()), this, SLOT(on_leftCalibrate_triggered()));
	connect(ui.rightCalibrate, SIGNAL(clicked()), this, SLOT(on_rightCalibrate_triggered()));
	connect(ui.leftObstacles, SIGNAL(clicked()), this, SLOT(on_leftObstacles_triggered()));
	connect(ui.rightObstacles, SIGNAL(clicked()), this, SLOT(on_rightObstacles_triggered()));
	connect(ui.leftReset, SIGNAL(clicked()), this, SLOT(on_leftReset_triggered()));
	connect(ui.rightReset, SIGNAL(clicked()), this, SLOT(on_rightReset_triggered()));
	
	connect(ui.leftSave, SIGNAL(clicked()), this, SLOT(on_leftsave_triggered()));
	connect(ui.rightSave, SIGNAL(clicked()), this, SLOT(on_rightsave_triggered()));
	connect(ui.leftLoad, SIGNAL(clicked()), this, SLOT(on_leftload_triggered()));
	connect(ui.rightLoad, SIGNAL(clicked()), this, SLOT(on_rightload_triggered()));

	connect(ui.spinLeftHueMin, SIGNAL(valueChanged(int)), this, SLOT(writeLeftThreshold()));
	connect(ui.spinLeftHueMax, SIGNAL(valueChanged(int)), this, SLOT(writeLeftThreshold()));
	connect(ui.spinLeftSatMin, SIGNAL(valueChanged(int)), this, SLOT(writeLeftThreshold()));
	connect(ui.spinLeftSatMax, SIGNAL(valueChanged(int)), this, SLOT(writeLeftThreshold()));
	connect(ui.spinLeftValMin, SIGNAL(valueChanged(int)), this, SLOT(writeLeftThreshold()));
	connect(ui.spinLeftValMax, SIGNAL(valueChanged(int)), this, SLOT(writeLeftThreshold()));
	connect(ui.spinLeftAreaMin, SIGNAL(valueChanged(int)), this, SLOT(writeLeftThreshold()));
	connect(ui.spinLeftAreaMax, SIGNAL(valueChanged(int)), this, SLOT(writeLeftThreshold()));
	
	connect(ui.spinRightHueMin, SIGNAL(valueChanged(int)), this, SLOT(writeRightThreshold()));
	connect(ui.spinRightHueMax, SIGNAL(valueChanged(int)), this, SLOT(writeRightThreshold()));
	connect(ui.spinRightSatMin, SIGNAL(valueChanged(int)), this, SLOT(writeRightThreshold()));
	connect(ui.spinRightSatMax, SIGNAL(valueChanged(int)), this, SLOT(writeRightThreshold()));
	connect(ui.spinRightValMin, SIGNAL(valueChanged(int)), this, SLOT(writeRightThreshold()));
	connect(ui.spinRightValMax, SIGNAL(valueChanged(int)), this, SLOT(writeRightThreshold()));
	connect(ui.spinRightAreaMin, SIGNAL(valueChanged(int)), this, SLOT(writeRightThreshold()));
	connect(ui.spinRightAreaMax, SIGNAL(valueChanged(int)), this, SLOT(writeRightThreshold()));

	connect(ui.buttonTask1, SIGNAL(clicked()), this, SLOT(task1()));
	connect(ui.buttonTask2, SIGNAL(clicked()), this, SLOT(task2()));
	connect(ui.buttonTask3, SIGNAL(clicked()), this, SLOT(task3()));
	connect(ui.buttonFinal, SIGNAL(clicked()), this, SLOT(final()));

	// Initialize Timer
	_timer = new QTimer(this);
	connect(_timer, SIGNAL(timeout()), this, SLOT(display()));

	// Initialize Webcams
	_cam1 = new Webcam(0);
	_cam2 = new Webcam(1);
	_thresholdLeft = new ThresholdFile(_cam1, "LabLeft.xml");
	_thresholdRight = new ThresholdFile(_cam2, "LabRight.xml");
	
	// Initialize Communication with Arduino
	_arduino = new ArduinoCOMM("C:/Users/Wilbur/Desktop/GoldenBoot474/Debug/SerialCOMM.exe");

	_task1 = false;
	_task2 = false;
	_task3 = false;
	_final = false;
	_prevTask = 0;

	_pathIndex = 0;

	_robotAngles.resize(5);
	for (int i = 0; i < _robotAngles.size(); i++) {
		_robotAngles[i] = -1;
	}
	_state = 0;

	// Initialize Images
	_image = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	_topImage = cvCreateImage(cvSize(FINAL_WIDTH,FINAL_HEIGHT), IPL_DEPTH_8U, 3);

	log("MAIN PROGRAM - Initializing ... COMPLETE");
}

void GUI::display()
{
	if (ui.mainTab->currentIndex() == 0) {
		if (_cam1->capture()) {
			if (ui.leftTab->currentIndex() == 0) {
				_cam1->calculateNormal(ui.leftArenaCheck->isChecked(),ui.leftBallsCheck->isChecked(),ui.leftObstaclesCheck->isChecked(),ui.leftRobotCheck->isChecked(), true);
				displayImage(_cam1->getNormal(), ui.leftRGB);
			} else if (ui.leftTab->currentIndex() == 1) {
				displayImage(_cam1->getHSV(), ui.leftHSV);
			} else if (ui.leftTab->currentIndex() == 2) {
				string type = "";
				if (ui.leftArenaRadio->isChecked()) type = "Arena";
				else if (ui.leftBallsRadio->isChecked()) type = "Balls";
				else if (ui.leftObstacles1Radio->isChecked()) type = "Obstacles1";
				else if (ui.leftObstacles2Radio->isChecked()) type = "Obstacles2";
				else if (ui.leftRobot1Radio->isChecked()) type = "Robot1";
				else if (ui.leftRobot2Radio->isChecked()) type = "Robot2";
				_cam1->calculateThreshold(type);
				displayImage(_cam1->getThreshold(), ui.leftThreshold, 1);
				readLeftThreshold();
			}
			_cam1->release();
		}
		if (_cam2->capture()) {
			if (ui.rightTab->currentIndex() == 0) {
				_cam2->calculateNormal(ui.rightArenaCheck->isChecked(),ui.rightBallsCheck->isChecked(),ui.rightObstaclesCheck->isChecked(),ui.rightRobotCheck->isChecked(), true);
				displayImage(_cam2->getNormal(), ui.rightRGB);
			} else if (ui.rightTab->currentIndex() == 1) {
				displayImage(_cam2->getHSV(), ui.rightHSV);
			} else if (ui.rightTab->currentIndex() == 2) {
				string type = "";
				if (ui.rightArenaRadio->isChecked()) type = "Arena";
				else if (ui.rightBallsRadio->isChecked()) type = "Balls";
				else if (ui.rightObstacles1Radio->isChecked()) type = "Obstacles1";
				else if (ui.rightObstacles2Radio->isChecked()) type = "Obstacles2";
				else if (ui.rightRobot1Radio->isChecked()) type = "Robot1";
				else if (ui.rightRobot2Radio->isChecked()) type = "Robot2";
				_cam2->calculateThreshold(type);
				displayImage(_cam2->getThreshold(), ui.rightThreshold, 1);
				readRightThreshold();
			}
			_cam2->release();
		}
	} else if (ui.mainTab->currentIndex() == 1) {
		if (_cam1->capture() && _cam2->capture()) {
			if (ui.mainCamTab->currentIndex() == 0) {
				displayFinal(_cam1->getFinal(), ui.leftFinal);
				displayFinal(_cam2->getFinal(), ui.rightFinal);
			} else if (ui.mainCamTab->currentIndex() == 1) {
				_cam1->calculateFinal();
				_cam2->calculateFinal();
				displayImage(_cam1->getNormal(), ui.leftRaw);
			} else if (ui.mainCamTab->currentIndex() == 2) {
				_cam1->calculateFinal();
				_cam2->calculateFinal();
				displayImage(_cam2->getNormal(), ui.rightRaw);
			}
			// Process Both Cameras
			_balls = combinePts(_cam1->getBalls(),_cam2->getBalls(), 20);
			// Process Both Obstacles
			_obstacles = combinePts(_cam1->getObstacles(), _cam2->getObstacles(), 50);
			// Process Both Robots
			_robot = combinePts(_cam1->getRobot(), _cam2->getRobot(), 20);
			double leftAngle = -1;
			double rightAngle = -1;
			if (_cam1->getRobot().size() > 0) leftAngle = _cam1->getRobotAngle();
			if (_cam2->getRobot().size() > 0) rightAngle = _cam2->getRobotAngle();
			if (rightAngle >= CV_PI) rightAngle -= CV_PI;
			else rightAngle += CV_PI;
			if (leftAngle == -1) leftAngle = rightAngle;
			if (rightAngle == -1) rightAngle = leftAngle;

			double tempAngle;
			if (abs(leftAngle - rightAngle) > CV_PI) {
				tempAngle = 0;
			} else {
				tempAngle = (leftAngle + rightAngle)/2;
			}
			
			_robotAngles[0] = tempAngle;
			for (int i = _robotAngles.size()-1; i > 0; i--) {
				_robotAngles[i] = _robotAngles[i-1];
			}
			
			_robotAngle = 0;
			int angleSize = 0;
			for (int i = 0; i < _robotAngles.size(); i++) {
				if (abs(_robotAngles[i]-_robotAngles[0]) < PI/2) {
					_robotAngle += _robotAngles[i];
					angleSize++;
				}
			}
			_robotAngle /= angleSize;
			
			ui.labelRobotAngle->setText("Left Angle: " + QString::number((int)(leftAngle*180/CV_PI)) + "   Right Angle: " + QString::number((int)(rightAngle*180/CV_PI)) + "   Combined Angle: " + QString::number((int)(_robotAngles[0]*180/CV_PI)) + "   Corrected Angle: " + QString::number((int)(_robotAngle*180/CV_PI)));

			// Algorithm
			//detectProblems();
			if (_task1 || _task2 || _task3) {
				if (_balls.size() > 0 && _robot.size() > 0) { 
					Robot robot;
					vector<Ball> balls;
					vector<Obstacle> obstacles;
				
					double xRatio = (double)800.0 / FINAL_WIDTH;
					double yRatio = (double)800.0 / FINAL_HEIGHT;

					robot.x = (double)_robot[1].x * xRatio;
					robot.y = (double)(FINAL_HEIGHT - _robot[1].y) * yRatio; // Flip the Y-axis
					robot.angle = _robotAngle;

					balls.resize(_balls.size());
					for (int i = 0; i < _balls.size(); i++) {
						balls[i].x = (double)_balls[i].x * xRatio;
						balls[i].y = (double)(FINAL_HEIGHT - _balls[i].y) * yRatio; // Flip the Y-axis
					}

					obstacles.resize(_obstacles.size());
					for (int i = 0; i < _obstacles.size(); i++) {
						obstacles[i].x = (double)_obstacles[i].x * xRatio;
						obstacles[i].y = (double)(FINAL_HEIGHT - _obstacles[i].y) * yRatio; // Flip the Y-axis
					}
					
					_testAlgorithm = testAlgorithm(obstacles);
					// Task 1 : Move Towards a Single Ball and Touch it
					if (_task1) {
						_prevTask = 1;
						_algorithm = MovementAlgorithm(robot, balls);
						_path.clear();
						_pathIndex = 0;
						Coord2D robotLocation;
						robotLocation.x = robot.x;
						robotLocation.y = robot.y;
						_path.push_back(robotLocation);
						_path.push_back(_algorithm.returnClosestBall()); // I'll be using this to draw a line from Robot to Closest Ball
						for (int i = 0; i < _path.size(); i++) {
							_path[i].x = _path[i].x / 800.0 * FINAL_WIDTH;
							_path[i].y = FINAL_HEIGHT - (_path[i].y / 800.0 * FINAL_HEIGHT);
						}
						vector<Coord2D> directions = _algorithm.returnPath(); // I'll be using this to make a list of Commands to Ball
						QString message = "Task 1: ";
						if (directions.size() > 0) {
							message += "BEGIN ";
							_arduino->write("STOP");
						}
						for (int i = 0; i < directions.size(); i++) {
							_arduino->write("START");
							char RTicksStr[5];
							itoa(directions[i].y,RTicksStr,10);
							_arduino->write((string)RTicksStr);

							char LTicksStr[5];
							itoa(directions[i].x,LTicksStr,10);
							_arduino->write((string)LTicksStr);
							message += "<" + QString::number(directions[i].y) + "," + QString::number(directions[i].x) + "> ";
						}
						message += " " + QString::number(_algorithm.returnAngle());
						if (directions.size() > 0) {
							_arduino->write("EOL");
							message += " EOL";
						}
						ui.labelTicks->setText(message);
						_task1 = false;
					}

					// Task 2 : Move Towards a Single Ball with 2 Obstacles and Touch it
					if (_task2) {
						_prevTask = 2;
						_testAlgorithm.analyzeField(robot,balls);
						_path.clear();
						_path = _testAlgorithm.getClosestPath();
						vector<Coord2D> _tick = _testAlgorithm.getClosestTick();
						for (int i = 0; i < _path.size(); i++) {
							_path[i].x /= 2;
							_path[i].y /= 2;
							_path[i].y = FINAL_HEIGHT - _path[i].y;
						}
						QString tickMessage = "Ticks: ";
						for (int i = 0; i < _tick.size(); i++) {
							tickMessage += "(" + QString::number(_tick[i].x) + "," + QString::number(_tick[i].y) + ") ";
						}
						/*
						_previousLocation.x = _robot[0].x;
						_previousLocation.y = _robot[0].y;
						_path = _algorithm.getClosestBall(robot, balls);
						vector<Coord2D> directions = _algorithm.pathToClosestBall(robot, balls, obstacles);
						QString message = "Task 2: ";
						for (int i = 0; i < directions.size(); i++) {
							char LTicksStr[5]
							itoa(directions[i].x,LTicksStr,10);
							_arduino->write((string)LTicksStr);

							char RTicksStr[5]
							itoa(directions[i].y,RTicksStr,10);
							_arduino->write((string)RTicksStr);
							message += "<" + QString::number(directions[i].x) + "," + QString::number(directions[i].y) + "> ";
						}
						log(message);
						_arduino->write("EOF");
						_task2 = false;
						*/
						_task2 = false;
					}

					// Task 3 : Move all Balls to Goal with 4 Obstacles
					if (_task3) {
						_prevTask = 3;
						/*
						if (_state == 0) {
							_previousLocation.x = _robot[0].x;
							_previousLocation.y = _robot[0].y;
							_path = _algorithm.getClosestBall(robot, balls);
							vector<Coord2D> directions = _algorithm.pathToClosestBall(robot, balls, obstacles);
							for (int i = 0; i < directions.size(); i++) {
								char LTicksStr[5]
								itoa(directions[i].x,LTicksStr,10);
								_arduino->write((string)LTicksStr);

								char RTicksStr[5]
								itoa(directions[i].y,RTicksStr,10);
								_arduino->write((string)RTicksStr);
							}
							_arduino->write("EOF");
							state = 1;						
						 } else if (_state == 1) {
							_previousLocation.x = _robot[0].x;
							_previousLocation.y = _robot[0].y;
							vector<Coord2D> directions = _algorithm.pathToGoal(robot, balls, obstacles);
							for (int i = 0; i < directions.size(); i++) {
								char LTicksStr[5]
								itoa(directions[i].x,LTicksStr,10);
								_arduino->write((string)LTicksStr);

								char RTicksStr[5]
								itoa(directions[i].y,RTicksStr,10);
								_arduino->write((string)RTicksStr);
							}
							_arduino->write("EOF");
							state = 2;
						 } else if (_state == 2) {
							_arduino->write("KICK");
							_arduino->write("EOF");
							_ballsScored++;
							state = 0;
							if (_ballsScored >= _ballsToScore) {
								_task3 = false;
							}
						 }
						*/
						_task3 = false;
					}
				}
				else {
					log("ERROR: No Balls or Robot detected");
				}
			}
			/*
			else {
				_path.x = -1;
				_path.y = -1;
			}
			*/
			displayMain();
			_cam1->release();
			_cam2->release();
		}
		else {
			log ("ERROR: No Cameras Detected");
		}
	}
}

vector<Point2f> GUI::combinePts(vector<Point2f> pts1, vector<Point2f> pts2, double distLimit)
{
	vector<Point2f> combined;
	vector<Point2f> tempPts;
	for (int i = 0; i < pts2.size(); i++) {
		pts2[i].x = FINAL_WIDTH - pts2[i].x;
		pts2[i].y = FINAL_HEIGHT - pts2[i].y;
	}
	if (pts1.size() > pts2.size()) {
		combined = pts1;
		tempPts = pts2;
	} else {
		combined = pts2;
		tempPts = pts1;
	}
	for (int i = 0; i < tempPts.size(); i++) {
		for (int j = 0; j < combined.size(); j++) {
			if (dist(tempPts[i].x, combined[j].x, tempPts[i].y, combined[j].y) < distLimit) {
				combined[j] = Point2f(
					(combined[j].x + tempPts[i].x)/2 , (combined[j].y + tempPts[i].y)/2
					);
				break;
			}
			if (j == combined.size()-1) {
				combined.push_back(tempPts[i]);
			}
		}
	}
	return combined;
}

void GUI::detectProblems()
{
	if (_robot.size() > 0) {
		// Detect Obstacles in Front
		for (int i = 0; i < _obstacles.size(); i++) {
			double robotX = _robot[1].x;
			double robotY = _robot[1].y;
			if (dist(robotX, _obstacles[i].x, robotY, _obstacles[i].y) < 25) {
				double obstacleAngle = 0;
				double xDiff = _obstacles[i].x - robotX;
				double yDiff = _obstacles[i].y - robotY;
				if (abs(xDiff) < 0.5) {
					if (yDiff > 0) obstacleAngle = CV_PI/2;
					else obstacleAngle = 3*CV_PI/2;
				}
				else if(abs(yDiff) < 0.5) {
					if (xDiff > 0) obstacleAngle = 0;
					else obstacleAngle = CV_PI;
				}
				else {
					obstacleAngle = atan(abs(yDiff) / abs(xDiff));
					if (_obstacles[i].y > robotY && _obstacles[i].x < robotX) obstacleAngle = CV_PI/2 - obstacleAngle;
					else if (_obstacles[i].y < robotY && _obstacles[i].x < robotX) obstacleAngle = CV_PI/2 + obstacleAngle;
					else if (_obstacles[i].y < robotY && _obstacles[i].x > robotX) obstacleAngle = 2*CV_PI - obstacleAngle;
				}
				
				if (abs(obstacleAngle - _robotAngle) < 0.1) {
					if (_prevTask == 1) _task1 = true;
					else if (_prevTask == 2) _task2 = true;
					else if (_prevTask == 3) _task3 = true;
					log("STOP - OBSTACLE IN FRONT");
				}
			}
		}
		// Detect if Robot is still On-Route
		// cosTheta = A DOT B / (LEN(A) * LEN(B))
		if (_path.size() > 1) {
			Coord2D A;
			A.x = _robot[1].x;
			A.y = _robot[1].y;
			Coord2D B;
			B.x = _path[_pathIndex+1].x;
			B.y = _path[_pathIndex+1].y;
			Coord2D C;
			C.x = _path[_pathIndex].x;
			C.x = _path[_pathIndex].y;
			if (distFromLine(A,B,C) < 10) {
				/*
				if (_prevTask == 1) _task1 = true;
				else if (_prevTask == 2) _task2 = true;
				else if (_prevTask == 3) _task3 = true;
				*/
				log ("STOP - NO LONGER ON-ROUTE");
			}
		}
	}
}

void GUI::displayImage(IplImage * webcamFeed, QLabel * location, int type)
{
	if (type == 0) cvCvtColor(webcamFeed,_image,CV_BGR2RGB);
	else if (type == 1) cvCvtColor(webcamFeed, _image, CV_GRAY2RGB);
	else return;

	QImage qimage = QImage((uchar*)_image->imageData, _image->width, _image->height, QImage::Format_RGB888);
	location->resize(_image->width, _image->height);
	location->setPixmap(QPixmap::fromImage(qimage));
}

void GUI::displayFinal(IplImage * webcamFeed, QLabel * location)
{
	cvCvtColor(webcamFeed,_topImage,CV_BGR2RGB);
	QImage qimage = QImage((uchar*)_topImage->imageData, _topImage->width, _topImage->height, QImage::Format_RGB888);
	location->resize(_topImage->width, _topImage->height);
	location->setPixmap(QPixmap::fromImage(qimage));
}

void GUI::displayMain()
{
	// Draw on Main
	QImage qimage = QImage(FINAL_WIDTH, FINAL_HEIGHT, QImage::Format_RGB888);
	qimage.fill(QColor(Qt::white));
	QPainter p;
	p.begin(&qimage);
	p.setPen(QPen(QColor(Qt::gray), 3));
	for (int i = 1; i < 8; i++) {
		p.drawLine(QLine(0, FINAL_HEIGHT*i/8.0, FINAL_WIDTH, FINAL_HEIGHT*i/8.0));
		p.drawLine(QLine(FINAL_WIDTH*i/8.0, 0, FINAL_WIDTH*i/8.0, FINAL_HEIGHT));
	}
	p.setPen(QPen(QColor(Qt::gray), 1));
	for (double i = 0.5; i < 8; i++) {
		p.drawLine(QLine(0, FINAL_HEIGHT*i/8.0, FINAL_WIDTH, FINAL_HEIGHT*i/8.0));
		p.drawLine(QLine(FINAL_WIDTH*i/8.0, 0, FINAL_WIDTH*i/8.0, FINAL_HEIGHT));
	}
	p.setPen(QPen(QColor(Qt::black),10));
	p.drawRect(QRect(0,0,FINAL_WIDTH,FINAL_HEIGHT));
	p.setPen(QPen(QColor(Qt::black),5));
	p.drawRect(QRect(FINAL_WIDTH*2.5/8.0,0,FINAL_WIDTH*3.0/8.0,FINAL_HEIGHT/8.0));
	p.drawRect(QRect(FINAL_WIDTH*2.5/8.0,FINAL_HEIGHT*7.0/8.0,FINAL_WIDTH*3.0/8.0,FINAL_HEIGHT/8.0));
	p.drawLine(QLine(0, FINAL_HEIGHT/2.0, FINAL_WIDTH, FINAL_HEIGHT/2.0));


	// Draw Balls
	for (int i = 0; i < _balls.size(); i++) {
		p.setPen(QPen(QColor(Qt::green),5));
		p.drawArc(_balls[i].x-_ballRadius, _balls[i].y-_ballRadius, _ballRadius*2, _ballRadius*2, 0, 16*360);
	}

	// Draw Robot
	if (_robot.size() > 0) {
		p.setPen(QPen(QColor(Qt::blue),5));
		p.drawArc(_robot[1].x-_robotRadius, _robot[1].y-_robotRadius, _robotRadius*2, _robotRadius*2, 0, 16*360);
		p.setPen(QPen(QColor(Qt::darkBlue),5));
		p.drawLine(_robot[1].x, _robot[1].y, _robot[1].x + cos(_robotAngle)*20, _robot[1].y - sin(_robotAngle)*20);
	}

	// Draw Obstacles
	for (int i = 0; i < _obstacles.size(); i++) {
		p.setPen(QPen(QColor(Qt::red),5));
		p.drawArc(_obstacles[i].x-_obstacleRadius, _obstacles[i].y-_obstacleRadius, _obstacleRadius*2, _obstacleRadius*2, 0, 16*360);
	}

	// Draw Pathing
	QString pathMessage = "Path: ";
	if (_path.size() > 0) {
		pathMessage += "(" + QString::number((int)_path[0].x*2) + "," + QString::number((int)(FINAL_HEIGHT - _path[0].y)*2) + ")";
	}
	for (int i = 0; i < _path.size()-1 && _path.size() > 0; i++) {
		p.setPen(QPen(QColor(Qt::yellow), 5));
		pathMessage += "(" + QString::number((int)_path[i+1].x*2) + "," + QString::number((int)(FINAL_HEIGHT - _path[i+1].y)*2) + ")";
		p.drawLine(_path[i].x, _path[i].y, _path[i+1].x, _path[i+1].y);
		p.setPen(QPen(QColor(Qt::darkYellow), 5));
		p.drawArc(_path[i].x-1, _path[i].y-1, 2, 2, 0, 16*360);
		p.drawArc(_path[i+1].x-1, _path[i+1].y-1, 2, 2, 0, 16*360);
	}
	p.end();
	ui.labelPaths->setText(pathMessage);

	ui.topView->resize(FINAL_WIDTH, FINAL_HEIGHT);
	ui.topView->setPixmap(QPixmap::fromImage(qimage));
}

void GUI::on_leftCalibrate_triggered()
{
	log("LEFT CAMERA - Calibrating ...");
	_progressBar->setValue(0);
	for (int i = 0; i < calibrationSize; i++) {
		_cam1->calibrate(i);
		display();
		_progressBar->setValue((i+1)*100/calibrationSize);
	}
	_cam1->finishCalibrate();
	log("LEFT CAMERA - Calibrating ... COMPLETE");
}

void GUI::on_leftObstacles_triggered()
{
	log("LEFT CAMERA - Getting Obstacles ...");
	_cam1->calculateObstacles();
	log("LEFT CAMERA - Getting Obstacles ... COMPLETE");
}

void GUI::on_leftReset_triggered()
{
	log("LEFT CAMERA - Resetting ...");
	_cam1->resetCalibrate();
	log("LEFT CAMERA - Resetting ... COMPLETE");
}

void GUI::on_rightCalibrate_triggered()
{
	log("RIGHT CAMERA - Calibrating ...");
	_progressBar->setValue(0);
	for (int i = 0; i < calibrationSize; i++) {
		_cam2->calibrate(i);
		display();
		_progressBar->setValue((i+1)*100/calibrationSize);
	}
	_cam2->finishCalibrate();
	log("RIGHT CAMERA - Calibrating ... COMPLETE");
}

void GUI::on_rightObstacles_triggered()
{
	log("RIGHT CAMERA - Getting Obstacles ...");
	_cam2->calculateObstacles();
	log("RIGHT CAMERA - Getting Obstacles ... COMPLETE");
}

void GUI::on_rightReset_triggered()
{
	log("RIGHT CAMERA - Resetting ...");
	_cam2->resetCalibrate();
	log("RIGHT CAMERA - Resetting ... COMPLETE");
}

void GUI::readLeftThreshold()
{
	ui.spinLeftHueMin->blockSignals(true);
	ui.spinLeftHueMax->blockSignals(true);
	ui.spinLeftSatMin->blockSignals(true);
	ui.spinLeftSatMax->blockSignals(true);
	ui.spinLeftValMin->blockSignals(true);
	ui.spinLeftValMax->blockSignals(true);
	ui.spinLeftAreaMin->blockSignals(true);
	ui.spinLeftAreaMax->blockSignals(true);
	if (ui.leftArenaRadio->isChecked()) {
		ui.spinLeftHueMin->setValue(_cam1->_arenaHmin);
		ui.spinLeftHueMax->setValue(_cam1->_arenaHmax);
		ui.spinLeftSatMin->setValue(_cam1->_arenaSmin);
		ui.spinLeftSatMax->setValue(_cam1->_arenaSmax);
		ui.spinLeftValMin->setValue(_cam1->_arenaVmin);
		ui.spinLeftValMax->setValue(_cam1->_arenaVmax);
		ui.spinLeftAreaMin->setValue(_cam1->_arenaAmin);
		ui.spinLeftAreaMax->setValue(_cam1->_arenaAmax);
	} else if (ui.leftBallsRadio->isChecked()) {
		ui.spinLeftHueMin->setValue(_cam1->_ballHmin);
		ui.spinLeftHueMax->setValue(_cam1->_ballHmax);
		ui.spinLeftSatMin->setValue(_cam1->_ballSmin);
		ui.spinLeftSatMax->setValue(_cam1->_ballSmax);
		ui.spinLeftValMin->setValue(_cam1->_ballVmin);
		ui.spinLeftValMax->setValue(_cam1->_ballVmax);
		ui.spinLeftAreaMin->setValue(_cam1->_ballAmin);
		ui.spinLeftAreaMax->setValue(_cam1->_ballAmax);
	} else if (ui.leftObstacles1Radio->isChecked()) {
		ui.spinLeftHueMin->setValue(_cam1->_obstacles1Hmin);
		ui.spinLeftHueMax->setValue(_cam1->_obstacles1Hmax);
		ui.spinLeftSatMin->setValue(_cam1->_obstacles1Smin);
		ui.spinLeftSatMax->setValue(_cam1->_obstacles1Smax);
		ui.spinLeftValMin->setValue(_cam1->_obstacles1Vmin);
		ui.spinLeftValMax->setValue(_cam1->_obstacles1Vmax);
		ui.spinLeftAreaMin->setValue(_cam1->_obstacles1Amin);
		ui.spinLeftAreaMax->setValue(_cam1->_obstacles1Amax);
	} else if (ui.leftObstacles2Radio->isChecked()) {
		ui.spinLeftHueMin->setValue(_cam1->_obstacles2Hmin);
		ui.spinLeftHueMax->setValue(_cam1->_obstacles2Hmax);
		ui.spinLeftSatMin->setValue(_cam1->_obstacles2Smin);
		ui.spinLeftSatMax->setValue(_cam1->_obstacles2Smax);
		ui.spinLeftValMin->setValue(_cam1->_obstacles2Vmin);
		ui.spinLeftValMax->setValue(_cam1->_obstacles2Vmax);
		ui.spinLeftAreaMin->setValue(_cam1->_obstacles2Amin);
		ui.spinLeftAreaMax->setValue(_cam1->_obstacles2Amax);
	} else if (ui.leftRobot1Radio->isChecked()) {
		ui.spinLeftHueMin->setValue(_cam1->_robot1Hmin);
		ui.spinLeftHueMax->setValue(_cam1->_robot1Hmax);
		ui.spinLeftSatMin->setValue(_cam1->_robot1Smin);
		ui.spinLeftSatMax->setValue(_cam1->_robot1Smax);
		ui.spinLeftValMin->setValue(_cam1->_robot1Vmin);
		ui.spinLeftValMax->setValue(_cam1->_robot1Vmax);
		ui.spinLeftAreaMin->setValue(_cam1->_robot1Amin);
		ui.spinLeftAreaMax->setValue(_cam1->_robot1Amax);
	} else if (ui.leftRobot2Radio->isChecked()) {
		ui.spinLeftHueMin->setValue(_cam1->_robot2Hmin);
		ui.spinLeftHueMax->setValue(_cam1->_robot2Hmax);
		ui.spinLeftSatMin->setValue(_cam1->_robot2Smin);
		ui.spinLeftSatMax->setValue(_cam1->_robot2Smax);
		ui.spinLeftValMin->setValue(_cam1->_robot2Vmin);
		ui.spinLeftValMax->setValue(_cam1->_robot2Vmax);
		ui.spinLeftAreaMin->setValue(_cam1->_robot2Amin);
		ui.spinLeftAreaMax->setValue(_cam1->_robot2Amax);
	}
	ui.spinLeftHueMin->blockSignals(false);
	ui.spinLeftHueMax->blockSignals(false);
	ui.spinLeftSatMin->blockSignals(false);
	ui.spinLeftSatMax->blockSignals(false);
	ui.spinLeftValMin->blockSignals(false);
	ui.spinLeftValMax->blockSignals(false);
	ui.spinLeftAreaMin->blockSignals(false);
	ui.spinLeftAreaMax->blockSignals(false);
}

void GUI::readRightThreshold()
{
	ui.spinRightHueMin->blockSignals(true);
	ui.spinRightHueMax->blockSignals(true);
	ui.spinRightSatMin->blockSignals(true);
	ui.spinRightSatMax->blockSignals(true);
	ui.spinRightValMin->blockSignals(true);
	ui.spinRightValMax->blockSignals(true);
	ui.spinRightAreaMin->blockSignals(true);
	ui.spinRightAreaMax->blockSignals(true);
	if (ui.rightArenaRadio->isChecked()) {
		ui.spinRightHueMin->setValue(_cam2->_arenaHmin);
		ui.spinRightHueMax->setValue(_cam2->_arenaHmax);
		ui.spinRightSatMin->setValue(_cam2->_arenaSmin);
		ui.spinRightSatMax->setValue(_cam2->_arenaSmax);
		ui.spinRightValMin->setValue(_cam2->_arenaVmin);
		ui.spinRightValMax->setValue(_cam2->_arenaVmax);
		ui.spinRightAreaMin->setValue(_cam2->_arenaAmin);
		ui.spinRightAreaMax->setValue(_cam2->_arenaAmax);
	} else if (ui.rightBallsRadio->isChecked()) {
		ui.spinRightHueMin->setValue(_cam2->_ballHmin);
		ui.spinRightHueMax->setValue(_cam2->_ballHmax);
		ui.spinRightSatMin->setValue(_cam2->_ballSmin);
		ui.spinRightSatMax->setValue(_cam2->_ballSmax);
		ui.spinRightValMin->setValue(_cam2->_ballVmin);
		ui.spinRightValMax->setValue(_cam2->_ballVmax);
		ui.spinRightAreaMin->setValue(_cam2->_ballAmin);
		ui.spinRightAreaMax->setValue(_cam2->_ballAmax);
	} else if (ui.rightObstacles1Radio->isChecked()) {
		ui.spinRightHueMin->setValue(_cam2->_obstacles1Hmin);
		ui.spinRightHueMax->setValue(_cam2->_obstacles1Hmax);
		ui.spinRightSatMin->setValue(_cam2->_obstacles1Smin);
		ui.spinRightSatMax->setValue(_cam2->_obstacles1Smax);
		ui.spinRightValMin->setValue(_cam2->_obstacles1Vmin);
		ui.spinRightValMax->setValue(_cam2->_obstacles1Vmax);
		ui.spinRightAreaMin->setValue(_cam2->_obstacles1Amin);
		ui.spinRightAreaMax->setValue(_cam2->_obstacles1Amax);
	} else if (ui.rightObstacles2Radio->isChecked()) {
		ui.spinRightHueMin->setValue(_cam2->_obstacles2Hmin);
		ui.spinRightHueMax->setValue(_cam2->_obstacles2Hmax);
		ui.spinRightSatMin->setValue(_cam2->_obstacles2Smin);
		ui.spinRightSatMax->setValue(_cam2->_obstacles2Smax);
		ui.spinRightValMin->setValue(_cam2->_obstacles2Vmin);
		ui.spinRightValMax->setValue(_cam2->_obstacles2Vmax);
		ui.spinRightAreaMin->setValue(_cam2->_obstacles2Amin);
		ui.spinRightAreaMax->setValue(_cam2->_obstacles2Amax);
	} else if (ui.rightRobot1Radio->isChecked()) {
		ui.spinRightHueMin->setValue(_cam2->_robot1Hmin);
		ui.spinRightHueMax->setValue(_cam2->_robot1Hmax);
		ui.spinRightSatMin->setValue(_cam2->_robot1Smin);
		ui.spinRightSatMax->setValue(_cam2->_robot1Smax);
		ui.spinRightValMin->setValue(_cam2->_robot1Vmin);
		ui.spinRightValMax->setValue(_cam2->_robot1Vmax);
		ui.spinRightAreaMin->setValue(_cam2->_robot1Amin);
		ui.spinRightAreaMax->setValue(_cam2->_robot1Amax);
	} else if (ui.rightRobot2Radio->isChecked()) {
		ui.spinRightHueMin->setValue(_cam2->_robot2Hmin);
		ui.spinRightHueMax->setValue(_cam2->_robot2Hmax);
		ui.spinRightSatMin->setValue(_cam2->_robot2Smin);
		ui.spinRightSatMax->setValue(_cam2->_robot2Smax);
		ui.spinRightValMin->setValue(_cam2->_robot2Vmin);
		ui.spinRightValMax->setValue(_cam2->_robot2Vmax);
		ui.spinRightAreaMin->setValue(_cam2->_robot2Amin);
		ui.spinRightAreaMax->setValue(_cam2->_robot2Amax);
	}
	ui.spinRightHueMin->blockSignals(false);
	ui.spinRightHueMax->blockSignals(false);
	ui.spinRightSatMin->blockSignals(false);
	ui.spinRightSatMax->blockSignals(false);
	ui.spinRightValMin->blockSignals(false);
	ui.spinRightValMax->blockSignals(false);
	ui.spinRightAreaMin->blockSignals(false);
	ui.spinRightAreaMax->blockSignals(false);
}

void GUI::writeLeftThreshold()
{
	if (ui.leftArenaRadio->isChecked()) {
		_cam1->_arenaHmin = ui.spinLeftHueMin->value();
		_cam1->_arenaHmax = ui.spinLeftHueMax->value();
		_cam1->_arenaSmin = ui.spinLeftSatMin->value();
		_cam1->_arenaSmax = ui.spinLeftSatMax->value();
		_cam1->_arenaVmin = ui.spinLeftValMin->value();
		_cam1->_arenaVmax = ui.spinLeftValMax->value();
		_cam1->_arenaAmin = ui.spinLeftAreaMin->value();
		_cam1->_arenaAmax = ui.spinLeftAreaMax->value();
	} else if (ui.leftBallsRadio->isChecked()) {
		_cam1->_ballHmin = ui.spinLeftHueMin->value();
		_cam1->_ballHmax = ui.spinLeftHueMax->value();
		_cam1->_ballSmin = ui.spinLeftSatMin->value();
		_cam1->_ballSmax = ui.spinLeftSatMax->value();
		_cam1->_ballVmin = ui.spinLeftValMin->value();
		_cam1->_ballVmax = ui.spinLeftValMax->value();
		_cam1->_ballAmin = ui.spinLeftAreaMin->value();
		_cam1->_ballAmax = ui.spinLeftAreaMax->value();
	} else if (ui.leftObstacles1Radio->isChecked()) {
		_cam1->_obstacles1Hmin = ui.spinLeftHueMin->value();
		_cam1->_obstacles1Hmax = ui.spinLeftHueMax->value();
		_cam1->_obstacles1Smin = ui.spinLeftSatMin->value();
		_cam1->_obstacles1Smax = ui.spinLeftSatMax->value();
		_cam1->_obstacles1Vmin = ui.spinLeftValMin->value();
		_cam1->_obstacles1Vmax = ui.spinLeftValMax->value();
		_cam1->_obstacles1Amin = ui.spinLeftAreaMin->value();
		_cam1->_obstacles1Amax = ui.spinLeftAreaMax->value();
	} else if (ui.leftObstacles2Radio->isChecked()) {
		_cam1->_obstacles2Hmin = ui.spinLeftHueMin->value();
		_cam1->_obstacles2Hmax = ui.spinLeftHueMax->value();
		_cam1->_obstacles2Smin = ui.spinLeftSatMin->value();
		_cam1->_obstacles2Smax = ui.spinLeftSatMax->value();
		_cam1->_obstacles2Vmin = ui.spinLeftValMin->value();
		_cam1->_obstacles2Vmax = ui.spinLeftValMax->value();
		_cam1->_obstacles2Amin = ui.spinLeftAreaMin->value();
		_cam1->_obstacles2Amax = ui.spinLeftAreaMax->value();
	} else if (ui.leftRobot1Radio->isChecked()) {
		_cam1->_robot1Hmin = ui.spinLeftHueMin->value();
		_cam1->_robot1Hmax = ui.spinLeftHueMax->value();
		_cam1->_robot1Smin = ui.spinLeftSatMin->value();
		_cam1->_robot1Smax = ui.spinLeftSatMax->value();
		_cam1->_robot1Vmin = ui.spinLeftValMin->value();
		_cam1->_robot1Vmax = ui.spinLeftValMax->value();
		_cam1->_robot1Amin = ui.spinLeftAreaMin->value();
		_cam1->_robot1Amax = ui.spinLeftAreaMax->value();
	} else if (ui.leftRobot2Radio->isChecked()) {
		_cam1->_robot2Hmin = ui.spinLeftHueMin->value();
		_cam1->_robot2Hmax = ui.spinLeftHueMax->value();
		_cam1->_robot2Smin = ui.spinLeftSatMin->value();
		_cam1->_robot2Smax = ui.spinLeftSatMax->value();
		_cam1->_robot2Vmin = ui.spinLeftValMin->value();
		_cam1->_robot2Vmax = ui.spinLeftValMax->value();
		_cam1->_robot2Amin = ui.spinLeftAreaMin->value();
		_cam1->_robot2Amax = ui.spinLeftAreaMax->value();
	}
}

void GUI::writeRightThreshold()
{
	if (ui.rightArenaRadio->isChecked()) {
		_cam2->_arenaHmin = ui.spinRightHueMin->value();
		_cam2->_arenaHmax = ui.spinRightHueMax->value();
		_cam2->_arenaSmin = ui.spinRightSatMin->value();
		_cam2->_arenaSmax = ui.spinRightSatMax->value();
		_cam2->_arenaVmin = ui.spinRightValMin->value();
		_cam2->_arenaVmax = ui.spinRightValMax->value();
		_cam2->_arenaAmin = ui.spinRightAreaMin->value();
		_cam2->_arenaAmax = ui.spinRightAreaMax->value();
	} else if (ui.rightBallsRadio->isChecked()) {
		_cam2->_ballHmin = ui.spinRightHueMin->value();
		_cam2->_ballHmax = ui.spinRightHueMax->value();
		_cam2->_ballSmin = ui.spinRightSatMin->value();
		_cam2->_ballSmax = ui.spinRightSatMax->value();
		_cam2->_ballVmin = ui.spinRightValMin->value();
		_cam2->_ballVmax = ui.spinRightValMax->value();
		_cam2->_ballAmin = ui.spinRightAreaMin->value();
		_cam2->_ballAmax = ui.spinRightAreaMax->value();
	} else if (ui.rightObstacles1Radio->isChecked()) {
		_cam2->_obstacles1Hmin = ui.spinRightHueMin->value();
		_cam2->_obstacles1Hmax = ui.spinRightHueMax->value();
		_cam2->_obstacles1Smin = ui.spinRightSatMin->value();
		_cam2->_obstacles1Smax = ui.spinRightSatMax->value();
		_cam2->_obstacles1Vmin = ui.spinRightValMin->value();
		_cam2->_obstacles1Vmax = ui.spinRightValMax->value();
		_cam2->_obstacles1Amin = ui.spinRightAreaMin->value();
		_cam2->_obstacles1Amax = ui.spinRightAreaMax->value();
	} else if (ui.rightObstacles2Radio->isChecked()) {
		_cam2->_obstacles2Hmin = ui.spinRightHueMin->value();
		_cam2->_obstacles2Hmax = ui.spinRightHueMax->value();
		_cam2->_obstacles2Smin = ui.spinRightSatMin->value();
		_cam2->_obstacles2Smax = ui.spinRightSatMax->value();
		_cam2->_obstacles2Vmin = ui.spinRightValMin->value();
		_cam2->_obstacles2Vmax = ui.spinRightValMax->value();
		_cam2->_obstacles2Amin = ui.spinRightAreaMin->value();
		_cam2->_obstacles2Amax = ui.spinRightAreaMax->value();
	} else if (ui.rightRobot1Radio->isChecked()) {
		_cam2->_robot1Hmin = ui.spinRightHueMin->value();
		_cam2->_robot1Hmax = ui.spinRightHueMax->value();
		_cam2->_robot1Smin = ui.spinRightSatMin->value();
		_cam2->_robot1Smax = ui.spinRightSatMax->value();
		_cam2->_robot1Vmin = ui.spinRightValMin->value();
		_cam2->_robot1Vmax = ui.spinRightValMax->value();
		_cam2->_robot1Amin = ui.spinRightAreaMin->value();
		_cam2->_robot1Amax = ui.spinRightAreaMax->value();
	} else if (ui.rightRobot2Radio->isChecked()) {
		_cam2->_robot2Hmin = ui.spinRightHueMin->value();
		_cam2->_robot2Hmax = ui.spinRightHueMax->value();
		_cam2->_robot2Smin = ui.spinRightSatMin->value();
		_cam2->_robot2Smax = ui.spinRightSatMax->value();
		_cam2->_robot2Vmin = ui.spinRightValMin->value();
		_cam2->_robot2Vmax = ui.spinRightValMax->value();
		_cam2->_robot2Amin = ui.spinRightAreaMin->value();
		_cam2->_robot2Amax = ui.spinRightAreaMax->value();
	}
}

void GUI::log(QString text)
{
	_progressLabel->setText(text);
}