#include "gui.h"

GUI::GUI(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
	
	/*
	QStringList arguments;
	arguments << "-100" << "100";
	_test.execute("C:/Users/Wilbur/Desktop/GoldenBoot474/Debug/SerialTest.exe", arguments);
	_test.waitForFinished();
	_test.close();
	*/
	// Start Recording!
	init();
	_timer->start(50);
}

GUI::~GUI()
{
	_test.close();
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
	connect(ui.leftReset, SIGNAL(clicked()), this, SLOT(on_leftReset_triggered()));
	connect(ui.rightReset, SIGNAL(clicked()), this, SLOT(on_rightReset_triggered()));

	// Initialize Timer
	_timer = new QTimer(this);
	connect(_timer, SIGNAL(timeout()), this, SLOT(display()));

	// Initialize Webcams
	_cam1 = new Webcam(0);
	_cam2 = new Webcam(1);
	
	// Initialize Communication with Arduino
	//_arduino = new ArduinoCOMM("C:/Users/Wilbur/Desktop/GoldenBoot474/Debug/SerialTest.exe");
	//_arduino = new ArduinoCOMM("C:/Users/Wilbur/Desktop/GoldenBoot474/Debug/Test.exe");
	//_test.start("C:/Users/Wilbur/Desktop/GoldenBoot474/Debug/Test.exe");

	bool test = true;

	// Initialize Images
	_image = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	_topImage = cvCreateImage(cvSize(FINAL_WIDTH,FINAL_HEIGHT), IPL_DEPTH_8U, 3);

	log("MAIN PROGRAM - Initializing ... COMPLETE");
}

void GUI::display()
{
	//_arduino->write("test");
	if (ui.mainTab->currentIndex() == 0) {
		if (_cam1->capture()) {
			if (ui.leftTab->currentIndex() == 0) {
				_cam1->calculateNormal(ui.leftArenaCheck->isChecked(),ui.leftBallsCheck->isChecked(),ui.leftObstaclesCheck->isChecked(),ui.leftRobotCheck->isChecked(), true);
				displayImage(_cam1->getNormal(), ui.leftRGB);
			} else if (ui.leftTab->currentIndex() == 1) {
				displayImage(_cam1->getHSV(), ui.leftHSV);
			} else if (ui.leftTab->currentIndex() == 2) {
				int type = -1;
				if (ui.leftArenaRadio->isChecked()) type = 0;
				else if (ui.leftBallsRadio->isChecked()) type = 1;
				else if (ui.leftObstaclesRadio->isChecked()) type = 2;
				else if (ui.leftRobotRadio->isChecked()) type = 3;
				_cam1->calculateThreshold(type);
				displayImage(_cam1->getThreshold(), ui.leftThreshold, 1);
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
				int type = -1;
				if (ui.rightArenaRadio->isChecked()) type = 0;
				else if (ui.rightBallsRadio->isChecked()) type = 1;
				else if (ui.rightObstaclesRadio->isChecked()) type = 2;
				else if (ui.rightRobotRadio->isChecked()) type = 3;
				_cam2->calculateThreshold(type);
				displayImage(_cam2->getThreshold(), ui.rightThreshold, 1);
			}
			_cam2->release();
		}
	} else if (ui.mainTab->currentIndex() == 1) {
		if (_cam1->capture() && _cam1->capture()) {
			// Draw on Left
			displayFinal(_cam1->getFinal(), ui.leftFinal);
			vector<Point2f> ballsFT = _cam1->getBalls();
			vector<Point2f> robotFT = _cam1->getRobots();
			vector<Point2f> obstaclesFT = _cam1->getObstacles();
			double testX = 0;
			double testY = 0;
			if (_cam1->getBalls().size() > 0 && _cam1->getRobots().size() > 0) {
				vector<Ball> ballsCM;
				ballsCM.resize(ballsFT.size());
				Robot robotCM;
				robotCM.x = (double)robotFT[0].x / (double)FINAL_WIDTH * 8.0;
				robotCM.y = (double)robotFT[0].y / (double)FINAL_HEIGHT * 8.0;
				robotCM.angle = _cam1->getRobotAngle()/CV_PI*180;
				for (int i = 0; i < ballsFT.size(); i++) {
					ballsCM[i].x = (double)ballsFT[i].x / (double)400.0 * 8.0;
					ballsCM[i].y = (double)ballsFT[i].y / (double)FINAL_HEIGHT * 8.0;
				}
				MovementAlgorithm _algorithm = MovementAlgorithm(robotCM, ballsCM);
				testX = _algorithm.getX();
				testY = _algorithm.getY();
				vector<int> leftTicks = _algorithm.returnLeftMotor();
				vector<int> rightTicks = _algorithm.returnRightMotor();
				if (test) {
					for (int i = 0; i < leftTicks.size(); i++) {
						/*
						QByteArray data;
						data = _test.readAllStandardOutput();
						QString text = QString::fromLocal8Bit(data);
						log(text);
						_test.write("test");
						*/
						QStringList arguments;
						arguments << QString::number(leftTicks[i]) << QString::number(rightTicks[i]);
						//arguments << "-10" << "10";
						_test.execute("C:/Users/Wilbur/Desktop/GoldenBoot474/Debug/SerialTest.exe", arguments);
						_test.waitForFinished();
						_test.close();
						Sleep(1000);
						/*
						arguments.clear();
						arguments << QString::number(-10) << QString::number(10);
						_test.start("C:/Users/Wilbur/Desktop/GoldenBoot474/Debug/SerialTest.exe", arguments);
						Sleep(1000);
						_test.close();
						/*/
					}
					test = false;
				}

			}
			_balls = ballsFT;
			_robot = robotFT;
			_obstacles = obstaclesFT;
		
			// Draw on Main
			QImage qimage = QImage(FINAL_WIDTH, FINAL_HEIGHT, QImage::Format_RGB888);
			qimage.fill(QColor(Qt::white));
			QPainter p;
			p.begin(&qimage);
			p.setPen(QPen(QColor(Qt::black),10));
			p.drawRect(QRect(0,0,FINAL_WIDTH,FINAL_HEIGHT));
			p.setPen(QPen(QColor(Qt::black),5));
			p.drawRect(QRect(FINAL_WIDTH*2.5/8.0,0,FINAL_WIDTH*3.0/8.0,FINAL_HEIGHT/8.0));
			p.drawRect(QRect(FINAL_WIDTH*2.5/8.0,FINAL_HEIGHT*7.0/8.0,FINAL_WIDTH*3.0/8.0,FINAL_HEIGHT/8.0));
			p.drawLine(QLine(0, FINAL_HEIGHT/2.0, FINAL_WIDTH, FINAL_HEIGHT/2.0));
			for (int i = 0; i < _balls.size(); i++) {
				p.setPen(QPen(QColor(Qt::green),5));
				p.drawArc(_balls[i].x-_ballRadius, _balls[i].y-_ballRadius, _ballRadius*2, _ballRadius*2, 0, 16*360);
			}
			if (_robot.size() > 0) {
				p.setPen(QPen(QColor(Qt::blue),5));
				p.drawArc(_robot[0].x-_robotRadius, _robot[0].y-_robotRadius, _robotRadius*2, _robotRadius*2, 0, 16*360);
				if (_robot.size() > 1) {
					p.setPen(QPen(QColor(Qt::green),5));
					p.drawLine(_robot[0].x, _robot[0].y, _robot[1].x, _robot[1].y);
				}
			}
			for (int i = 0; i < _obstacles.size(); i++) {
				p.setPen(QPen(QColor(Qt::red),5));
				p.drawArc(_obstacles[i].x-_obstacleRadius, _obstacles[i].y-_obstacleRadius, _obstacleRadius*2, _obstacleRadius*2, 0, 16*360);
			}
			if (_robot.size() > 0 && _balls.size() > 0) {
				p.setPen(QPen(QColor(Qt::yellow),5));
				p.drawLine(_robot[0].x, _robot[0].y, testX * FINAL_WIDTH / 8, testY * FINAL_HEIGHT / 8);
			}
			p.end();
			ui.topView->resize(FINAL_WIDTH, FINAL_HEIGHT);
			ui.topView->setPixmap(QPixmap::fromImage(qimage));
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

void GUI::on_leftCalibrate_triggered()
{
	log("LEFT CAMERA - Calibrating ...");
	_progressBar->setValue(0);
	_cam1->beginCalibrate();
	for (int i = 0; i < calibrationSize; i++) {
		_cam1->calibrate(i);
		display();
		_progressBar->setValue((i+1)*100/calibrationSize);
	}
	_cam1->finishCalibrate();
	log("LEFT CAMERA - Calibrating ... COMPLETE");
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
	_cam2->beginCalibrate();
	for (int i = 0; i < calibrationSize; i++) {
		_cam2->calibrate(i);
		display();
		_progressBar->setValue((i+1)*100/calibrationSize);
	}
	_cam2->finishCalibrate();
	log("RIGHT CAMERA - Calibrating ... COMPLETE");
}

void GUI::on_rightReset_triggered()
{
	log("RIGHT CAMERA - Resetting ...");
	_cam2->resetCalibrate();
	log("RIGHT CAMERA - Resetting ... COMPLETE");
}

void GUI::log(QString text)
{
	_progressLabel->setText(text);
}