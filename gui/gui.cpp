#include "gui.h"

GUI::GUI(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
	
	init();

	// Start Recording!
	_timer->start(50);
}

GUI::~GUI()
{
	_cam1->release();
}

void GUI::init()
{
	// Initialize Status Bar
	_progressBar = new QProgressBar;
	_progressBar->setAlignment(Qt::AlignCenter);
	_progressLabel = new QLabel;
	_progressLabel->setAlignment(Qt::AlignRight);
	log("Initializing Program...");
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
	
	// Initialize Images
	_image = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	_topImage = cvCreateImage(cvSize(FINAL_WIDTH,FINAL_HEIGHT), IPL_DEPTH_8U, 3);

	log("Program Started");
}

void GUI::display()
{
	if (_cam1->capture()) {
		if (ui.mainTab->currentIndex() == 0) {
			if (ui.leftTab->currentIndex() == 0) {
				_cam1->calculateNormal(ui.leftArenaCheck->isChecked(),ui.leftBallsCheck->isChecked(),ui.leftObstaclesCheck->isChecked(),ui.leftRobotCheck->isChecked());
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
		} else if (ui.mainTab->currentIndex() == 1) {
			displayFinal();
		}
	}
	_cam1->release();
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

void GUI::displayFinal()
{
	cvCvtColor(_cam1->getFinal(),_topImage,CV_BGR2RGB);
	QImage qimage = QImage((uchar*)_topImage->imageData, _topImage->width, _topImage->height, QImage::Format_RGB888);
	ui.topView->resize(_topImage->width, _topImage->height);
	ui.topView->setPixmap(QPixmap::fromImage(qimage));
}

void GUI::on_leftCalibrate_triggered()
{
	log("Calibrating Left Camera...");
	_progressBar->setValue(0);
	_cam1->clearCalibrate();
	for (int i = 0; i < calibrationSize; i++) {
		_cam1->calibrate(i);
		display();
		_progressBar->setValue(i*100/calibrationSize+1);
	}
	_cam1->finishCalibrate();
}

void GUI::on_leftReset_triggered()
{
	_cam1->reset();
}

void GUI::on_rightCalibrate_triggered()
{	
}

void GUI::on_rightReset_triggered()
{
}

void GUI::log(QString text)
{
	_progressLabel->setText(text);
}