#include "camerawindow.h"

//Constructor
CameraWindow::CameraWindow(CvCapture *cam, QWidget *parent)
    : QWidget(parent)
{
    m_camera = cam;
    m_photoCounter = 0;
    m_cvwidget = new CameraWidget(this);

    QVBoxLayout *layout = new QVBoxLayout;
    QPushButton *button = new QPushButton("Take picture");

    layout->addWidget(m_cvwidget);
    layout->addWidget(button);
    setLayout(layout);
	resize(640, 480);

    connect(button, SIGNAL(pressed()), this, SLOT(savePicture()));

    startTimer(100);  // 0.1-second timer
 }

//Puts a new frame in the widget every 100 msec
void CameraWindow::timerEvent(QTimerEvent*)
{
    IplImage *image = cvQueryFrame(m_camera);
	
	CvSize size = cvSize(640, 480);

	// filter green ball
	CvScalar hsv_min1 = cvScalar(40, 100, 100);
	CvScalar hsv_max1 = cvScalar(100, 255, 255);
	//CvScalar hsv_min2 = cvScalar(170, 50, 170);
	//CvScalar hsv_max2 = cvScalar(256, 180, 256);
	CvScalar hsv_min2 = hsv_min1;
	CvScalar hsv_max2 = hsv_max2;

	IplImage *hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3);
	IplImage *thresholded1 = cvCreateImage(size, IPL_DEPTH_8U, 1);
	IplImage *thresholded2 = cvCreateImage(size, IPL_DEPTH_8U, 1);

    m_cvwidget->putFrame(image, hsv_min1, hsv_max1, hsv_min2, hsv_max2,
							hsv_frame, thresholded1, thresholded2);
}

//Saves a new picture
void CameraWindow::savePicture(void)
{
    IplImage *image = cvQueryFrame(m_camera);

    QPixmap photo = m_cvwidget->toPixmap(image);

    if (photo.save(QString::number(m_photoCounter) + ".jpg")) {
        qDebug("Picture successfully saved!");
        m_photoCounter++;
    } else {
        qDebug("Error while saving the picture");
    }
}
