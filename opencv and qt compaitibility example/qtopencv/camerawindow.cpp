#include "camerawindow.h"

//Constructor
CameraWindow::CameraWindow(CvCapture *cam, QWidget *parent)
    : QWidget(parent)
{
    m_camera = cam;
    m_cvwidget = new CameraWidget(this);

    QVBoxLayout *layout = new QVBoxLayout;

    layout->addWidget(m_cvwidget);
    setLayout(layout);
	resize(640, 480);

    startTimer(25);  // 25ms
 }

//Puts a new frame in the widget every 25ms
void CameraWindow::timerEvent(QTimerEvent*)
{
    IplImage *image = cvQueryFrame(m_camera);

    m_cvwidget->putFrame(image);
}
