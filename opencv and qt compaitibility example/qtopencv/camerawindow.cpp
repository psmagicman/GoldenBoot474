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
    resize(300, 200);

    connect(button, SIGNAL(pressed()), this, SLOT(savePicture()));

    startTimer(100);  // 0.1-second timer
 }

//Puts a new frame in the widget every 100 msec
void CameraWindow::timerEvent(QTimerEvent*)
{
    IplImage *image = cvQueryFrame(m_camera);
    m_cvwidget->putFrame(image);
}

//Saves a new picture
void CameraWindow::savePicture(void)
{
    IplImage *image = cvQueryFrame(m_camera);

    QPixmap photo = m_cvwidget->toPixmap(image);

    if (photo.save("/home/rborges/Desktop/picture" + QString::number(m_photoCounter) + ".jpg")) {
        qDebug("Picture successfully saved!");
        m_photoCounter++;
    } else {
        qDebug("Error while saving the picture");
    }
}
