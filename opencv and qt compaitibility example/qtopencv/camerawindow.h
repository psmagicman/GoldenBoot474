#ifndef CAMERAWINDOW_H_
#define CAMERAWINDOW_H_

#include <QWidget>
#include <QVBoxLayout>
#include <QDebug>
#include <QPushButton>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "camerawidget.h"

class CameraWindow : public QWidget
{
    Q_OBJECT
public:
    CameraWindow(CvCapture *camera1, QWidget *parent=0);

private:
    CameraWidget *m_cvwidget;
    CvCapture *m_camera;

protected:
    void timerEvent(QTimerEvent*);
};

#endif
