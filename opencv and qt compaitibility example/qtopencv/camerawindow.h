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
    CameraWindow(CvCapture *camera, QWidget *parent=0);

private:
    CameraWidget *m_cvwidget;
    CvCapture *m_camera;
    int m_photoCounter;

protected:
    void timerEvent(QTimerEvent*);

public slots:
    void savePicture(void);

};

#endif
