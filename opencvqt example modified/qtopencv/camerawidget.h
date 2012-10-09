#ifndef CAMERAWIDGET_H
#define CAMERAWIDGET_H

#include <QPixmap>
#include <QLabel>
#include <QWidget>
#include <QVBoxLayout>
#include <QImage>
#include <QDebug>

#include <opencv\cv.h>
#include <opencv\cxcore.h>
#include <opencv2\opencv.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>

class CameraWidget : public QWidget
{

public:
    CameraWidget(QWidget *parent = 0);
    ~CameraWidget(void);
    QPixmap toPixmap(IplImage *);
    void putFrame(IplImage *, CvScalar, CvScalar, CvScalar, CvScalar,
					IplImage *, IplImage *, IplImage *);

private:
    QLabel *m_imageLabel;
    QVBoxLayout *m_layout;
    QImage m_image;

}; 

#endif
