#include "camerawidget.h"

// Constructor
CameraWidget::CameraWidget(QWidget *parent)
    : QWidget(parent)
{
    m_layout = new QVBoxLayout;
    m_imageLabel = new QLabel;

    QImage dummy(100, 100, QImage::Format_RGB32);
    m_image = dummy;

    m_layout->addWidget(m_imageLabel);

    for (int x = 0; x < 100; x ++)
        for (int y =0; y < 100; y++)
            m_image.setPixel(x,y,qRgb(x, y, y));

    m_imageLabel->setPixmap(QPixmap::fromImage(m_image));

    setLayout(m_layout);
}

CameraWidget::~CameraWidget(void){}

void CameraWidget::putFrame(IplImage *image, CvScalar hsv_min1, CvScalar hsv_max1, CvScalar hsv_min2, CvScalar hsv_max2,
								IplImage *hsv_frame, IplImage *thresholded1, IplImage *thresholded2)
{
	IplImage *frame = image;

	// convert color space to HSV
	cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
	// filter out of range colours
	cvInRangeS(hsv_frame, hsv_min1, hsv_max1, thresholded1);
	cvInRangeS(hsv_frame, hsv_min2, hsv_max2, thresholded2);
	cvOr(thresholded1, thresholded2, thresholded1);
	// memory for hough circles
	CvMemStorage *storage = cvCreateMemStorage(0);
	// use gaussian blur to smooth image
	cvSmooth(thresholded1, thresholded1, CV_GAUSSIAN, 9, 9);
	CvSeq *circles = cvHoughCircles(thresholded1, storage, CV_HOUGH_GRADIENT, 2, thresholded1->height/4, 100, 50, 10, 400);
	for(int i = 0; i < circles->total; i++) {
		float *p = (float*)cvGetSeqElem(circles, i);
		printf("Ball! x=%f, y=%f\n\r", p[0], p[1], p[2]);
		cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])), 3, CV_RGB(0, 255, 0), -1, 8, 0);
		cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])), 3, CV_RGB(255, 0, 0), 3, 8, 0);
	}

	//cvShowImage("test", thresholded1);
	cvReleaseMemStorage(&storage);

    m_imageLabel->setPixmap(toPixmap(frame));
}

QPixmap CameraWidget::toPixmap(IplImage *cvimage) {
    int cvIndex, cvLineStart;

    switch (cvimage->depth) {
        case IPL_DEPTH_8U:
            switch (cvimage->nChannels) {
                case 3:
                    if ( (cvimage->width != m_image.width()) || (cvimage->height != m_image.height()) ) {
                        QImage temp(cvimage->width, cvimage->height, QImage::Format_RGB32);
                        m_image = temp;
                    }
                    cvIndex = 0; cvLineStart = 0;
                    for (int y = 0; y < cvimage->height; y++) {
                        unsigned char red,green,blue;
                        cvIndex = cvLineStart;
                        for (int x = 0; x < cvimage->width; x++) {
                            red = cvimage->imageData[cvIndex+2];
                            green = cvimage->imageData[cvIndex+1];
                            blue = cvimage->imageData[cvIndex+0];
                            
                            m_image.setPixel(x,y,qRgb(red, green, blue));
                            cvIndex += 3;
                        }
                        cvLineStart += cvimage->widthStep;                        
                    }
                    break;
                default:
                    qWarning("This number of channels is not supported\n");
                    break;
            }
            break;
        default:
            qWarning("This type of IplImage is not implemented in QOpenCVWidget\n");
            break;
    }

    return QPixmap::fromImage(m_image);
}

