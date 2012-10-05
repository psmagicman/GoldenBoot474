#include <QApplication>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <assert.h>

#include "camerawidget.h"
#include "camerawindow.h"

int main(int argc, char **argv) {
    CvCapture *camera = cvCreateCameraCapture(0);
    assert(camera);
    IplImage *image = cvQueryFrame(camera);
    assert(image);

    QApplication app(argc, argv);

    CameraWindow *window = new CameraWindow(camera);
    window->setWindowTitle("Qt + OpenCV example");
    window->show();

    int retval = app.exec();
    
    cvReleaseCapture(&camera);
    
    return retval;
}

