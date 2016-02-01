#include <QApplication>
#include <QDebug>
#include <stdio.h>

#include "myqwidget.h"
#include <QDebug>
#include <QDesktopWidget>
#include <QWidget>
#include <QImageReader>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //qDebug() << QImageReader::supportedImageFormats();
    QList<QScreen*> screens = qApp->screens();
    if(screens.length() >1)
    {
        QRect screenres = QApplication::desktop()->screenGeometry(1/*screenNumber*/);

        QPixmap* pixmap = new QPixmap("/home/duy/pico-projector/graycode.png");
        int w = screenres.width();
        int h = screenres.height();

        if(pixmap->width() != w || pixmap->height() != h){
            qDebug() << "Input Graycode has different resolution than 1024x720\n" ;
            //exit(1);
        }

        MyQWidget *secondDisplay = new MyQWidget();
        secondDisplay->setPixmap(pixmap);
        secondDisplay->move(QPoint(screenres.x(), screenres.y()));
        secondDisplay->resize(screenres.width(), screenres.height());
        secondDisplay->showFullScreen();

    }
    else {
        qDebug() << "Second display is not active\n" ;
        exit(1);
    }
    return a.exec();

}
