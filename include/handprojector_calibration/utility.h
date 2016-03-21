#ifndef UTILITY_H
#define UTILITY_H

#include <QtGui/QImage>
#include <QtGui/QPixmap>
#include <QtCore/QDebug>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

namespace Utility {
    
QImage cvMatToQImage( const cv::Mat &inMat );
QPixmap cvMatToQPixmap( const cv::Mat &inMat );
  
}

#endif // UTILITY_H
