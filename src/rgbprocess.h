#ifndef RGBPROCESS_H
#define RGBPROCESS_H
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace ros;
using namespace sensor_msgs;

class rgbProcess {
    NodeHandle nh;
    Subscriber sub;
    Mat raw_rgb;
public:
    rgbProcess() {
        sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1, &rgbProcess::process, this);
    }

    ~rgbProcess() {
     }

    void process( const sensor_msgs::ImageConstPtr& rgb_raw ){

    }
};


#endif // RGBPROCESS_H
