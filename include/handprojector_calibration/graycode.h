#ifndef GRAYCODEDECODER_H
#define GRAYCODEDECODER_H

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>

namespace GrayCode {
    
std::vector<cv::Mat> generate(size_t width, size_t height);

//decode graycode
void decode(std::vector<cv::Mat> patternImages, 
            const cv::Mat& blackImage, 
            const cv::Mat& whiteImage,
            const cv::Mat& Kp, 
            const cv::Mat& Kc, 
            cv::Mat& R, 
            cv::Mat& u);

// TODO: generate

}

#endif // GRAYCODEDECODER_H
