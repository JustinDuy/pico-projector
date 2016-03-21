#ifndef GRAYCODEDECODER_H
#define GRAYCODEDECODER_H

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>

namespace ProjectorLocalizer {
    
std::vector<cv::Mat> generatePatterns(size_t width, size_t height);
    
void estimateCameraProjectorPose(const std::vector<cv::Mat>& captured_patterns, 
                                    int pro_width, 
                                    int pro_height, 
                                    const cv::Mat& blackImage,
                                    const cv::Mat& whiteImage,
                                    const cv::Mat& Kc, 
                                    const cv::Mat& camdistCoeffs, 
                                    const cv::Mat& Kp, 
                                    cv::Mat& Ra, 
                                    cv::Mat& ua);
    


}

#endif // GRAYCODEDECODER_H
