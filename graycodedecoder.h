#ifndef GRAYCODEDECODER_H
#define GRAYCODEDECODER_H
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>
#include <stdio.h>

using namespace cv;
using namespace std;
class GrayCodeDecoder
{
public:
    GrayCodeDecoder();
    GrayCodeDecoder(int w, int h);

    //decode graycode
    bool decode( vector<Mat> patternImages,const Mat blackImage, const Mat whiteImage);// output : Transformation matrix ? to be defined

    // Sets the value for black threshold
    void setBlackThreshold( size_t val );

    // Sets the value for set the value for white threshold
    void setWhiteThreshold( size_t val );


private:
    // Parameters
    size_t width;
    size_t height;

    // The number of images of the pattern
    size_t numOfPatternImages;

    // The number of row images of the pattern
    size_t numOfRowImgs;

    // The number of column images of the pattern
    size_t numOfColImgs;

    // Number between 0-255 that represents the minimum brightness difference
    // between the fully illuminated (white) and the non - illuminated images (black)
    size_t blackThreshold;

    // Number between 0-255 that represents the minimum brightness difference
    // between the gray-code pattern and its inverse images
    size_t whiteThreshold;

    int grayToDec( const std::vector<uchar>& gray ) ;
    bool getProjPixel( InputArrayOfArrays patternImages, int x, int y, Point &projPix );
    void computeShadowMask( const Mat blackImage,const Mat whiteImage, double blackThreshold, Mat& shadowMask);
    void computeNumberOfPatternImages();

};


#endif // GRAYCODEDECODER_H
