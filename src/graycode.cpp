#include <handprojector_calibration/graycode.h>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace std;
using namespace cv;

namespace ProjectorLocalizer {

// Generates the images needed for shadowMasks computation
void getImagesForShadowMasks( int _w, int _h, Mat& blackImage, Mat& whiteImage)
{
    blackImage = Mat( _h, _w, CV_8UC3, Scalar( 0, 0, 0 ) );
    whiteImage = Mat( _h, _w, CV_8UC3, Scalar( 255, 255, 255 ) );
}
const unsigned int DEFAULT_BLACK_THRESHOLD = 25;  // 3D_underworld default value 40
const unsigned int DEFAULT_WHITE_THRESHOLD = 5;   // 3D_underworld default value  5

// Computes the required number of pattern images
void computeNumberOfImages(size_t width, size_t height, size_t& numOfColImgs, size_t& numOfRowImgs, size_t& numOfPatternImages)
{
    numOfColImgs = ( size_t ) ceil( log( double( width ) ) / log( 2.0 ) );
    numOfRowImgs = ( size_t ) ceil( log( double( height ) ) / log( 2.0 ) );
    numOfPatternImages = 2 * numOfColImgs + 2 * numOfRowImgs;
}
int getPatternImageNum(int width, int height){
    size_t numOfPatterns = -1;
    size_t nCols,nRows;
    computeNumberOfImages(width, height, nRows, nCols, numOfPatterns);
    return (int) numOfPatterns;
}
vector<Mat> generatePatterns(size_t width, size_t height) {
    // The number of images of the pattern
    size_t numOfPatternImages ;
    // The number of row images of the pattern
    size_t numOfRowImgs;
    // The number of column images of the pattern
    size_t numOfColImgs;
    computeNumberOfImages(width, height, numOfColImgs, numOfRowImgs, numOfPatternImages);

    // Storage for pattern
    vector<Mat> pattern_;
    pattern_.resize( numOfPatternImages );

    for( size_t i = 0; i < numOfPatternImages; i++ )
    {
        pattern_[i] = Mat( height, width, CV_8U );
    }

    uchar flag = 0;

    for( int j = 0; j < width; j++ )  // rows loop
    {
        int rem = 0, num = j, prevRem = j % 2;

        for( size_t k = 0; k < numOfColImgs; k++ )  // images loop
        {
            num = num / 2;
            rem = num % 2;

            if( ( rem == 0 && prevRem == 1 ) || ( rem == 1 && prevRem == 0) )
            {
                flag = 1;
            }
            else
            {
                flag = 0;
            }

            for( int i = 0; i < height; i++ )  // rows loop
            {

                uchar pixel_color = ( uchar ) flag * 255;

                pattern_[2 * numOfColImgs - 2 * k - 2].at<uchar>( i, j ) = pixel_color;
                if( pixel_color > 0 )
                    pixel_color = ( uchar ) 0;
                else
                    pixel_color = ( uchar ) 255;
                pattern_[2 * numOfColImgs - 2 * k - 1].at<uchar>( i, j ) = pixel_color;  // inverse
            }

            prevRem = rem;
        }
    }

    for( int i = 0; i < height; i++ )  // rows loop
    {
        int rem = 0, num = i, prevRem = i % 2;

        for( size_t k = 0; k < numOfRowImgs; k++ )
        {
            num = num / 2;
            rem = num % 2;

            if( (rem == 0 && prevRem == 1) || (rem == 1 && prevRem == 0) )
            {
                flag = 1;
            }
            else
            {
                flag = 0;
            }

            for( int j = 0; j < width; j++ )
            {
                uchar pixel_color = ( uchar ) flag * 255;
                pattern_[2 * numOfRowImgs - 2 * k + 2 * numOfColImgs - 2].at<uchar>( i, j ) = pixel_color;

                if( pixel_color > 0 )
                    pixel_color = ( uchar ) 0;
                else
                    pixel_color = ( uchar ) 255;

                pattern_[2 * numOfRowImgs - 2 * k + 2 * numOfColImgs - 1].at<uchar>( i, j ) = pixel_color;
            }

            prevRem = rem;
        }
    }
    // Generate the all-white and all-black images needed for shadows mask computation
    Mat white;
    Mat black;
    getImagesForShadowMasks( width, height, black, white );
    pattern_.push_back ( black );
    pattern_.push_back ( white );
    return pattern_;
}
// Computes the shadows occlusion where we cannot reconstruct the model
void computeShadowMask( const Mat blackImage,const Mat whiteImage, double blackThreshold, Mat& shadowMask)
{
    //Mat grayWhite, grayBlack;
    //cvtColor(blackImage, grayBlack, CV_BGR2GRAY);
    //cvtColor(whiteImage, grayWhite, CV_BGR2GRAY);

    int cam_width = whiteImage.cols;
    int cam_height = blackImage.rows;
    shadowMask = Mat( cam_height, cam_width, CV_8U );
    for( int i = 0; i < cam_height; i++ )
    {
      for( int j = 0; j < cam_width; j++ )
      {
        double white = whiteImage.at<uchar>( i, j ) ;
        double black = blackImage.at<uchar>( i, j ) ;
        double diff = abs(white - black) ;
        //cout << diff << ", ";
        if( diff > blackThreshold )
        {
          shadowMask.at<uchar>( i, j )  = ( uchar ) 1;
        }
        else
        {
          shadowMask.at<uchar>( i, j ) = ( uchar ) 0;
        }
      }

    }
    //shadowMask = whiteImage - blackImage;
}

// Converts a gray code sequence (~ binary number) to a decimal number
int grayToDec( const vector<uchar>& gray )
{
  int dec = 0;

  uchar tmp = gray[0];

  if( tmp )
    dec += ( int ) pow( ( float ) 2, int( gray.size() - 1 ) );

  for( int i = 1; i < (int) gray.size(); i++ )
  {
    // XOR operation
    tmp = tmp ^ gray[i];
    if( tmp )
      dec += (int) pow( ( float ) 2, int( gray.size() - i - 1 ) );
  }

  return dec;
}

// For a (x,y) pixel of the camera returns the corresponding projector's pixel
bool getProjPixel( const vector<Mat>& patternImages, int x, int y, Point &projPix )
{
  vector<uchar> grayCol;
  vector<uchar> grayRow;

  bool error = false;
  int xDec, yDec;
  
  size_t width = patternImages[0].cols;
  size_t height = patternImages[0].rows;
  
  size_t numOfColImgs, numOfRowImgs, numOfPatternImages;
  computeNumberOfImages(width, height, numOfColImgs, numOfRowImgs, numOfPatternImages);
  //cout << "Num. of col images " << numOfColImgs << endl;
  // process column images
  for( size_t count = 0; count < numOfColImgs; count++ )
  {
    // get pixel intensity for regular pattern projection and its inverse
    double val1 = patternImages[count * 2].at<uchar>( Point( x, y ) );
    double val2 = patternImages[count * 2 + 1].at<uchar>( Point( x, y ) );
    double diff = abs(val1 - val2) ;
    // check if the intensity difference between the values of the normal and its inverse projection image is in a valid range
    if( diff < DEFAULT_WHITE_THRESHOLD )
      error = true;

    // determine if projection pixel is on or off
    if( val1 > val2 )
      grayCol.push_back( 1 );
    else
      grayCol.push_back( 0 );
  }

  xDec = grayToDec( grayCol );
  //cout << "Num. of row images " << numOfRowImgs << endl;
  // process row images
  for( size_t count = 0; count < numOfRowImgs; count++ )
  {
    // get pixel intensity for regular pattern projection and its inverse
    double val1 = patternImages[count * 2 + numOfColImgs * 2].at<uchar>( Point( x, y ) );
    double val2 = patternImages[count * 2 + numOfColImgs * 2 + 1].at<uchar>( Point( x, y ) );

    // check if the intensity difference between the values of the normal and its inverse projection image is in a valid range
    if( abs(val1 - val2) < DEFAULT_WHITE_THRESHOLD )
      error = true;

    // determine if projection pixel is on or off
    if( val1 > val2 )
      grayRow.push_back( 1 );
    else
      grayRow.push_back( 0 );
  }

  yDec = grayToDec( grayRow );

  if( (yDec >= height || xDec >= width) )
  {
    error = true;
  }

  projPix.x = xDec;
  projPix.y = yDec;

  return error;
}

void decode( vector<Mat> patternImages, const Mat& blackImage, const Mat& whiteImage,
                              const Mat& Kp, const Mat& Kc, Mat& R, Mat& u)
// output : camera-projector stereo matrix R, u
{
    // Computing shadows mask
    Mat shadowMask;
    computeShadowMask( blackImage, whiteImage, DEFAULT_BLACK_THRESHOLD, shadowMask );

    cvNamedWindow("Shadow Mask");
    //resizeWindow( "Shadow Mask", 640, 480 );
    imshow( "Shadow Mask", shadowMask *255 );
    waitKey(0);

    int cam_width = patternImages[0].cols;
    int cam_height = patternImages[0].rows;
    cout << "CAM WIDTH, HEIGHT = " << cam_width << "," << cam_height <<endl;
    Point projPixel;
    // Storage for the pixels of the camera that correspond to the same pixel of the projector
    vector<Point> camPixels;
    vector<Point> projPixels;

    //camPixels.resize( cam_height * cam_width );
    //for drawing:

    // then put the text itself
    char text[80];
    int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 1;
    int thickness = 1;
    int baseline=0;
    Size textSize = getTextSize(text, fontFace,
                                fontScale, thickness, &baseline);
    baseline += thickness;
    for( int i = 0; i < cam_width; i++ )
    {
        for( int j = 0; j < cam_height; j++ )
        {
          //if the pixel is not shadowed, reconstruct
          if( shadowMask.at<uchar>( j, i ) )
          {
            //for a (x,y) pixel of the camera returns the corresponding projector pixel by calculating the decimal number
            bool error = getProjPixel( patternImages, i, j, projPixel );

            if( error )
            {
              continue;
            }
            else
            {
                camPixels.push_back( Point( i, j ) );
                projPixels.push_back(projPixel);
                Point textOrg( i,j);
                //if(projPixel.x < 20 && projPixel.y < 20)
                //{
                    circle(whiteImage,Point(i ,j), 2, (255,0,0), 0);
                    //display correspondences:
                  //  sprintf(text, "(%d,%d)", projPixel.x, projPixel.y);
                    //cout << text <<",";
                  //  putText(whiteImage, string(text) , textOrg, fontFace, fontScale,
                  //      Scalar::all(255), thickness, 3);
                //}
            }
          }
        }
        //cout << endl;

    }
    cout << "Num. of Correspondences " << camPixels.size() << endl;
    //show correspondences:
    cvNamedWindow("Correspondences");
    resizeWindow( "Correspondences", 640, 480 );
    imshow( "Correspondences", whiteImage );
    waitKey(0);
    if(camPixels.size() == projPixels.size() && camPixels.size() > 9 )
    {
        Mat F = findFundamentalMat(camPixels, projPixels, FM_RANSAC, 3, 0.99);
        Mat E = Kp.t()*F*Kc;
        //Perfrom SVD on E
        SVD decomp = SVD(E);

        //U
        Mat U = decomp.u;

        //S
        Mat S(3, 3, CV_64F, Scalar(0));
        S.at<double>(0, 0) = decomp.w.at<double>(0, 0);
        S.at<double>(1, 1) = decomp.w.at<double>(0, 1);
        S.at<double>(2, 2) = decomp.w.at<double>(0, 2);

        //Vt
        Mat Vt = decomp.vt;

        //W
        Mat W(3, 3, CV_64F, Scalar(0));
        W.at<double>(0, 1) = -1;
        W.at<double>(1, 0) = 1;
        W.at<double>(2, 2) = 1;

        Mat Wt(3, 3, CV_64F, Scalar(0));
        Wt.at<double>(0, 1) = 1;
        Wt.at<double>(1, 0) = -1;
        Wt.at<double>(2, 2) = 1;

        Mat R1 = U * W * Vt;
        Mat R2 = U * Wt * Vt;
        Mat u1 = U.col(2);
        Mat t2 = -U.col(2);
        //4 candidates
        cout << "computed rotation, translation: " << endl;
        cout << R1 << "," << u1 << endl;
        R = R1;//R2
        u = u1;
    }
    else
    {
        cout << "corresspodence lists size mitmatched or not enough corresspondences" << endl;
    }
}

void estimateCameraProjectorPose(const vector<Mat>& captured_patterns, 
                                    int pro_width, 
                                    int pro_height, 
                                    const Mat& blackImage,
                                    const Mat& whiteImage,
                                    const Mat& Kc, 
                                    const Mat& camdistCoeffs, 
                                    const Mat& Kp, 
                                    Mat& Ra, 
                                    Mat& ua)
{
    //decode graycode

    //loading Kinect RGB Instrinsic and Distortion coeffs: TODO 
    //Size imagesSize = blackImage.size();
    //Mat R1, map1x, map1y;

    //initUndistortRectifyMap( Kc, camdistCoeffs, R1, Kc, imagesSize, CV_32FC1, map1x, map1y );
    //R1 empty -> default identity transformation

    //rectify all captured patterns before decoding
    //for(int i=0; i< captured_patterns.size(); i++){
    //    remap( captured_patterns[i], captured_patterns[i], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
    //}
    //remap( blackImage, blackImage, map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
    //remap( whiteImage, whiteImage, map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
    ProjectorLocalizer::decode(captured_patterns, blackImage, whiteImage, Kp, Kc, Ra, ua);

}

}
