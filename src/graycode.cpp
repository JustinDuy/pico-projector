#include <handprojector_calibration/graycode.h>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
using namespace std;
using namespace cv;

namespace ProjectorLocalizer {
    
vector<Mat> generatePatterns(size_t width, size_t height) {
    structured_light::GrayCodePattern::Params params;

    params.width = width;
    params.height = height;
    if( params.width < 1 || params.height < 1 )
    {
      throw runtime_error("Invalid width or height");
    }

    // Set up GraycodePattern with params
    Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create( params );
    // Storage for pattern
    vector<Mat> pattern;
    graycode->generate( pattern );
    size_t numberOfPatternImages = pattern.size();
    cout << numberOfPatternImages << " pattern images + 2 images for shadows mask computation to acquire with both cameras"
           << endl;
    // Generate the all-white and all-black images needed for shadows mask computation
    Mat white;
    Mat black;
    graycode->getImagesForShadowMasks( black, white );
    pattern.push_back ( black );
    pattern.push_back ( white );
    
    return pattern;
}
    
const unsigned int DEFAULT_BLACK_THRESHOLD = 40;  // 3D_underworld default value
const unsigned int DEFAULT_WHITE_THRESHOLD = 5;   // 5 3D_underworld default value

// Computes the required number of pattern images
void computeNumberOfImages(size_t width, size_t height, size_t& numOfColImgs, size_t& numOfRowImgs, size_t& numOfPatternImages)
{
  numOfColImgs = ( size_t ) ceil( log( double( width ) ) / log( 2.0 ) );
  numOfRowImgs = ( size_t ) ceil( log( double( height ) ) / log( 2.0 ) );
  numOfPatternImages = 2 * numOfColImgs + 2 * numOfRowImgs;
}

// Computes the shadows occlusion where we cannot reconstruct the model
void computeShadowMask( const Mat blackImage,const Mat whiteImage, double blackThreshold, Mat& shadowMask)
{

    int cam_width = whiteImage.cols;
    int cam_height = whiteImage.rows;
    shadowMask = Mat( cam_height, cam_width, CV_8U );
    for( int i = 0; i < cam_width; i++ )
    {
      for( int j = 0; j < cam_height; j++ )
      {
        double white = whiteImage.at<uchar>( Point( i, j ) );
        double black = blackImage.at<uchar>( Point( i, j ) );

        if( abs(white - black) > blackThreshold )
        {
          shadowMask.at<uchar>( Point( i, j ) ) = ( uchar ) 1;
        }
        else
        {
          shadowMask.at<uchar>( Point( i, j ) ) = ( uchar ) 0;
        }
      }
    }
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

  // process column images
  for( size_t count = 0; count < numOfColImgs; count++ )
  {
    // get pixel intensity for regular pattern projection and its inverse
    double val1 = patternImages[count * 2].at<uchar>( Point( x, y ) );
    double val2 = patternImages[count * 2 + 1].at<uchar>( Point( x, y ) );

    // check if the intensity difference between the values of the normal and its inverse projection image is in a valid range
    if( abs(val1 - val2) < DEFAULT_WHITE_THRESHOLD )
      error = true;

    // determine if projection pixel is on or off
    if( val1 > val2 )
      grayCol.push_back( 1 );
    else
      grayCol.push_back( 0 );
  }

  xDec = grayToDec( grayCol );

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
    int cam_width = patternImages[0].cols;
    int cam_height = patternImages[0].rows;

    Point projPixel;
    // Storage for the pixels of the camera that correspond to the same pixel of the projector
    vector<Point> camPixels;
    vector<Point> projPixels;
    camPixels.resize( cam_height * cam_width );
    //for drawing:

    // then put the text itself
    char text[80];
    int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 2;
    int thickness = 3;
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
            else{
                camPixels.push_back( Point( i, j ) );
                projPixels.push_back(projPixel);
                Point textOrg( i,j);
                //display correspondences:
                sprintf(text, "(%d,%d)", i, j);
                putText(whiteImage, string(text) , textOrg, fontFace, fontScale,
                        Scalar::all(255), thickness, 8);
            }
          }
        }
    }
    //show correspondences:
    cvNamedWindow("Correspondences");
    resizeWindow( "Correspondences", 640, 480 );
    imshow( "Correspondences", whiteImage );
    waitKey(0);

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
    Size imagesSize = blackImage.size();
    Mat R1, map1x, map1y;

    initUndistortRectifyMap( Kc, camdistCoeffs, R1, Kc, imagesSize, CV_32FC1, map1x, map1y );
    //R1 empty -> default identity transformation

    //rectify all captured patterns before decoding
    for(int i=0; i< captured_patterns.size(); i++){
        remap( captured_patterns[i], captured_patterns[i], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
    }
    remap( blackImage, blackImage, map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
    remap( whiteImage, whiteImage, map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
    ProjectorLocalizer::decode(captured_patterns, blackImage, whiteImage, Kp, Kc, Ra, ua);

}

}
