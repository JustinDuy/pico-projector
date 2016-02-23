#include "graycodedecoder.h"

GrayCodeDecoder::GrayCodeDecoder()
{

}

/*
 *  GrayCodePattern
 */

GrayCodeDecoder::GrayCodeDecoder( int _w, int _h)
{
  width = _w;
  height = _h;
  computeNumberOfPatternImages();
  blackThreshold = 40;  // 3D_underworld default value
  whiteThreshold = 5;   // 3D_underworld default value
}

// Computes the required number of pattern images
void GrayCodeDecoder::computeNumberOfPatternImages()
{
  numOfColImgs = ( size_t ) ceil( log( double( width ) ) / log( 2.0 ) );
  numOfRowImgs = ( size_t ) ceil( log( double( height ) ) / log( 2.0 ) );
  numOfPatternImages = 2 * numOfColImgs + 2 * numOfRowImgs;
}



// Computes the shadows occlusion where we cannot reconstruct the model
void GrayCodeDecoder::computeShadowMask( const Mat blackImage,const Mat whiteImage, double blackThreshold, Mat& shadowMask)
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
// For a (x,y) pixel of the camera returns the corresponding projector's pixel
bool GrayCodeDecoder::getProjPixel( InputArrayOfArrays patternImages, int x, int y, Point &projPix )
{
  std::vector<Mat>& _patternImages = *( std::vector<Mat>* ) patternImages.getObj();
  std::vector<uchar> grayCol;
  std::vector<uchar> grayRow;

  bool error = false;
  int xDec, yDec;

  // process column images
  for( size_t count = 0; count < numOfColImgs; count++ )
  {
    // get pixel intensity for regular pattern projection and its inverse
    double val1 = _patternImages[count * 2].at<uchar>( Point( x, y ) );
    double val2 = _patternImages[count * 2 + 1].at<uchar>( Point( x, y ) );

    // check if the intensity difference between the values of the normal and its inverse projection image is in a valid range
    if( abs(val1 - val2) < whiteThreshold )
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
    double val1 = _patternImages[count * 2 + numOfColImgs * 2].at<uchar>( Point( x, y ) );
    double val2 = _patternImages[count * 2 + numOfColImgs * 2 + 1].at<uchar>( Point( x, y ) );

    // check if the intensity difference between the values of the normal and its inverse projection image is in a valid range
    if( abs(val1 - val2) < whiteThreshold )
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

// Converts a gray code sequence (~ binary number) to a decimal number
int GrayCodeDecoder::grayToDec( const std::vector<uchar>& gray )
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


bool GrayCodeDecoder::decode( vector<Mat> patternImages, const Mat blackImage, const Mat whiteImage)// output : Transformation matrix ? to be defined
{
    // Computing shadows mask
    Mat shadowMask;
    computeShadowMask( blackImage, whiteImage, blackThreshold, shadowMask );
    int cam_width = patternImages[0].cols;
    int cam_height = patternImages[0].rows;

    Point projPixel;

    // Storage for the pixels of the camera that correspond to the same pixel of the projector
    std::vector<std::vector<Point> >  camPixels;
    camPixels.resize( height * width );
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

            camPixels[projPixel.x * height + projPixel.y].push_back( Point( i, j ) );
          }
        }
    }

}

