#include <QApplication>
#include <QDebug>
#include <stdio.h>

#include "myqwidget.h"
#include <QDebug>
#include <QDesktopWidget>
#include <QWidget>
#include <QImageReader>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>
#include <iostream>
#include <stdio.h>
#include <utility.h>
#include "graycodedecoder.h"

using namespace cv;
using namespace std;
using namespace Utility;
void project(Mat img);
static const char* keys =
{ "{@path | | Path of the folder where the captured pattern images will be save }"
     "{@proj_width      | | Projector width            }"
     "{@proj_height     | | Projector height           }" };
static void help()
{
    cout << "\nThis example shows how to use the \"Structured Light module\" to acquire a graycode pattern"
         "\nCall (with the two cams connected):\n"
         "./example_structured_light_cap_pattern <path> <proj_width> <proj_height> \n"
         << endl;
}


int main( int argc, char** argv )
{
    QApplication a(argc, argv);
    structured_light::GrayCodePattern::Params params;
    //CommandLineParser parser( argc, argv, keys );
    //String path = parser.get<String>( 0 );
    //params.width = parser.get<int>( 1 );
    //params.height = parser.get<int>( 2 );
    String path = "/home/duy/pico-projector/captured/";
    params.width = 1280;
    params.height = 720;
    if( path.empty() || params.width < 1 || params.height < 1 )
    {
      help();
      return -1;
    }
    //load Kinect calibration file
    FileStorage fs( "/home/duy/pico-projector/rgb_A00363813595051A.yaml", FileStorage::READ );
    if( !fs.isOpened() )
    {
      cout << "Failed to open Calibration Data File." << endl;
      help();
      return -1;
    }
    // Loading calibration parameters
    Mat cam1intrinsics, cam1distCoeffs;

    fs["camera_matrix"] >> cam1intrinsics;

    fs["distortion_coefficients"] >> cam1distCoeffs;

    cout << "cam1intrinsics" << endl << cam1intrinsics << endl;
    cout << "cam1distCoeffs" << endl << cam1distCoeffs << endl;

    if((!cam1intrinsics.data) || (!cam1distCoeffs.data) )
    {
      cout << "Failed to load cameras calibration parameters" << endl;
      help();
      return -1;
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
    pattern.push_back( white );
    pattern.push_back( black );

    namedWindow( "cam1", WINDOW_NORMAL );
    //resizeWindow( "cam1", 640, 480 );
    resizeWindow("cam1", params.width, params.height);
    // Setting pattern window on second monitor (the projector's one)
    namedWindow( "Pattern Window", WINDOW_NORMAL );
    resizeWindow( "Pattern Window", params.width, params.height );
    moveWindow( "Pattern Window", params.width + 316, -20 );
    setWindowProperty( "Pattern Window", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN );

    //for captured images
    vector<Mat>  captured_patterns;
    captured_patterns.resize( numberOfPatternImages );
    Mat blackImage, whiteImage;

    // Open kinect rgb cam, using openni
    VideoCapture capture( CV_CAP_OPENNI );
    if( !capture.isOpened() )
    {
      // check if cam opened
      cout << "kinect not opened!" << endl;
      help();
      return -1;
    }
    //turn off autofocus?
    //capture.set( CAP_PROP_SETTINGS, 1 );

    int i = 0;
    while( i < (int) pattern.size() )
    {
      cout << "Waiting to save image number " << i + 1 << endl << "Press any key to acquire the photo" << endl;
      imshow( "Pattern Window", pattern[i] );
      Mat frame1;
      capture.grab();
      //capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP );
      capture.retrieve( frame1, CV_CAP_OPENNI_BGR_IMAGE );
      if( frame1.data  )
      {
        Mat tmp;
        cout << "Kinect rgb cam  size: " << Size( ( int ) capture.get( CV_CAP_PROP_FRAME_WIDTH ), ( int ) capture.get( CV_CAP_PROP_FRAME_HEIGHT ) )
             << endl;
        cout << "focus kinect rgb cam: " << capture.get( CV_CAP_PROP_OPENNI_FOCAL_LENGTH  ) << endl ;
        cout << "Press enter to save the photo or an other key to re-acquire the photo" << endl;

        // Resizing images to avoid issues for high resolution images, visualizing them as grayscale
        resize( frame1, tmp, Size( 640, 480 ) );
        cvtColor( tmp, tmp, COLOR_RGB2GRAY );
        //imshow( "cam1", tmp );
        bool save1 = false;
        int key = waitKey( 0 );
        // Pressing enter, it saves the output
        if( key == 10 )
        {
          ostringstream name;
          name << i + 1;
          save1 = imwrite( path + "pattern_cam1_im" + name.str() + ".png", frame1 );
          if( save1 )
          {
            cout << "pattern cam1 images number " << i + 1 << " saved" << endl << endl;
            if(i < pattern.size() - 2) {
                captured_patterns.push_back(frame1);
            }
            else if(i == pattern.size() -2){
                whiteImage = frame1.clone();
            }
            else if(i == pattern.size() -1){
                blackImage = frame1.clone();
            }
            i++;
          }
          else
          {
            cout << "pattern cam1 images number " << i + 1 << " NOT saved" << endl << endl << "Retry, check the path"<< endl << endl;
          }
        }
        // Pressing escape, the program closes
        if( key == 27 )
        {
          cout << "Closing program" << endl;
        }
      }
      else
      {
        cout << "No frame data, waiting for new frame" << endl;
      }
    }
    //end while loop for capturing patterns

    //decode graycode

    //loading Kinect RGB Instrinsic and Distortion coeffs: To Do
    Size imagesSize = blackImage.size();
    Mat R1, map1x, map1y;

    initUndistortRectifyMap( cam1intrinsics, cam1distCoeffs, R1, cam1intrinsics, imagesSize, CV_32FC1, map1x, map1y );
    //R1 empty -> default identity transformation

    //rectify all captured patterns before decoding
    for(int i=0; i< captured_patterns.size(); i++){
        remap( captured_patterns[i], captured_patterns[i], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
    }
    remap( blackImage, blackImage, map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
    remap( whiteImage, whiteImage, map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
    GrayCodeDecoder decoder(params.width, params.height);
    decoder.decode(captured_patterns, blackImage, whiteImage);
    // the camera will be deinitialized automatically in VideoCapture destructor

    return 0;

}

