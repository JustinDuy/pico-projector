#include <handprojector_calibration/graycode.h>
#include <handprojector_calibration/utility.h>

#include <QApplication>
#include <QDebug>
#include <QDesktopWidget>
#include <QWidget>
#include <QImageReader>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>

#include <iostream>

using namespace cv;
using namespace std;
using namespace Utility;

void project(Mat img);

static void help()
{
    cout << "\nThis example shows how to use the \"Structured Light module\" to acquire a graycode pattern"
         "\nCall (with the two cams connected):\n"
         "./example_structured_light_cap_pattern <path> <proj_width> <proj_height> \n"
         << endl;
}

void kroneckerProduct(const Mat& a, const Mat& b, Mat& product);

void composeLinearSystem(const Mat& Ra, const Mat& ua, const Mat& Rb, const Mat& tb, Mat& M, int poseNum );
void capturePatterns(VideoCapture& capture, const vector<Mat>& pattern, vector<Mat>& captured_patterns, Mat& black_image, Mat& white_image);

String captured_path = "captured/";

int main( int argc, char** argv )
{
    QApplication a(argc, argv);
    
    //load Kinect calibration file
    FileStorage fs( "data/rgb_A00363813595051A.yaml", FileStorage::READ );
    if( !fs.isOpened() )
    {
      cout << "Failed to open Camera Calibration Data File." << endl;
      help();
      return -1;
    }
    FileStorage fs2( "data/calibration.yml", FileStorage::READ);
    if( !fs2.isOpened()){
        cout << "Failed to open Projector Calibration Data File." << endl;
    }

    // Loading calibration parameters
    Mat cam1intrinsics, cam1distCoeffs;
    Mat prointrinsics;

    fs["camera_matrix"] >> cam1intrinsics;
    fs["distortion_coefficients"] >> cam1distCoeffs;
    cout << "cam1intrinsics" << endl << cam1intrinsics << endl;
    cout << "cam1distCoeffs" << endl << cam1distCoeffs << endl;

    fs2["proj_K"] >> prointrinsics;
    cout << "prointrinsics" << endl << prointrinsics << endl;
    if((!cam1intrinsics.data) || (!cam1distCoeffs.data) || (!prointrinsics.data))
    {
      cout << "Failed to load camera/projector calibration parameters" << endl;
      help();
      return -1;
    }
    
    const size_t CAMERA_WIDTH = 1280;   
    const size_t CAMERA_HEIGHT = 720;   
    vector<Mat> pattern = ProjectorLocalizer::generatePatterns(CAMERA_WIDTH, CAMERA_HEIGHT);

    namedWindow( "cam1", WINDOW_NORMAL );
    resizeWindow( "cam1", 640, 480 );
    //resizeWindow("cam1", params.width, params.height);

    // Setting pattern window on second monitor (the projector's one)
    namedWindow( "Pattern Window", WINDOW_NORMAL );
    resizeWindow( "Pattern Window", CAMERA_WIDTH, CAMERA_HEIGHT );
    moveWindow( "Pattern Window", CAMERA_WIDTH + 316, -20 );
    setWindowProperty( "Pattern Window", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN );

    // Open kinect rgb cam, using openni
    VideoCapture capture( CV_CAP_OPENNI );
    if( !capture.isOpened() )
    {
      // check if cam opened
      cout << "kinect not opened!" << endl;
      help();
      return -1;
    }

    int numOfPose = 6; // at least 3 poses are necessary
    /**********************************************************************************
     * linear system : to be updated every pose
     *
     * M x [vec(Rx);vec(Rz);tx;tz;[lamda1,lamda2,...,lamda_i]^T] = 0
     *
     * M dim = (12 * #pose) x (24 + #pose)
     *
     **********************************************************************************/
    Mat M;
    for(int pose = 0; pose < numOfPose; pose++){
        //estimate Ra, ua: using graycode
        Mat Ra, ua;
        vector<Mat> captured_patterns;
        Mat blackImage, whiteImage;
        capturePatterns(capture, pattern, captured_patterns, blackImage, whiteImage);
        ProjectorLocalizer::estimateCameraProjectorPose(captured_patterns, CAMERA_WIDTH, CAMERA_HEIGHT, blackImage, whiteImage, 
                                       cam1intrinsics, cam1distCoeffs, prointrinsics, Ra, ua);
        //read in Robot Hand-Base transformation : Rb,tb
        Mat Rb, tb;
        //update linear system to solve
        composeLinearSystem(Ra,ua,Rb,tb,M, pose+1);
    }
    //solve for R_x,t_x,R_z,t_z:
    Mat b = Mat::zeros(12*numOfPose,1,CV_64F);
    Mat out;
    solve(M,b,out,DECOMP_NORMAL);
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

void composeLinearSystem(const Mat& Ra, const Mat& ua, const Mat& Rb, const Mat& tb, Mat& M, int poseNum )
{
    int curRows = M.rows;
    int curCols = M.cols;
    Mat ret = Mat::zeros(12*poseNum, 24+poseNum, CV_64F);
    if(poseNum > 1){
        //check if previous system size qualified
        if(curRows != 12*poseNum || curCols != 24+poseNum)
            return ;
        //copy previous linear system
        M.copyTo(ret(Rect(0, 0, M.cols, M.rows)));

        //add 12x24 matrix
        int offset_y = 12 * (poseNum -1) ;
        Mat tmp ;//= Mat::zeros(9,9, CV_64F);
        kroneckerProduct(Ra, Rb, tmp);
        tmp.copyTo(ret(Rect(0,offset_y,9,9)));
        Mat I9 = Mat::eye(9,9,CV_64F);
        Mat I3 = Mat::eye(3,3,CV_64F);
        I9.copyTo(ret(Rect(9,offset_y,9,9)));
        kroneckerProduct(I3,tb.t(), tmp);
        tmp.copyTo(ret(Rect(9,9 + offset_y,9,3)));
        Mat neg_Ra = (-1) * Ra ;
        neg_Ra.copyTo(ret(Rect(18,9 + offset_y,3,3)));
        I3.copyTo(ret(Rect(21,9 + offset_y,3,3)));
        ua.copyTo(ret(Rect(24,9 + offset_y,1,3)));

        //add last col
        int y = 12*(poseNum -1) + 9;
        int x =  24 + (poseNum -1);
        ua.copyTo(ret(Rect(x,y,1,3)));
    }
    else if(poseNum == 1){
        //M = Mat::zeros(12, 25, CV_64F);
        Mat tmp ;//= Mat::zeros(9,9, CV_64F);
        kroneckerProduct(Ra, Rb, tmp);
        tmp.copyTo(ret(Rect(0,0,9,9)));
        Mat I9 = Mat::eye(9,9,CV_64F);
        Mat I3 = Mat::eye(3,3,CV_64F);
        I9.copyTo(ret(Rect(9,0,9,9)));
        kroneckerProduct(I3,tb.t(), tmp);
        tmp.copyTo(ret(Rect(9,9,9,3)));
        Mat neg_Ra = (-1) * Ra ;
        neg_Ra.copyTo(ret(Rect(18,9,3,3)));
        I3.copyTo(ret(Rect(21,9,3,3)));
        ua.copyTo(ret(Rect(24,9,1,3)));
    }

    M = ret;
    cout << M << endl;
}

void kroneckerProduct(const Mat& a, const Mat& b, Mat& product)
{
    int m = a.rows;
    int n = a.cols;

    int p = b.rows;
    int q = b.cols;
    product = Mat::zeros(m*p, n*q, CV_64F);
    for(int i = 0; i < m; i++)
    {
        for(int j = 0; j < n; i++)
        {
            //a (i,j)
            for(int r = 0; r < p ; r++)
            {
                for(int c = 0; c < q; c++)
                {
                    product.at<double>(i*p + r,j*q + c) = a.at<double>(i,j) * b.at<double> (r,c);
                }
            }
        }
    }
}

void capturePatterns(VideoCapture& capture, const vector<Mat>& pattern, vector<Mat>& captured_patterns, Mat& black_image, Mat& white_image) 
{
    //for captured images
    Mat blackImage, whiteImage;

    int i =0;
    while( i < pattern.size() )
    {
      cout << "Waiting to save image number " << i + 1 << endl << "Press any key to acquire the photo" << endl;
      imshow( "Pattern Window", pattern[i] );
      Mat frame1;
      capture.grab();
      //capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP );
      capture.retrieve( frame1, CV_CAP_OPENNI_BGR_IMAGE );

      if( frame1.data )
      {
        Mat tmp;
        cout << "Kinect rgb cam  size: " << Size( ( int ) capture.get( CV_CAP_PROP_FRAME_WIDTH ), ( int ) capture.get( CV_CAP_PROP_FRAME_HEIGHT ) )
             << endl;
        cout << "focus kinect rgb cam: " << capture.get( CV_CAP_PROP_OPENNI_FOCAL_LENGTH  ) << endl ;
        cout << "Press enter to save the photo or an other key to re-acquire the photo" << endl;

        // Resizing images to avoid issues for high resolution images, visualizing them as grayscale
        resize( frame1, tmp, Size( 640, 480 ) );
        cvtColor( tmp, tmp, COLOR_RGB2GRAY );
        imshow( "cam1", tmp );
        bool save1 = false;


        int key = waitKey( 0 );
        // Pressing enter, it saves the output
        if( key == 10 )
        {
          ostringstream name;
          name << i + 1;
          save1 = imwrite( captured_path + "pattern_cam1_im" + name.str() + ".png", frame1 );
          if( save1 )
          {
            cout << "pattern cam1 images number " << i + 1 << " saved" << endl << endl;
            if(i < pattern.size() - 2) {
                captured_patterns.push_back(frame1);
            }
            else if(i == pattern.size() -2){
                white_image = frame1.clone();
            }
            else if(i == pattern.size() -1){
                black_image = frame1.clone();
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
    
}


