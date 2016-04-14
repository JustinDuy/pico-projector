#include <handprojector_calibration/graycode.h>

#include <QApplication>
#include <QDebug>
#include <QDesktopWidget>
#include <QWidget>
#include <QImageReader>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>

#include <iostream>
//#define ___DEBUG 1
using namespace cv;
using namespace std;
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
void loadTestPatterns(int width, int height, const char* path, vector<Mat>& captured_patterns, Mat& black_image, Mat& white_image);
String captured_path = "../captured/";

int main( int argc, char** argv )
{
    QApplication a(argc, argv);
    
    //load Kinect calibration file
    FileStorage fs( "../data/rgb_A00363813595051A.yaml", FileStorage::READ );
    if( !fs.isOpened() )
    {
      cout << "Failed to open Camera Calibration Data File." << endl;
      help();
      return -1;
    }
    FileStorage fs2( "../data/calibration.yml", FileStorage::READ);
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
      return -1;
    }
    
    const size_t CAMERA_WIDTH = 640;//1280
    const size_t CAMERA_HEIGHT = 480;//720
    vector<Mat> pattern = ProjectorLocalizer::generatePatterns(CAMERA_WIDTH, CAMERA_HEIGHT);

    //namedWindow( "cam1", WINDOW_NORMAL );
    //resizeWindow( "cam1", 640, 480 );

    // Setting pattern window on second monitor (the projector's one)
    namedWindow( "Pattern Window", WINDOW_NORMAL );
    resizeWindow( "Pattern Window", CAMERA_WIDTH*2, CAMERA_HEIGHT*2 );
    moveWindow( "Pattern Window", CAMERA_WIDTH + 1000, 50 );
    //setWindowProperty( "Pattern Window", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN );
#ifndef ___DEBUG
    // Open kinect rgb cam, using openni
    VideoCapture capture( CV_CAP_OPENNI );
    if( !capture.isOpened() )
    {
      // check if cam opened
      cout << "kinect not opened!" << endl;
      help();
      return -1;
    }
#endif
    bool known_Z = true;
    if(!known_Z){
        //Camera-base is not known
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
#ifndef ___DEBUG
            capturePatterns(capture, pattern, captured_patterns, blackImage, whiteImage);
#endif
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
    }
    else{
        //camera-base is known and assume scale factor is known
        //estimate Ra, ta: using graycode
        Mat Ra, ta;
        vector<Mat> captured_patterns;
        Mat blackImage, whiteImage;
#ifndef ___DEBUG
        capturePatterns(capture, pattern, captured_patterns, blackImage, whiteImage);
#endif

#ifdef  ___DEBUG
        loadTestPatterns(CAMERA_WIDTH,CAMERA_HEIGHT, "captured_640_480_2", captured_patterns, blackImage, whiteImage);
#endif

        ProjectorLocalizer::estimateCameraProjectorPose(captured_patterns, CAMERA_WIDTH, CAMERA_HEIGHT,
                                       blackImage, whiteImage,
                                       cam1intrinsics, cam1distCoeffs, prointrinsics, Ra, ta);
#ifndef __DEBUG
        //read in Robot Hand-Base transformation : Rb,tb
        Mat Rb, tb;
        //read in Camera-Base transformation : Rz,tz
        Mat Rz, tz;

        //compute Hand-Projector : Rx,tx
        Mat Rx = Ra.t()*(Rz*Rb);
        Mat tx = Ra.t()*(Rz*tb + tz - ta);
#endif
    }
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
void loadTestPatterns(int width, int height, const char* path, vector<Mat>& captured_patterns, Mat& black_image, Mat& white_image){
    int nPatterns = ProjectorLocalizer::getPatternImageNum(width, height);
    for(int i = 0 ; i < nPatterns + 2; i++)
    {
        char imgFile[50];
        sprintf(imgFile,"../%s/im%d.png",path,i+1);
        cout << "Reading in " << imgFile << endl;
        Mat img = imread(imgFile, IMREAD_GRAYSCALE);
        if(i < nPatterns )
            captured_patterns.push_back(img);
        else if(i == nPatterns )
        {
            black_image = img.clone();
        }
        else if(i == (nPatterns + 1))
        {
            white_image = img.clone();
        }
        //imshow("Read image ",img);
        //waitKey(100);
    }
    //waitKey();

}
void capturePatterns(VideoCapture& capture, const vector<Mat>& pattern, vector<Mat>& captured_patterns, Mat& black_image, Mat& white_image) 
{
    int i =0;
    while( i < pattern.size() )
    {
      cout << "Waiting to save image number " << i + 1 << endl ;
      imshow( "Pattern Window", pattern[i] );
      if(i==0)
          waitKey(500);
      else
          waitKey(250);

      Mat frame1;
      capture.grab();
      capture.retrieve( frame1, CV_CAP_OPENNI_BGR_IMAGE );

      if(frame1.data )
      {
        Mat gray;

        // Resizing images to avoid issues for high resolution images, visualizing them as grayscale
        resize( frame1, gray, Size( 640, 480 ) );
        cvtColor( gray, gray, COLOR_RGB2GRAY );
        bool save1 = false;

        ostringstream name;
        name << i + 1;
        save1 = imwrite( captured_path + "im" + name.str() + ".png", frame1 );
        if( save1 )
        {
            cout << "pattern camera image number " << i + 1 << " saved" << endl ;
            if(i < pattern.size() - 2)
            {
                captured_patterns.push_back(gray);
            }
            else if(i == pattern.size() -2){
            black_image = gray.clone();
        }
        else if(i == pattern.size() -1){
            white_image = gray.clone();
        }
        i++;
        }
        else
        {
            cout << "pattern cam1 images number " << i + 1 << " NOT saved" << endl << endl << "Retry, check the path"<< endl << endl;
        }
      }
      else
      {
        cout << "No frame data, waiting for new frame" << endl;
      }
    }
    //end while loop for capturing patterns
    
}


