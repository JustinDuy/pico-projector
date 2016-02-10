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
    // Set up GraycodePattern with params
    Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create( params );
    // Storage for pattern
    vector<Mat> pattern;
    graycode->generate( pattern );
    cout << pattern.size() << " pattern images + 2 images for shadows mask computation to acquire with both cameras"
           << endl;
    // Generate the all-white and all-black images needed for shadows mask computation
    Mat white;
    Mat black;
    graycode->getImagesForShadowMasks( black, white );
    pattern.push_back( white );
    pattern.push_back( black );

    namedWindow( "cam1", WINDOW_NORMAL );
    resizeWindow( "cam1", 640, 480 );
    // Setting pattern window on second monitor (the projector's one)
    namedWindow( "Pattern Window", WINDOW_NORMAL );
    resizeWindow( "Pattern Window", params.width, params.height );
    moveWindow( "Pattern Window", params.width + 316, -20 );
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
        imshow( "cam1", tmp );
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
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;

}

/*
void project(Mat img)
{
    QList<QScreen*> screens = qApp->screens();
    if(screens.length() >1)
    {
        QRect screenres = QApplication::desktop()->screenGeometry(1);//screenNumber

        QPixmap pixmap = cvMatToQPixmap(img);
        int w = screenres.width();
        int h = screenres.height();

        if(pixmap.width() != w || pixmap.height() != h){
            qDebug() << "Input Graycode has different resolution than 1024x720\n" ;
            //exit(1);
        }

        MyQWidget *secondDisplay = new MyQWidget();
        secondDisplay->setPixmap(&pixmap);
        secondDisplay->move(QPoint(screenres.x(), screenres.y()));
        secondDisplay->resize(screenres.width(), screenres.height());
        secondDisplay->showFullScreen();
        cv::waitKey();
    }
    else {
        qDebug() << "Second display is not active\n" ;
        exit(1);
    }
}
*/

/*
using namespace cv;
using namespace std;


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //qDebug() << QImageReader::supportedImageFormats();
    QList<QScreen*> screens = qApp->screens();
    if(screens.length() >1)
    {
        QRect screenres = QApplication::desktop()->screenGeometry(1);//screenNumber

        QPixmap* pixmap = new QPixmap("/home/duy/pico-projector/graycode.png");
        int w = screenres.width();
        int h = screenres.height();

        if(pixmap->width() != w || pixmap->height() != h){
            qDebug() << "Input Graycode has different resolution than 1024x720\n" ;
            //exit(1);
        }

        MyQWidget *secondDisplay = new MyQWidget();
        secondDisplay->setPixmap(pixmap);
        secondDisplay->move(QPoint(screenres.x(), screenres.y()));
        secondDisplay->resize(screenres.width(), screenres.height());
        secondDisplay->showFullScreen();

    }
    else {
        qDebug() << "Second display is not active\n" ;
        exit(1);
    }
    return a.exec();

}

*/
