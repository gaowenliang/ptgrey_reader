#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

//#include <code_utils/sys_utils.h>
#include "camerareader.h"
#include <cv_bridge/cv_bridge.h>
#include <flycapture/FlyCapture2.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sstream>

using namespace FlyCapture2;
using namespace std;

ros::Publisher imagePublisher;
bool is_show  = true;
bool is_pub   = true;
bool is_color = false;

CameraReader cameras;
singleCamera camera;

int
RunSingleCamera( )
{
    FlyCapture2::Error error;
    camera.connectCamera( error );

    camera.getCameraInfo( error );
    camera.printCameraInfo( );

    camera.getCameraConfiguration( error );

    FC2Config config;
    // Set the number of driver buffers used to 10.
    camera.camConfig( ).numBuffers = 3;
    //    config.numImageNotifications    = 0;
    //    config.minNumImageNotifications = 0;
    camera.camConfig( ).grabTimeout                   = TIMEOUT_UNSPECIFIED;
    camera.camConfig( ).highPerformanceRetrieveBuffer = true;
    camera.camConfig( ).grabMode                      = DROP_FRAMES;
    camera.setCameraConfiguration( error );
    std::cout << " camera.camConfig( ).grabTimeout  " << camera.camConfig( ).grabTimeout << " should be "
              << TIMEOUT_UNSPECIFIED << std::endl;
    camera.setMetadata( error );
    camera.setFrameRate( error, 15 );

    //    bool frameBrightness = setBrightness( error, cam, 3 );
    //    std::cout << "Brightness set is: " << frameBrightness << std::endl;

    std::cout << " Brightness is: " << camera.getBrightness( error ) << std::endl;
    std::cout << "Frame Rate is: " << camera.getFrameRate( error ) << std::endl;
    std::cout << " Sharpness is: " << camera.getSharpness( error ) << std::endl;
    std::cout << " AutoExposure is: " << camera.getAutoExposure( error ) << std::endl;
    std::cout << " WhiteBalance is: " << camera.getWhiteBalance( error ) << std::endl;
    std::cout << " getHue is: " << camera.getHue( error ) << std::endl;
    std::cout << " getSaturation is: " << camera.getSaturation( error ) << std::endl;

    std::cout << " getGamma is: " << camera.getGamma( error ) << std::endl;
    std::cout << " getIris is: " << camera.getIris( error ) << std::endl;
    std::cout << " getShutter is: " << camera.getShutter( error ) << std::endl;
    std::cout << " getGain is: " << camera.getGain( error ) << std::endl;
    std::cout << " getTriggerMode is: " << camera.getTriggerMode( error ) << std::endl;
    std::cout << " getTriggerDelay is: " << camera.getTriggerDelay( error ) << std::endl;

    camera.startCapture( error );

    int imageCnt = 0;
    while ( ros::ok( ) )
    {
        cv::Mat cv_image;
        bool getNewImage = camera.captureOneImage( error, cv_image );
        if ( !getNewImage )
        {
            cv_image.release( );
            continue;
        }
        else
        {

            ++imageCnt;
            cout << "Grabbed image " << imageCnt << endl;
            if ( is_pub )
            {
                cv_bridge::CvImage outImg;
                outImg.header.stamp    = ros::Time::now( );
                outImg.header.frame_id = "frame";
                if ( is_color )
                    outImg.encoding = sensor_msgs::image_encodings::BGR8;
                else
                    outImg.encoding = sensor_msgs::image_encodings::MONO8;
                outImg.image        = cv_image;
                imagePublisher.publish( outImg );
            }

            if ( is_show )
            {
                cv::imshow( "image", cv_image );
                cv::waitKey( 10 );
            }
            cv_image.release( );
        }
    }

    camera.StopCapture( error );

    camera.disconnectCamera( error );

    return 0;
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "pointGreyReader" );
    ros::NodeHandle nh( "~" );

    nh.getParam( "is_show", is_show );

    imagePublisher = nh.advertise< sensor_msgs::Image >( "/image_out", 3 );

    Error error;

    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras( &numCameras );

    cameras.getCameraNum( );

    if ( is_show )
        cv::namedWindow( "image", CV_WINDOW_NORMAL );

    cout << "Number of cameras detected: " << numCameras << endl;

    error = busMgr.GetCameraFromIndex( 0, &camera.UniquePGRGuid( ) );

    RunSingleCamera( );

    cout << "Done! Press Enter to exit..." << endl;

    return 0;
}
