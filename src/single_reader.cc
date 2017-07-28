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

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "pointGreyReader" );
    ros::NodeHandle nh( "~" );

    singleCameraReader camReader;
    singleCamera camera;

    bool is_pub   = true;
    bool is_show  = true;
    bool is_print = false;
    int serialNum;
    bool is_auto_shutter = false;
    double brightness    = 0.1;
    double exposure      = 0.1;
    double gain          = 1.0;
    double frameRate     = 20;
    double shutter       = 5;

    nh.getParam( "is_show", is_show );
    nh.getParam( "serialNum", serialNum );

    ros::Publisher imagePublisher = nh.advertise< sensor_msgs::Image >( "/image_out", 3 );

    unsigned int numCameras;
    numCameras = camReader.getCameraNum( );

    if ( is_show )
        cv::namedWindow( "image", CV_WINDOW_NORMAL );

    cout << "Number of cameras detected: " << numCameras << endl;
    camReader.busMgr.GetCameraFromIndex( 0, &camReader.camera.UniquePGRGuid( ) );
    camReader.connectToCamera( );
    camReader.setCameraProperty( frameRate, brightness, exposure, gain, is_auto_shutter, shutter );

    if ( is_print )
    {
        camReader.printCameraProperty( );
    }
    camReader.startCapture( );

    int imageCnt = 0;
    while ( ros::ok( ) )
    {
        cv::Mat cv_image = camReader.grabImage( );
        if ( !getNewImage )
        {
            cv_image.release( );
            continue;
        }
        else
        {
            ++imageCnt;
            if ( is_print )
                cout << "Grabbed image " << imageCnt << endl;

            if ( is_pub )
            {
                cv_bridge::CvImage outImg;
                outImg.header.stamp    = ros::Time::now( );
                outImg.header.frame_id = "frame";
                if ( camera.isColorCamera( ) )
                    outImg.encoding = sensor_msgs::image_encodings::BGR8;
                else
                    outImg.encoding = sensor_msgs::image_encodings::MONO8;

                outImg.image = cv_image;
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

    cout << "Done! Press Enter to exit..." << endl;

    return 0;
}
