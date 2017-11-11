#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

//#include <code_utils/sys_utils.h>
#include "ptgrey_lib/singleCameraReader.h"
#include <cv_bridge/cv_bridge.h>
#include <flycapture/FlyCapture2.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sstream>

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "singleReader" );
    ros::NodeHandle nh( "~" );

    bool is_pub          = true;
    bool is_show         = false;
    bool is_print        = true;
    int serialNum        = 17221121;
    bool is_auto_shutter = false;
    bool is_sync         = true;
    double brightness    = 0.1;
    double exposure      = 0.1;
    double gain          = 1.0;
    double frameRate     = 20.0;
    double shutter       = 5.0;

    nh.getParam( "is_pub", is_pub );
    nh.getParam( "is_show", is_show );
    nh.getParam( "is_print", is_print );
    nh.getParam( "serialNum", serialNum );
    nh.getParam( "is_auto_shutter", is_auto_shutter );
    nh.getParam( "is_sync", is_sync );
    nh.getParam( "brightness", brightness );
    nh.getParam( "exposure", exposure );
    nh.getParam( "gain", gain );
    nh.getParam( "frameRate", frameRate );
    nh.getParam( "shutter", shutter );

    ros::Publisher imagePublisher = nh.advertise< sensor_msgs::Image >( "/image_out", 3 );

    unsigned int cameraId = serialNum;

    ptgrey_reader::singleCameraReader camReader( cameraId );

    if ( is_show )
        cv::namedWindow( "image", CV_WINDOW_NORMAL );

    bool is_cameraStarted
    = camReader.startCamera( cameraId, frameRate, brightness, exposure, gain, is_auto_shutter, shutter, is_print, is_sync );
    if ( !is_cameraStarted )
    {
        ros::shutdown( );
        std::cout << "[#INFO] Camera cannot start" << std::endl;
    }

    std::cout << "[#INFO] Loop start." << ros::ok( ) << std::endl;

    ros::Rate loop( frameRate );

    int imageCnt = 0;
    while ( ros::ok( ) )
    {
        ptgrey_reader::cvImage cv_image = camReader.grabImage( );
        if ( cv_image.image.empty( ) )
        {
            std::cout << "[#INFO] Grabbed no image." << std::endl;
            continue;
        }
        else
        {
            ++imageCnt;
            if ( is_print )
                std::cout << "Grabbed image " << imageCnt << std::endl;

            if ( is_pub )
            {
                cv_bridge::CvImage outImg;
                outImg.header.stamp.sec  = cv_image.time.seconds;
                outImg.header.stamp.nsec = cv_image.time.microSeconds * 1000;
                outImg.header.frame_id   = "frame";
                if ( camReader.Camera( ).isColorCamera( ) )
                    outImg.encoding = sensor_msgs::image_encodings::BGR8;
                else
                    outImg.encoding = sensor_msgs::image_encodings::MONO8;

                outImg.image = cv_image.image;
                imagePublisher.publish( outImg );
            }

            if ( is_show )
            {
                cv::imshow( "image", cv_image.image );
                cv::waitKey( 10 );
            }
        }
        loop.sleep( );
    }

    camReader.stopCamera( );

    std::cout << "[#INFO] stop Camera Done!" << std::endl;

    return 0;
}
