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

    bool is_pub          = true;
    bool is_show         = false;
    bool is_print        = true;
    int serialNum        = 17221121;
    bool is_auto_shutter = false;
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
    nh.getParam( "brightness", brightness );
    nh.getParam( "exposure", exposure );
    nh.getParam( "gain", gain );
    nh.getParam( "frameRate", frameRate );
    nh.getParam( "shutter", shutter );

    ros::Publisher imagePublisher = nh.advertise< sensor_msgs::Image >( "/image_out", 3 );

    unsigned int cameraId = serialNum;

    if ( is_show )
        cv::namedWindow( "image", CV_WINDOW_NORMAL );

    camReader.startCamera( cameraId, frameRate, brightness, exposure, gain,
                           is_auto_shutter, shutter, is_print );

    int imageCnt = 0;
    while ( ros::ok( ) )
    {
        cv::Mat cv_image = camReader.grabImage( );
        if ( cv_image.empty( ) )
        {
            std::cout << "[#INFO] Grabbed no image." << std::endl;
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
                if ( camReader.Camera( ).isColorCamera( ) )
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

    camReader.stopCamera( );

    cout << "Done! Press Enter to exit..." << endl;

    return 0;
}
