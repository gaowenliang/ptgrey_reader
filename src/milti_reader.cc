#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
backward::SignalHandling sh;
}

//#include <code_utils/sys_utils.h>
#include "ptgrey_lib/multiCameraReader.h"
//#include "ptgrey_lib/singleCameraReader.h"
#include <cv_bridge/cv_bridge.h>
#include <flycapture/FlyCapture2.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sstream>

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "multiReader" );
    ros::NodeHandle nh( "~" );

    bool is_pub          = true;
    bool is_show         = false;
    bool is_print        = true;
    bool is_auto_shutter = false;
    bool is_sync         = false;
    double brightness    = 0.1;
    double exposure      = 0.1;
    double gain          = 1.0;
    double frameRate     = 30.0;
    double shutter       = 5.0;
    int cam_cnt          = 1;

    nh.getParam( "is_pub", is_pub );
    nh.getParam( "is_show", is_show );
    nh.getParam( "is_print", is_print );
    nh.getParam( "is_auto_shutter", is_auto_shutter );
    nh.getParam( "is_sync", is_sync );
    nh.getParam( "brightness", brightness );
    nh.getParam( "exposure", exposure );
    nh.getParam( "gain", gain );
    nh.getParam( "frameRate", frameRate );
    nh.getParam( "shutter", shutter );
    nh.getParam( "cam_cnt", cam_cnt );

    std::vector< unsigned int > IDs;
    std::vector< ros::Publisher > Publishers;
    for ( int i = 0; i < cam_cnt; i++ )
    {
        std::string prefix = boost::str( boost::format( "camera%d/" ) % i );
        int serialNum      = 17221121;
        std::string topic;

        nh.getParam( prefix + "serialNum", serialNum );
        nh.getParam( prefix + "topic", topic );

        unsigned int cameraId = serialNum;
        IDs.push_back( cameraId );

        ros::Publisher imagePublisher = nh.advertise< sensor_msgs::Image >( topic, 3 );
        Publishers.push_back( imagePublisher );
    }

    ptgrey_reader::multiCameraReader camReader( IDs );

    if ( is_show )
    {
        cv::namedWindow( "image", CV_WINDOW_NORMAL );
        cv::namedWindow( "image2", CV_WINDOW_NORMAL );
    }
    bool is_cameraStarted = camReader.startCamera( IDs, frameRate, brightness, exposure, gain,
                                                   is_auto_shutter, shutter, is_print, is_sync );

    if ( !is_cameraStarted )
    {
        ros::shutdown( );
        std::cout << "[#INFO] Camera cannot start" << std::endl;
    }

    std::cout << "[#INFO] Loop start." << ros::ok( ) << std::endl;

    int imageCnt = 0;
    while ( ros::ok( ) )
    {
        std::vector< ptgrey_reader::cvImage > images_tmp;
        images_tmp.resize( cam_cnt );
        camReader.grabImage( images_tmp );
        if ( images_tmp.at( 0 ).image.empty( ) )
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
                outImg.header.stamp.sec  = images_tmp.at( 0 ).time.seconds;
                outImg.header.stamp.nsec = images_tmp.at( 0 ).time.microSeconds * 1000;
                outImg.header.frame_id   = "frame";
                if ( camReader.Cameras( )->isColorCamera( ) )
                    outImg.encoding = sensor_msgs::image_encodings::BGR8;
                else
                    outImg.encoding = sensor_msgs::image_encodings::MONO8;

                for ( int pub_index = 0; pub_index < cam_cnt; ++pub_index )
                {
                    outImg.image = images_tmp.at( pub_index ).image;
                    Publishers.at( pub_index ).publish( outImg );
                }
            }

            if ( is_show )
            {
                cv::imshow( "image", images_tmp.at( 0 ).image );
                cv::imshow( "image2", images_tmp.at( 1 ).image );
                cv::waitKey( 10 );
            }
        }
    }

    camReader.stopCamera( );

    std::cout << "[#INFO] stop Camera Done!" << std::endl;

    return 0;
}
