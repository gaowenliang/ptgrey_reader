//#include <code_utils/sys_utils.h>
#include "preprocess/process.h"
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

    bool is_pub              = true;
    bool is_show             = false;
    bool is_print            = true;
    bool is_grey             = false;
    int serialNum            = 17221121;
    bool is_auto_shutter     = false;
    bool is_sync             = true;
    bool is_roi              = false;
    double brightness        = 0.1;
    double exposure          = 0.1;
    double gain              = 1.0;
    double frameRate         = 20.0;
    double shutter           = 5.0;
    int WB_red               = 500;
    int WB_Blue              = 800;
    double down_sample_scale = 0.75;
    int size_x = 0, size_y = 0;
    int center_x = 0, center_y = 0;
    int cropper_x = 0, cropper_y = 0;

    nh.getParam( "is_grey", is_grey );
    nh.getParam( "is_pub", is_pub );
    nh.getParam( "is_show", is_show );
    nh.getParam( "is_print", is_print );
    nh.getParam( "serialNum", serialNum );
    nh.getParam( "is_auto_shutter", is_auto_shutter );
    nh.getParam( "is_sync", is_sync );
    nh.getParam( "is_roi", is_roi );
    nh.getParam( "brightness", brightness );
    nh.getParam( "exposure", exposure );
    nh.getParam( "gain", gain );
    nh.getParam( "frameRate", frameRate );
    nh.getParam( "shutter", shutter );
    nh.getParam( "WB_red", WB_red );
    nh.getParam( "WB_Blue", WB_Blue );
    nh.getParam( "down_sample_scale", down_sample_scale );
    nh.getParam( "size_x", size_x );
    nh.getParam( "size_y", size_y );
    nh.getParam( "center_x", center_x );
    nh.getParam( "center_y", center_y );
    nh.getParam( "cropper_x", cropper_x );
    nh.getParam( "cropper_y", cropper_y );

    std::stringstream os;
    os << serialNum;

    preprocess::PreProcess* pre;
    pre = new preprocess::PreProcess( cv::Size( size_x, size_y ),
                                      cv::Size( cropper_x, cropper_y ),
                                      cv::Point( center_x, center_y ),
                                      down_sample_scale );

    unsigned int cameraId = serialNum;

    ptgrey_reader::singleCameraReader camReader( cameraId );

    ros::Publisher imagePublisher
    = nh.advertise< sensor_msgs::Image >( "/pg_" + os.str( ) + "/image_raw", 3 );
    ros::Publisher imageROIPublisher;
    ros::Publisher imageGreyPublisher;
    ros::Publisher imageROIGreyPublisher;

    if ( is_show )
        cv::namedWindow( "image", CV_WINDOW_NORMAL );

    bool is_cameraStarted = camReader.startCamera( cameraId,
                                                   frameRate,
                                                   brightness,
                                                   exposure,
                                                   gain, //
                                                   is_auto_shutter,
                                                   shutter,
                                                   WB_red,
                                                   WB_Blue,
                                                   is_print,
                                                   is_sync );

    if ( is_roi )
        imageROIPublisher = nh.advertise< sensor_msgs::Image >( "/pg_" + os.str( ) + "/image", 3 );
    if ( is_grey && camReader.Camera( ).isColorCamera( ) )
    {
        imageGreyPublisher
        = nh.advertise< sensor_msgs::Image >( "/pg_" + os.str( ) + "/image_grey", 3 );

        if ( is_roi )
            imageROIGreyPublisher
            = nh.advertise< sensor_msgs::Image >( "/pg_" + os.str( ) + "/image_roi", 3 );
    }

    if ( !is_cameraStarted )
    {
        ros::shutdown( );
        std::cout << "[#INFO] Camera cannot start" << std::endl;
    }

    std::cout << "[#INFO] Loop start." << ros::ok( ) << std::endl;

    // ros::Rate loop( frameRate );

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
                std::cout << serialNum << " grabbed image " << imageCnt << std::endl;

            if ( is_pub )
            {
                cv_bridge::CvImage outImg;
                outImg.header.stamp.sec  = cv_image.time.seconds;
                outImg.header.stamp.nsec = cv_image.time.microSeconds * 1000;

                //    ros::Time t1 = ros::Time::now( );
                //    ros::Time t2;
                //    t2.sec  = cv_image.time.seconds;
                //    t2.nsec = cv_image.time.microSeconds * 1000;

                //    ros::Duration dt1 = t1 - t2;
                //    std::cout << "dt1 " << dt1.toSec( ) << std::endl;

                outImg.header.frame_id = "frame";
                if ( camReader.Camera( ).isColorCamera( ) )
                {
                    outImg.encoding = sensor_msgs::image_encodings::BGR8;
                }
                else
                    outImg.encoding = sensor_msgs::image_encodings::MONO8;

                outImg.image = cv_image.image;
                imagePublisher.publish( outImg );

                if ( is_roi )
                {
                    outImg.image = pre->do_preprocess( cv_image.image );
                    imageROIPublisher.publish( outImg );
                }

                if ( is_grey && camReader.Camera( ).isColorCamera( ) )
                {
                    outImg.encoding = sensor_msgs::image_encodings::MONO8;

                    cv::cvtColor( cv_image.image, outImg.image, CV_BGR2GRAY );
                    imageGreyPublisher.publish( outImg );

                    if ( is_roi )
                    {
                        outImg.image = pre->do_preprocess( outImg.image );
                        imageROIGreyPublisher.publish( outImg );
                    }
                }
            }

            if ( is_show )
            {
                cv::imshow( "image", cv_image.image );
                cv::waitKey( 10 );
            }
        }
        // loop.sleep( );
    }

    camReader.stopCamera( );

    std::cout << "[#INFO] stop Camera Done!" << std::endl;

    return 0;
}
