

//#include <code_utils/sys_utils.h>
#include "ptgrey_lib/multiCameraReader.h"
//#include "ptgrey_lib/singleCameraReader.h"
#include "preprocess/process.h"
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
    std::vector< ros::Publisher > imageROIPublishers;
    std::vector< preprocess::PreProcess* > pres;
    std::vector< bool > is_rois;
    for ( int i = 0; i < cam_cnt; i++ )
    {
        std::string prefix = boost::str( boost::format( "camera%d/" ) % i );
        int serialNum      = 17221121;
        std::string topic;
        std::string topic_roi;
        bool is_roi              = false;
        double down_sample_scale = 0.75;
        int size_x = 0, size_y = 0;
        int center_x = 0, center_y = 0;
        int cropper_x = 0, cropper_y = 0;

        nh.getParam( prefix + "serialNum", serialNum );
        nh.getParam( prefix + "topic", topic );
        nh.getParam( prefix + "topic_roi", topic_roi );
        nh.getParam( prefix + "is_roi", is_roi );
        nh.getParam( prefix + "down_sample_scale", down_sample_scale );
        nh.getParam( prefix + "size_x", size_x );
        nh.getParam( prefix + "size_y", size_y );
        nh.getParam( prefix + "center_x", center_x );
        nh.getParam( prefix + "center_y", center_y );
        nh.getParam( prefix + "cropper_x", cropper_x );
        nh.getParam( prefix + "cropper_y", cropper_y );

        preprocess::PreProcess* pre = new preprocess::PreProcess( cv::Size( size_x, size_y ),
                                                                  cv::Size( cropper_x, cropper_y ),
                                                                  cv::Point( center_x, center_y ),
                                                                  down_sample_scale );

        pres.push_back( pre );
        unsigned int cameraId = serialNum;
        IDs.push_back( cameraId );
        ros::Publisher imagePublisher    = nh.advertise< sensor_msgs::Image >( topic, 3 );
        ros::Publisher imageROIPublisher = nh.advertise< sensor_msgs::Image >( topic_roi, 3 );
        Publishers.push_back( imagePublisher );
        imageROIPublishers.push_back( imageROIPublisher );
        is_rois.push_back( is_roi );
    }

    ptgrey_reader::multiCameraReader camReader( IDs );

    if ( is_show )
    {
        cv::namedWindow( "image", CV_WINDOW_NORMAL );
        cv::namedWindow( "image2", CV_WINDOW_NORMAL );
    }
    bool is_cameraStarted
    = camReader.startCamera( IDs, frameRate, brightness, exposure, gain, is_auto_shutter, shutter, is_print, is_sync );

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

                ros::Time t1 = ros::Time::now( );
                ros::Time t2;
                ros::Time t3;
                t2.sec  = images_tmp.at( 0 ).time.seconds;
                t2.nsec = images_tmp.at( 0 ).time.microSeconds * 1000;
                t3.sec  = images_tmp.at( 1 ).time.seconds;
                t3.nsec = images_tmp.at( 1 ).time.microSeconds * 1000;

                ros::Duration dt1 = t1 - t2;
                ros::Duration dt2 = t1 - t3;
                ros::Duration dt3 = t3 - t2;
                std::cout << "dt1 " << dt1.toSec( ) << std::endl;
                std::cout << "dt2 " << dt2.toSec( ) << std::endl;
                std::cout << "dt3 " << dt3.toSec( ) << std::endl;

                outImg.header.frame_id = "frame";
                if ( camReader.Cameras( )->isColorCamera( ) )
                    outImg.encoding = sensor_msgs::image_encodings::BGR8;
                else
                    outImg.encoding = sensor_msgs::image_encodings::MONO8;

                for ( int pub_index = 0; pub_index < cam_cnt; ++pub_index )
                {
                    outImg.image = images_tmp.at( pub_index ).image;
                    Publishers.at( pub_index ).publish( outImg );

                    if ( is_rois.at( pub_index ) )
                    {
                        outImg.image
                        = pres.at( pub_index )->do_preprocess( images_tmp.at( pub_index ).image );
                        imageROIPublishers.at( pub_index ).publish( outImg );
                    }
                }
            }

            if ( is_show )
            {
                cv::imshow( "image", images_tmp.at( 0 ).image );
                cv::imshow( "image2", images_tmp.at( 1 ).image );
                cv::waitKey( 10 );
            }
        }
        // loop.sleep( );
    }

    camReader.stopCamera( );

    std::cout << "[#INFO] stop Camera Done!" << std::endl;

    return 0;
}
