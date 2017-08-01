#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

#include "multiCameraReader.h"
#include <cv_bridge/cv_bridge.h>
#include <flycapture/FlyCapture2.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sstream>

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "multiCameraReader" );
    ros::NodeHandle nh( "~" );

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

    return 0;
}
