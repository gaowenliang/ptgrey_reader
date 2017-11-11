//#define BACKWARD_HAS_DW 1
//#include "backward.hpp"
// namespace backward
//{
// backward::SignalHandling sh;
//}

//#include <code_utils/sys_utils.h>
#include <flycapture/FlyCapture2.h>
#include <iostream>
#include <opencv2/opencv.hpp>
//#include <ros/ros.h>
#include <sstream>

using namespace FlyCapture2;
using namespace std;

bool is_show = true;

void
PrintBuildInfo( )
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion( &fc2Version );

    ostringstream version;
    version << "FlyCapture2 library version: " << fc2Version.major << "." << fc2Version.minor << "."
            << fc2Version.type << "." << fc2Version.build;
    std::cout << version.str( ) << std::endl;

    ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
    std::cout << timeStamp.str( ) << std::endl << std::endl;
}

void
PrintCameraInfo( CameraInfo* pCamInfo )
{
    std::cout << std::endl;
    std::cout << "========== CAMERA INFORMATION ============" << std::endl;
    std::cout << "         Serial number | " << pCamInfo->serialNumber << std::endl;
    std::cout << "          Camera model | " << pCamInfo->modelName << std::endl;
    std::cout << "         Camera vendor | " << pCamInfo->vendorName << std::endl;
    std::cout << "                Sensor | " << pCamInfo->sensorInfo << std::endl;
    std::cout << "            Resolution | " << pCamInfo->sensorResolution << std::endl;
    std::cout << "      Firmware version | " << pCamInfo->firmwareVersion << std::endl;
    std::cout << "   Firmware build time | " << pCamInfo->firmwareBuildTime << std::endl << std::endl;
}

void
PrintError( Error error )
{
    error.PrintErrorTrace( );
}

int
RunSingleCamera( PGRGuid guid )
{
    Error error;
    Camera cam;

    // Connect to a camera
    error = cam.Connect( &guid );
    if ( error != PGRERROR_OK )
    {
        PrintError( error );
        return -1;
    }

    // Get the camera information
    CameraInfo camInfo;
    error = cam.GetCameraInfo( &camInfo );
    if ( error != PGRERROR_OK )
    {
        PrintError( error );
        return -1;
    }
    PrintCameraInfo( &camInfo );

    // Get the camera configuration
    FC2Config config;
    error = cam.GetConfiguration( &config );
    if ( error != PGRERROR_OK )
    {
        PrintError( error );
        return -1;
    }

    // Set the number of driver buffers used to 10.
    config.numBuffers = 10;
    //    config.numImageNotifications    = 0;
    //    config.minNumImageNotifications = 0;
    config.grabTimeout = 100;

    config.grabMode = DROP_FRAMES;

    // Set the camera configuration
    error = cam.SetConfiguration( &config );
    if ( error != PGRERROR_OK )
    {
        PrintError( error );
        return -1;
    }

    // Disconnect the camera
    error = cam.Disconnect( );
    if ( error != PGRERROR_OK )
    {
        std::cout << "    // Disconnect the camera  " << std::endl;
        PrintError( error );
        return -1;
    }

    return 0;
}

int
main( int argc, char** argv )
{
    //ros::init( argc, argv, "pointGreyReader" );
    //ros::NodeHandle nh( "~" );

    PrintBuildInfo( );

    Error error;

    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras( &numCameras );
    if ( error != PGRERROR_OK )
    {
        PrintError( error );
        return -1;
    }

    std::cout << "Number of cameras detected: " << numCameras << std::endl;

    for ( unsigned int i = 0; i < numCameras; i++ )
    {
        PGRGuid guid;
        error = busMgr.GetCameraFromIndex( i, &guid );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }

        RunSingleCamera( guid );
    }

    std::cout << "Done! Press Enter to exit..." << std::endl;

    return 0;
}
