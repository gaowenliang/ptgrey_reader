#include "multiCameraReader.h"

unsigned int
ptgrey_reader::multiCameraReader::getConnectCameraNum( )
{
    error = busMgr.GetNumOfCameras( &cameraNumber );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getCameraNum." << std::endl;
        error.PrintErrorTrace( );
        return 0;
    }
    else
    {
        std::cout << "Number of cameras detected: " << cameraNumber << std::endl;
        return cameraNumber;
    }
}

void
ptgrey_reader::multiCameraReader::setCameraProperty( const double frameRate,
                                                     const double brightness,
                                                     const double exposure,
                                                     const double gain,
                                                     const bool is_auto_shutter,
                                                     const double shutter,
                                                     const bool is_sync )
{
    for ( int camera_index = 0; camera_index < int( cameraNumber ); ++camera_index )
    {
        Cameras( )->getCameras( ).at( camera_index )->setBrightness( error, brightness );
        Cameras( )->getCameras( ).at( camera_index )->setAutoExposure( error, exposure );
        Cameras( )->getCameras( ).at( camera_index )->setGain( error, gain );
        Cameras( )->getCameras( ).at( camera_index )->setFrameRate( error, frameRate );
        if ( is_auto_shutter )
            Cameras( )->getCameras( ).at( camera_index )->setShutterAuto( error );
        else
            Cameras( )->getCameras( ).at( camera_index )->setShutter( error, shutter );

        if ( is_sync )
            Cameras( )->getCameras( ).at( camera_index )->setTrigger( error );
        else
            Cameras( )->getCameras( ).at( camera_index )->setTriggerOFF( error );
    }
}

void
ptgrey_reader::multiCameraReader::printCameraProperty( )
{
    for ( int camera_index = 0; camera_index < int( cameraNumber ); ++camera_index )
    {
        std::cout << "[#INFO] Property of camera " << Cameras( )->getCameras( ).at( camera_index )->getSerialNumber( );

        Cameras( )->getCameras( ).at( camera_index )->getBrightness( error );
        Cameras( )->getCameras( ).at( camera_index )->getFrameRate( error );
        Cameras( )->getCameras( ).at( camera_index )->getSharpness( error );
        Cameras( )->getCameras( ).at( camera_index )->getAutoExposure( error );
        Cameras( )->getCameras( ).at( camera_index )->getWhiteBalance( error );
        Cameras( )->getCameras( ).at( camera_index )->getHue( error );
        Cameras( )->getCameras( ).at( camera_index )->getSaturation( error );
        Cameras( )->getCameras( ).at( camera_index )->getGamma( error );
        Cameras( )->getCameras( ).at( camera_index )->getIris( error );
        Cameras( )->getCameras( ).at( camera_index )->getShutter( error );
        Cameras( )->getCameras( ).at( camera_index )->getGain( error );
        Cameras( )->getCameras( ).at( camera_index )->getTriggerMode( error );
        Cameras( )->getCameras( ).at( camera_index )->getTriggerDelay( error );
        std::cout << "[#INFO] print Camera Property Done." << std::endl;
    }
}

bool
ptgrey_reader::multiCameraReader::startCamera( const std::vector< unsigned int > serialNum,
                                               const double frameRate,
                                               const double brightness,
                                               const double exposure,
                                               const double gain,
                                               const bool is_auto_shutter,
                                               const double shutter,
                                               bool is_print_info,
                                               bool is_synchronized )
{
    getConnectCameraNum( );

    if ( cameraNum( ) <= 0 && cameraNum( ) == int( serialNum.size( ) ) )
    {
        std::cout << "[#INFO] No PointGrey Camera connect." << std::endl;
        std::cout << "[#INFO] Check Camera." << std::endl;
        return false;
    }
    else if ( cameraNum( ) > 0 )
        Cameras( )->initMultiCamera( error, BusManager( ) );

    bool is_connected = Cameras( )->connectCamera( error );
    if ( is_connected )
    {
        std::cout << "[#INFO] Cameras connected." << std::endl;
        Cameras( )->getCameraInfo( error );
        //        if ( is_print_info )
        Cameras( )->printCameraInfo( );

        Cameras( )->getCameraConfiguration( error );

        FlyCapture2::FC2Config cameraConfig;
        cameraConfig.numBuffers                    = 10; // Set the number of driver buffers used to 10.
        cameraConfig.grabTimeout                   = FlyCapture2::TIMEOUT_UNSPECIFIED;
        cameraConfig.highPerformanceRetrieveBuffer = true;
        cameraConfig.grabMode                      = FlyCapture2::DROP_FRAMES;
        Cameras( )->setCameraConfiguration( error, cameraConfig );
        Cameras( )->setMetadata( error );

        setCameraProperty( frameRate, brightness, exposure, gain, is_auto_shutter, shutter, is_synchronized );
        if ( is_print_info )
            printCameraProperty( );

        Cameras( )->startCapture( error );
        std::cout << "[#INFO] start Capture." << std::endl;

        return true;
    }
    else
        return false;
}

bool
ptgrey_reader::multiCameraReader::grabImage( std::vector< ptgrey_reader::cvImage >& cv_images )
{
    //    std::vector< std::pair< cv::Mat, FlyCapture2::TimeStamp > > imageWithStamp;
    Cameras( )->captureImage( error, imageWithStamp );

    //    cv_images.resize( cameraNumber );
    for ( int camera_index = 0; camera_index < cameraNum( ); ++camera_index )
    {
        cv_images.at( camera_index ).image = imageWithStamp.at( camera_index ).first;
        cv_images.at( camera_index ).time  = imageWithStamp.at( camera_index ).second;
    }

    return true;
}

void
ptgrey_reader::multiCameraReader::stopCamera( )
{
    Cameras( )->StopCapture( error );
    Cameras( )->disconnectCamera( error );
}
