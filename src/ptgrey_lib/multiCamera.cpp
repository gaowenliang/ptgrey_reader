#include "multiCamera.h"

using namespace ptgrey_reader;

bool
multiCamera::initMultiCamera( FlyCapture2::Error& error, FlyCapture2::BusManager& BusMana )
{
    bool is_allCameraOk = true;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras.at( camera_index )->initCamera( error, BusMana ) )
        {
            std::cout << "[#INFO]Error in initMultiCamera in camera " << pcameras[camera_index]->getSerialNumber( )
                      << "." << std::endl;
            is_allCameraOk = is_allCameraOk & false;
            continue;
        }
        else
        {
            is_allCameraOk = is_allCameraOk & true;
            continue;
        }
    }

    return is_allCameraOk;
}

bool
ptgrey_reader::multiCamera::setMetadata( FlyCapture2::Error& error )
{
    bool is_allSetRight = false;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->setMetadata( error ) )
        {
            std::cout << "[#INFO]Error in setMetadata in camera " << pcameras[camera_index]->getSerialNumber( ) << "." << std::endl;
            is_allSetRight = false;
            continue;
        }
        else
        {
            is_allSetRight = true;
            continue;
        }
    }

    return is_allSetRight == true ? true : false;
}

bool
multiCamera::setFrameRate( FlyCapture2::Error& error, float rate )
{
    bool is_allSetRight = false;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->setFrameRate( error, rate ) )
        {
            std::cout << "[#INFO]Error in setFrameRate in camera " << pcameras[camera_index]->getSerialNumber( ) << "."
                      << std::endl;
            is_allSetRight = false;
            continue;
        }
        else
        {
            is_allSetRight = true;
            continue;
        }
    }

    return is_allSetRight == true ? true : false;
}

bool
multiCamera::setBrightness( FlyCapture2::Error& error, float brightness )
{
    bool is_allSetRight = false;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->setBrightness( error, brightness ) )
        {
            std::cout << "[#INFO]Error in setBrightness in camera " << pcameras[camera_index]->getSerialNumber( ) << "."
                      << std::endl;
            is_allSetRight = false;
            continue;
        }
        else
        {
            is_allSetRight = true;
            continue;
        }
    }

    return is_allSetRight == true ? true : false;
}

bool
multiCamera::setAutoExposure( FlyCapture2::Error& error, float exposure )
{
    bool is_allSetRight = false;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->setAutoExposure( error, exposure ) )
        {
            std::cout << "[#INFO]Error in setAutoExposure in camera " << pcameras[camera_index]->getSerialNumber( )
                      << "." << std::endl;
            is_allSetRight = false;
            continue;
        }
        else
        {
            is_allSetRight = true;
            continue;
        }
    }

    return is_allSetRight == true ? true : false;
}

bool
multiCamera::setGain( FlyCapture2::Error& error, float gain )
{
    bool is_allSetRight = false;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->setGain( error, gain ) )
        {
            std::cout << "[#INFO]Error in setGain in camera " << pcameras[camera_index]->getSerialNumber( ) << "." << std::endl;
            is_allSetRight = false;
            continue;
        }
        else
        {
            is_allSetRight = true;
            continue;
        }
    }

    return is_allSetRight == true ? true : false;
}

bool
multiCamera::setGamma( FlyCapture2::Error& error, float gamma )
{
    bool is_allSetRight = false;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->setGamma( error, gamma ) )
        {
            std::cout << "[#INFO]Error in setGamma in camera " << pcameras[camera_index]->getSerialNumber( ) << "." << std::endl;
            is_allSetRight = false;
            continue;
        }
        else
        {
            is_allSetRight = true;
            continue;
        }
    }

    return is_allSetRight == true ? true : false;
}

bool
multiCamera::setShutter( FlyCapture2::Error& error, float shutter )
{
    bool is_allSetRight = false;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->setShutter( error, shutter ) )
        {
            std::cout << "[#INFO]Error in setShutter in camera " << pcameras[camera_index]->getSerialNumber( ) << "." << std::endl;
            is_allSetRight = false;
            continue;
        }
        else
        {
            is_allSetRight = true;
            continue;
        }
    }

    return is_allSetRight == true ? true : false;
}

bool
multiCamera::setShutterAuto( FlyCapture2::Error& error )
{
    bool is_allSetRight = false;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->setShutterAuto( error ) )
        {
            std::cout << "[#INFO]Error in setShutterAuto in camera " << pcameras[camera_index]->getSerialNumber( )
                      << "." << std::endl;
            is_allSetRight = false;
            continue;
        }
        else
        {
            is_allSetRight = true;
            continue;
        }
    }

    return is_allSetRight == true ? true : false;
}

bool
multiCamera::isColorCamera( )
{
    bool is_all_color = true;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        is_all_color = is_all_color & pcameras[camera_index]->isColorCamera( );
    }

    return is_all_color;
}

bool
multiCamera::connectCamera( FlyCapture2::Error& error )
{
    bool is_allCameraConnect = true;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->connectCamera( error ) )
        {
            std::cout << "[#INFO]Error in connectCamera in camera " << pcameras[camera_index]->getSerialNumber( ) << "."
                      << std::endl;
            is_allCameraConnect = is_allCameraConnect & false;
            continue;
        }
        else
        {
            is_allCameraConnect = is_allCameraConnect & true;
            continue;
        }
    }

    return is_allCameraConnect;
}

bool
multiCamera::getCameraInfo( FlyCapture2::Error& error )
{
    bool is_allCameraOk = true;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->getCameraInfo( error ) )
        {
            std::cout << "[#INFO]Error in connectCamera in camera " << pcameras[camera_index]->getSerialNumber( ) << "."
                      << std::endl;
            is_allCameraOk = is_allCameraOk & false;
            continue;
        }
        else
        {
            is_allCameraOk = is_allCameraOk & true;
            continue;
        }
    }

    return is_allCameraOk;
}

std::vector< unsigned int >
multiCamera::getSerialNumbers( ) const
{
    std::vector< unsigned int > serialNumbers;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        unsigned int serial_tmp = pcameras[camera_index]->getSerialNumber( );
        serialNumbers.push_back( serial_tmp );
    }
    return serialNumbers;
}

void
multiCamera::printCameraInfo( )
{
    std::cout << " pcameras " << pcameras.size( ) << std::endl;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        std::cout << "[#INFO] Camera " << camera_index << std::endl;
        pcameras[camera_index]->printCameraInfo( );
    }
}

bool
multiCamera::getCameraConfiguration( FlyCapture2::Error& error )
{
    bool is_allCameraOk = true;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->getCameraConfiguration( error ) )
        {
            std::cout << "[#INFO]Error in getCameraConfiguration in camera "
                      << pcameras[camera_index]->getSerialNumber( ) << "." << std::endl;
            is_allCameraOk = is_allCameraOk & false;
            continue;
        }
        else
        {
            is_allCameraOk = is_allCameraOk & true;
            continue;
        }
    }

    return is_allCameraOk;
}

bool
multiCamera::setCameraConfiguration( FlyCapture2::Error& error )
{
    bool is_allCameraOk = true;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->setCameraConfiguration( error ) )
        {
            std::cout << "[#INFO]Error in setCameraConfiguration in camera "
                      << pcameras[camera_index]->getSerialNumber( ) << "." << std::endl;
            is_allCameraOk = is_allCameraOk & false;
            continue;
        }
        else
        {
            is_allCameraOk = is_allCameraOk & true;
            continue;
        }
    }

    return is_allCameraOk;
}

bool
multiCamera::setCameraConfiguration( FlyCapture2::Error& error, FlyCapture2::FC2Config cfg )
{
    bool is_allCameraOk = true;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->setCameraConfiguration( error, cfg ) )
        {
            std::cout << "[#INFO]Error in setCameraConfiguration in camera "
                      << pcameras[camera_index]->getSerialNumber( ) << "." << std::endl;
            is_allCameraOk = is_allCameraOk & false;
            continue;
        }
        else
        {
            is_allCameraOk = is_allCameraOk & true;
            continue;
        }
    }

    return is_allCameraOk;
}

bool
multiCamera::startCapture( FlyCapture2::Error& error )
{
    bool is_allCameraOk = true;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->startCapture( error ) )
        {
            std::cout << "[#INFO]Error in startCapture in camera " << pcameras[camera_index]->getSerialNumber( ) << "."
                      << std::endl;
            is_allCameraOk = is_allCameraOk & false;
            continue;
        }
        else
        {
            is_allCameraOk = is_allCameraOk & true;
            continue;
        }
    }

    return is_allCameraOk;
}

void
multiCamera::captureImage( FlyCapture2::Error& error, std::vector< std::pair< cv::Mat, FlyCapture2::TimeStamp > >& images )
{
    bool is_allCameraOk = true;
    //    std::vector< std::pair< cv::Mat, FlyCapture2::TimeStamp > > images_tmp;
    // std::cout << " once " << std::endl;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        cv::Mat image_tmp;
        FlyCapture2::TimeStamp time_tmp;
        std::pair< cv::Mat, FlyCapture2::TimeStamp > imageWithTime;
        if ( !getCameras( ).at( camera_index )->captureOneImage( error, image_tmp, time_tmp ) )
        {
            std::cout << "[#INFO]Error in captureImage in camera " << pcameras[camera_index]->getSerialNumber( ) << "."
                      << std::endl;
            is_allCameraOk = is_allCameraOk & false;
            continue;
        }
        else
        {
            imageWithTime.first  = image_tmp;
            imageWithTime.second = time_tmp;
            // std::cout << camera_index << " " << double( time_tmp.seconds ) << " "
            //          << double( time_tmp.microSeconds * 0.000001 ) << std::endl;

            images.at( camera_index ) = imageWithTime;

            is_allCameraOk = is_allCameraOk & true;
            continue;
        }
    }

    //    return images_tmp;
}

bool
multiCamera::StopCapture( FlyCapture2::Error& error )
{
    bool is_allCameraStoped = true;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->StopCapture( error ) )
        {
            std::cout << "[#INFO]Error in startCapture in camera " << pcameras[camera_index]->getSerialNumber( ) << "."
                      << std::endl;
            is_allCameraStoped = is_allCameraStoped & false;
            continue;
        }
        else
        {
            is_allCameraStoped = is_allCameraStoped & true;
            continue;
        }
    }

    return is_allCameraStoped;
}

bool
multiCamera::disconnectCamera( FlyCapture2::Error& error )
{
    bool is_allCameraStoped = true;
    for ( int camera_index = 0; camera_index < int( cameraNum ); ++camera_index )
    {
        if ( !pcameras[camera_index]->disconnectCamera( error ) )
        {
            std::cout << "[#INFO]Error in startCapture in camera " << pcameras[camera_index]->getSerialNumber( ) << "."
                      << std::endl;
            is_allCameraStoped = is_allCameraStoped & false;
            continue;
        }
        else
        {
            is_allCameraStoped = is_allCameraStoped & true;
            continue;
        }
    }

    return is_allCameraStoped;
}
