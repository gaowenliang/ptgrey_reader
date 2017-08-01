#ifndef MULTICAMERA_H
#define MULTICAMERA_H

#include "singleCamera.h"

class multiCamera
{
    public:
    multiCamera( ) {}
    ~multiCamera( ) {}

    multiCamera( const std::vector< unsigned int > serialNum )
    {
        cameraNumber = serialNum.size( );

        for ( int cameraIndex = 0; cameraIndex < cameraNumber; ++cameraIndex )
        {
            singleCamera singleCam( serialNum.at( cameraIndex ) );

            cameras.push_back( singleCam );
            ppCameras[cameraIndex] = singleCam.getPCamera( );
        }
    }

    bool initMultiCamera( FlyCapture2::Error& error, FlyCapture2::BusManager& BusMana )
    {
        for ( int camera_index = 0; camera_index < cameraNumber; ++camera_index )
            cameras.at( camera_index ).initCamera( error, BusMana );
    }

    bool setMetadata( FlyCapture2::Error& error )
    {
        bool is_allSetRight = false;
        for ( int camera_index = 0; camera_index < cameraNumber; ++camera_index )
        {
            FlyCapture2::EmbeddedImageInfo info;
            info.timestamp.onOff    = true;
            info.gain.onOff         = true;
            info.shutter.onOff      = true;
            info.brightness.onOff   = true;
            info.exposure.onOff     = true;
            info.whiteBalance.onOff = true;
            info.frameCounter.onOff = true;
            info.ROIPosition.onOff  = true;
            error = cameras[camera_index].getPCamera( )->SetEmbeddedImageInfo( &info );
            if ( error != FlyCapture2::PGRERROR_OK )
            {
                std::cout << "[#INFO]Error in setMetadata in camera "
                          << cameras[camera_index].getSerialNumber( ) << "." << std::endl;
                error.PrintErrorTrace( );
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

    private:
    unsigned int cameraNumber;           // number of cameras
    FlyCapture2::Camera** ppCameras;     // for multiple camera SDK use
    std::vector< singleCamera > cameras; // single cameras
};

#endif // MULTICAMERA_H
