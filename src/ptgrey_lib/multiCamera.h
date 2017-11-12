#ifndef MULTICAMERA_H
#define MULTICAMERA_H

#include "singleCamera.h"

#include "../../config/multi_config.h"

namespace ptgrey_reader
{

class multiCamera
{

    public:
    multiCamera( ) {}
    multiCamera( const std::vector< unsigned int > serialNum )
    {
        cameraNum = serialNum.size( );
        FlyCapture2::Camera* thePppCameras[CAM_NUM]; // for multiple camera SDK use

        for ( int cameraIndex = 0; cameraIndex < int( cameraNum ); ++cameraIndex )
        {
            singleCamera* psingleCam = new singleCamera( serialNum.at( cameraIndex ) );
            std::cout << "[#INFO] CameraIndex " << cameraIndex << " serialNum " << serialNum[cameraIndex] << std::endl;

            pcameras.push_back( psingleCam );
            thePppCameras[cameraIndex] = psingleCam->getPCamera( );
        }
        ppCameras = thePppCameras;
        //        std::cout << "cameras size " << pcameras.size( ) << std::endl;
    }
    ~multiCamera( ) {}

    bool initMultiCamera( FlyCapture2::Error& error, FlyCapture2::BusManager& BusMana );

    public:
    bool setMetadata( FlyCapture2::Error& error );
    bool setFrameRate( FlyCapture2::Error& error, float rate );
    bool setBrightness( FlyCapture2::Error& error, float brightness );
    bool setAutoExposure( FlyCapture2::Error& error, float exposure );
    bool setGain( FlyCapture2::Error& error, float gain );
    bool setGamma( FlyCapture2::Error& error, float gamma );
    bool setShutter( FlyCapture2::Error& error, float shutter );
    bool setShutterAuto( FlyCapture2::Error& error );

    bool isColorCamera( );

    public:
    bool connectCamera( FlyCapture2::Error& error );
    bool getCameraInfo( FlyCapture2::Error& error );
    std::vector< unsigned int > getSerialNumbers( ) const;
    void printCameraInfo( );
    bool getCameraConfiguration( FlyCapture2::Error& error );
    bool setCameraConfiguration( FlyCapture2::Error& error );
    bool setCameraConfiguration( FlyCapture2::Error& error, FlyCapture2::FC2Config cfg );
    bool startCapture( FlyCapture2::Error& error );
    void captureImage( FlyCapture2::Error& error, std::vector< std::pair< cv::Mat, FlyCapture2::TimeStamp > >& images );
    bool StopCapture( FlyCapture2::Error& error );
    bool disconnectCamera( FlyCapture2::Error& error );

    unsigned int cameraNumber( ) const { return cameraNum; }
    FlyCapture2::Camera** getPpCameras( ) const { return ppCameras; }
    std::vector< singleCamera* > getCameras( ) const { return pcameras; }

    private:
    unsigned int cameraNum;                // number of cameras
    FlyCapture2::Camera** ppCameras;       // for multiple camera SDK use
    std::vector< singleCamera* > pcameras; // single cameras

    bool is_sync;
};
}
#endif // MULTICAMERA_H
