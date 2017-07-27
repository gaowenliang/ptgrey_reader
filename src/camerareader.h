#ifndef CAMERAREADER_H
#define CAMERAREADER_H

#include <iostream>
#include <vector>

#include <flycapture/FlyCapture2.h>
#include <opencv2/core.hpp>

class singleCamera
{
    public:
    singleCamera( ) {}
    ~singleCamera( ) {}

    public:
    float getFrameRate( FlyCapture2::Error& error );
    float getBrightness( FlyCapture2::Error& error );
    float getAutoExposure( FlyCapture2::Error& error );
    float getSharpness( FlyCapture2::Error& error );
    float getWhiteBalance( FlyCapture2::Error& error );
    float getHue( FlyCapture2::Error& error );
    float getSaturation( FlyCapture2::Error& error );
    float getGamma( FlyCapture2::Error& error );
    float getIris( FlyCapture2::Error& error );
    float getShutter( FlyCapture2::Error& error );
    float getGain( FlyCapture2::Error& error );
    float getTriggerMode( FlyCapture2::Error& error );
    float getTriggerDelay( FlyCapture2::Error& error );

    bool setMetadata( FlyCapture2::Error& error );
    bool setFrameRate( FlyCapture2::Error& error, float rate );
    bool setBrightness( FlyCapture2::Error& error, float brightness );

    public:
    bool connectCamera( FlyCapture2::Error& error );
    bool getCameraInfo( FlyCapture2::Error& error );
    void printCameraInfo( );
    bool getCameraConfiguration( FlyCapture2::Error& error );
    bool setCameraConfiguration( FlyCapture2::Error& error );
    bool startCapture( FlyCapture2::Error& error );
    bool captureOneImage( FlyCapture2::Error& error, cv::Mat& image );
    bool StopCapture( FlyCapture2::Error& error );
    bool disconnectCamera( FlyCapture2::Error& error );

    FlyCapture2::PGRGuid& UniquePGRGuid( ) { return guid; }
    FlyCapture2::CameraInfo& camInfo( ) { return cameraInfo; }
    FlyCapture2::FC2Config& camConfig( ) { return config; }

    private:
    std::string name;
    FlyCapture2::FC2Config config;
    FlyCapture2::CameraInfo cameraInfo;
    FlyCapture2::PGRGuid guid;
    FlyCapture2::Camera cam;
};

class CameraReader
{
    public:
    CameraReader( ) { getCameraNum( ); }
    ~CameraReader( ) {}

    public:
    unsigned int getCameraNum( );

    private:
    unsigned int cameraNum;
    std::vector< singleCamera > cameras;

    FlyCapture2::Error error;
    FlyCapture2::BusManager busMgr;
};

#endif // CAMERAREADER_H
