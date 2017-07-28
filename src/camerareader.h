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
    bool setAutoExposure( FlyCapture2::Error& error, float exposure );
    bool setGain( FlyCapture2::Error& error, float exposure );
    bool setGamma( FlyCapture2::Error& error, float gamma ); // useless
    bool setShutter( FlyCapture2::Error& error, float shutter );
    bool setShutterAuto( FlyCapture2::Error& error );

    bool isColorCamera( );

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
    FlyCapture2::FC2Config& camConfig( ) { return cameraConfig; }

    private:
    std::string name;

    FlyCapture2::FC2Config cameraConfig;
    FlyCapture2::CameraInfo cameraInfo;
    FlyCapture2::PGRGuid guid;
    FlyCapture2::Camera camera;
};

class singleCameraReader
{
    public:
    singleCameraReader( ) {}
    singleCameraReader( int serialNumber ) { getCameraNum( ); }
    ~singleCameraReader( ) {}

    public:
    unsigned int getCameraNum( );
    void connectToCamera( )
    {
        camera.connectCamera( error );

        camera.getCameraInfo( error );
        camera.printCameraInfo( );

        camera.getCameraConfiguration( error );

        // Set the number of driver buffers used to 10.
        camera.camConfig( ).numBuffers = 10;
        //    config.numImageNotifications    = 0;
        //    config.minNumImageNotifications = 0;
        camera.camConfig( ).grabTimeout = FlyCapture2::TIMEOUT_UNSPECIFIED;
        camera.camConfig( ).highPerformanceRetrieveBuffer = true;
        camera.camConfig( ).grabMode                      = FlyCapture2::DROP_FRAMES;
        camera.setCameraConfiguration( error );
        camera.setMetadata( error );
    }
    void setCameraProperty( double frameRate, double brightness, double exposure, double gain, bool is_auto_shutter, double shutter )
    {
        camera.setBrightness( error, brightness );
        camera.setAutoExposure( error, exposure );
        camera.setGain( error, gain );
        camera.setFrameRate( error, frameRate );
        if ( is_auto_shutter )
            camera.setShutterAuto( error );
        else
            camera.setShutter( error, shutter );
    }
    void printCameraProperty( )
    {
        camera.getBrightness( error );
        camera.getFrameRate( error );
        camera.getSharpness( error );
        camera.getAutoExposure( error );
        camera.getWhiteBalance( error );
        camera.getHue( error );
        camera.getSaturation( error );
        camera.getGamma( error );
        camera.getIris( error );
        camera.getShutter( error );
        camera.getGain( error );
        camera.getTriggerMode( error );
        camera.getTriggerDelay( error );
    }
    void startCapture( ) { camera.startCapture( error ); }
    cv::Mat grabImage( )
    {
        cv::Mat cv_image;
        bool is_getNewImage = camera.captureOneImage( error, cv_image );
        return cv_image;
    }
    void stopCamera( )
    {
        camera.StopCapture( error );
        camera.disconnectCamera( error );
    }

    private:
    unsigned int cameraNum;
    singleCamera camera;

    FlyCapture2::Error error;
    FlyCapture2::BusManager busMgr;
};

#endif // CAMERAREADER_H
