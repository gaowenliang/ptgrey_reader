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
    const unsigned int getSerialNumber( ) const;
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
    unsigned int serialNumber;
    FlyCapture2::FC2Config cameraConfig;
    FlyCapture2::CameraInfo cameraInfo;
    FlyCapture2::PGRGuid guid;
    FlyCapture2::Camera camera;
};

class singleCameraReader
{
    public:
    singleCameraReader( ) {}
    ~singleCameraReader( ) {}

    public:
    unsigned int getConnectCameraNum( )
    {
        error = busMgr.GetNumOfCameras( &cameraNumber );
        if ( error != FlyCapture2::PGRERROR_OK )
        {
            std::cout << "[#INFO]Error in getCameraNum " << std::endl;
            error.PrintErrorTrace( );
            return 0;
        }
        else
        {
            std::cout << "Number of cameras detected: " << cameraNumber << std::endl;
            return cameraNumber;
        }
    }
    void connectToCamera( bool is_print_info = false )
    {
        Camera( ).connectCamera( error );

        Camera( ).getCameraInfo( error );
        if ( is_print_info )
            Camera( ).printCameraInfo( );

        Camera( ).getCameraConfiguration( error );

        // Set the number of driver buffers used to 10.
        Camera( ).camConfig( ).numBuffers = 10;
        //    config.numImageNotifications    = 0;
        //    config.minNumImageNotifications = 0;
        Camera( ).camConfig( ).grabTimeout = FlyCapture2::TIMEOUT_UNSPECIFIED;
        Camera( ).camConfig( ).highPerformanceRetrieveBuffer = true;
        Camera( ).camConfig( ).grabMode                      = FlyCapture2::DROP_FRAMES;
        Camera( ).setCameraConfiguration( error );
        Camera( ).setMetadata( error );
    }
    void setCameraProperty( double frameRate, double brightness, double exposure, double gain, bool is_auto_shutter, double shutter )
    {
        Camera( ).setBrightness( error, brightness );
        Camera( ).setAutoExposure( error, exposure );
        Camera( ).setGain( error, gain );
        Camera( ).setFrameRate( error, frameRate );
        if ( is_auto_shutter )
            Camera( ).setShutterAuto( error );
        else
            Camera( ).setShutter( error, shutter );
    }
    void printCameraProperty( )
    {
        Camera( ).getBrightness( error );
        Camera( ).getFrameRate( error );
        Camera( ).getSharpness( error );
        Camera( ).getAutoExposure( error );
        Camera( ).getWhiteBalance( error );
        Camera( ).getHue( error );
        Camera( ).getSaturation( error );
        Camera( ).getGamma( error );
        Camera( ).getIris( error );
        Camera( ).getShutter( error );
        Camera( ).getGain( error );
        Camera( ).getTriggerMode( error );
        Camera( ).getTriggerDelay( error );
    }

    bool startCamera( unsigned int serialNum,
                      double frameRate,
                      double brightness,
                      double exposure,
                      double gain,
                      bool is_auto_shutter,
                      double shutter,
                      bool is_print_info = false )
    {
        getConnectCameraNum( );

        if ( cameraNum( ) == 0 )
        {
            std::cout << "[#INFO] No PointGrey Camera connect." << std::endl;
            std::cout << "[#INFO] Check Camera." << std::endl;
            return false;
        }
        else if ( cameraNum( ) >= 1 )
        {
            bool is_camera_correct = false;
            for ( int cameraIndex = 0; cameraIndex < cameraNum( ); ++cameraIndex )
            {
                BusManager( ).GetCameraFromIndex( cameraIndex, &Camera( ).UniquePGRGuid( ) );

                Camera( ).connectCamera( error );
                Camera( ).getCameraInfo( error );
                if ( is_print_info )
                    Camera( ).printCameraInfo( );
                Camera( ).disconnectCamera( error );

                if ( serialNum == Camera( ).getSerialNumber( ) )
                {
                    is_camera_correct = true;
                    break;
                }
                else
                    continue;
            }
            if ( !is_camera_correct )
            {
                std::cout << "[#INFO] No PointGrey Camera " << serialNum;
                std::cout << " connect." << std::endl;
                std::cout << "[#INFO] Check Camera." << std::endl;
                return false;
            }
            else
                std::cout << "[#INFO] Connect to Camera " << serialNum << std::endl;
        }

        connectToCamera( );

        setCameraProperty( frameRate, brightness, exposure, gain, is_auto_shutter, shutter );
        if ( is_print_info )
            printCameraProperty( );

        Camera( ).startCapture( error );

        return true;
    }
    cv::Mat grabImage( )
    {
        cv::Mat cv_image;
        Camera( ).captureOneImage( error, cv_image );
        return cv_image;
    }
    void stopCamera( )
    {
        Camera( ).StopCapture( error );
        Camera( ).disconnectCamera( error );
    }

    FlyCapture2::BusManager& BusManager( ) { return busMgr; }
    singleCamera& Camera( ) { return camera; }
    const int cameraNum( ) const { return cameraNumber; }

    private:
    unsigned int cameraNumber;
    singleCamera camera;
    FlyCapture2::Error error;
    FlyCapture2::BusManager busMgr;
};

#endif // CAMERAREADER_H
