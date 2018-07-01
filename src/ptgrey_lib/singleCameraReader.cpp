#include "singleCameraReader.h"

using namespace ptgrey_reader;

unsigned int
singleCameraReader::getConnectCameraNum( )
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

void
singleCameraReader::setCameraProperty( double frameRate,
                                       double brightness,
                                       double exposure,
                                       double gain,
                                       bool is_auto_shutter,
                                       double shutter,
                                       int WB_red,
                                       int WB_Blue,
                                       double saturation,
                                       double hue,
                                       double sharpness,
                                       bool is_sync )
{
    Camera( ).setBrightness( error, brightness );
    Camera( ).setAutoExposure( error, exposure );
    Camera( ).setWhiteBalance( error, WB_red, WB_Blue );
    Camera( ).setGain( error, gain );
    Camera( ).setFrameRate( error, frameRate );
    Camera( ).setTimeout( error, 1000. / frameRate );

    Camera( ).setSaturation( error, saturation );
    Camera( ).setHue( error, hue );
    Camera( ).setSharpness( error, sharpness );

    if ( is_auto_shutter )
        Camera( ).setShutterAuto( error );
    else
        Camera( ).setShutter( error, shutter );

    if ( is_sync )
        Camera( ).setTrigger( error );
    else
        Camera( ).setTriggerOFF( error );
}

void
singleCameraReader::printCameraProperty( )
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
    std::cout << "[#INFO] print Camera Property Done." << std::endl;
}

bool
singleCameraReader::startCamera( unsigned int serialNum,
                                 double frameRate,
                                 double brightness,
                                 double exposure,
                                 double gain,
                                 bool is_auto_shutter,
                                 double shutter,
                                 int WB_red,
                                 int WB_Blue,
                                 double saturation,
                                 double hue,
                                 double sharpness,
                                 bool is_print_info,
                                 bool is_sync )
{
    getConnectCameraNum( );

    if ( cameraNum( ) <= 0 )
    {
        std::cout << "[#INFO] No PointGrey Camera connect." << std::endl;
        std::cout << "[#INFO] Check Camera." << std::endl;
        return false;
    }
    else if ( cameraNum( ) > 0 )
        Camera( ).initCamera( error, BusManager( ) );

    bool is_connected = Camera( ).connectCamera( error );
    if ( is_connected )
    {
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

        setCameraProperty( frameRate,
                           brightness,
                           exposure,
                           gain,
                           is_auto_shutter, //
                           shutter,
                           WB_red,
                           WB_Blue,
                           saturation,
                           hue,
                           sharpness,
                           is_sync );

        if ( is_print_info )
            printCameraProperty( );

        Camera( ).startCapture( error );
        std::cout << "[#INFO] start Capture." << std::endl;

        return true;
    }
    else
        return false;
}

cvImage
singleCameraReader::grabImage( )
{
    cvImage cv_image;
    Camera( ).captureOneImage( error, cv_image.image, cv_image.time );
    return cv_image;
}

void
singleCameraReader::stopCamera( )
{
    Camera( ).StopCapture( error );
    Camera( ).disconnectCamera( error );
}

FlyCapture2::BusManager&
singleCameraReader::BusManager( )
{
    return busMgr;
}

singleCamera&
singleCameraReader::Camera( )
{
    return camera;
}

const int
singleCameraReader::cameraNum( ) const
{
    return cameraNumber;
}

ptgrey_reader::singleCameraReader::singleCameraReader( ) {}

singleCameraReader::singleCameraReader( const unsigned int serial_num )
: camera( serial_num )
{
}

singleCameraReader::~singleCameraReader( ) {}
