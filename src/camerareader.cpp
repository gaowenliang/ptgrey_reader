#include "camerareader.h"

unsigned int
CameraReader::getCameraNum( )
{
    error = busMgr.GetNumOfCameras( &cameraNum );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getCameraNum " << std::endl;
        error.PrintErrorTrace( );
        return 0;
    }
    else
    {
        std::cout << "Number of cameras detected: " << cameraNum << std::endl;
        return cameraNum;
    }
}

float
singleCamera::getFrameRate( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::FRAME_RATE;
    error      = cam.GetProperty( &fProp );
    std::cout << " Frame Rate: " << fProp.absValue << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getFrameRate " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
    else
        return fProp.absValue;
}

float
singleCamera::getBrightness( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::BRIGHTNESS;
    error      = cam.GetProperty( &fProp );
    std::cout << " Brightness: " << fProp.absValue << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getBrightness " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
    else
        return fProp.absValue;
}

float
singleCamera::getAutoExposure( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::AUTO_EXPOSURE;
    error      = cam.GetProperty( &fProp );
    std::cout << " AutoExposure " << fProp.absValue << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getAutoExposure " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
    else
        return fProp.absValue;
}

float
singleCamera::getSharpness( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::SHARPNESS;
    error      = cam.GetProperty( &fProp );
    std::cout << " getSharpness "
              << "abs:" << fProp.absValue << " A:" << fProp.valueA << " B:" << fProp.valueB << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getSharpness " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
    else
        return fProp.absValue;
}

float
singleCamera::getWhiteBalance( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::WHITE_BALANCE;
    error      = cam.GetProperty( &fProp );
    std::cout << " getWhiteBalance "
              << "abs:" << fProp.absValue << " A:" << fProp.valueA << " B:" << fProp.valueB << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getWhiteBalance " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
    else
        return fProp.absValue;
}

float
singleCamera::getHue( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::HUE;
    error      = cam.GetProperty( &fProp );
    std::cout << " getHue "
              << "abs:" << fProp.absValue << " A:" << fProp.valueA << " B:" << fProp.valueB << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getWhiteBalance " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
    else
        return fProp.absValue;
}

float
singleCamera::getSaturation( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::SATURATION;
    error      = cam.GetProperty( &fProp );
    std::cout << " getSaturation "
              << "abs:" << fProp.absValue << " A:" << fProp.valueA << " B:" << fProp.valueB << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getWhiteBalance " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
    else
        return fProp.absValue;
}

float
singleCamera::getGamma( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::GAMMA;
    error      = cam.GetProperty( &fProp );
    std::cout << " getGamma "
              << "abs:" << fProp.absValue << " A:" << fProp.valueA << " B:" << fProp.valueB << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getGamma " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
    else
        return fProp.absValue;
}

float
singleCamera::getIris( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::IRIS;
    error      = cam.GetProperty( &fProp );
    std::cout << " getIris "
              << "abs:" << fProp.absValue << " A:" << fProp.valueA << " B:" << fProp.valueB << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getIris " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
    else
        return fProp.absValue;
}

float
singleCamera::getShutter( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::SHUTTER;
    error      = cam.GetProperty( &fProp );
    std::cout << " getShutter "
              << "abs:" << fProp.absValue << " A:" << fProp.valueA << " B:" << fProp.valueB << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getShutter " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
    else
        return fProp.absValue;
}

float
singleCamera::getGain( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::GAIN;
    error      = cam.GetProperty( &fProp );
    std::cout << " getGain "
              << "abs:" << fProp.absValue << " A:" << fProp.valueA << " B:" << fProp.valueB << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getGain " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
    else
        return fProp.absValue;
}

float
singleCamera::getTriggerMode( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::TRIGGER_MODE;
    error      = cam.GetProperty( &fProp );
    std::cout << " getTriggerMode "
              << "abs:" << fProp.absValue << " A:" << fProp.valueA << " B:" << fProp.valueB << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getTriggerMode " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
    else
        return fProp.absValue;
}

float
singleCamera::getTriggerDelay( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::TRIGGER_DELAY;
    error      = cam.GetProperty( &fProp );
    std::cout << " getTriggerDelay "
              << "abs:" << fProp.absValue << " A:" << fProp.valueA << " B:" << fProp.valueB << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getTriggerDelay " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
    else
        return fProp.absValue;
}

bool
singleCamera::connectCamera( FlyCapture2::Error& error )
{
    // Connect to a camera
    error = cam.Connect( &guid );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in Connect " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::getCameraInfo( FlyCapture2::Error& error )
{
    // Connect to a camera
    error = cam.GetCameraInfo( &cameraInfo );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in GetCameraInfo " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

void
singleCamera::printCameraInfo( )
{
    std::cout << std::endl;
    std::cout << "========== CAMERA INFORMATION ============" << std::endl;
    std::cout << "        Serial number | " << cameraInfo.serialNumber << std::endl;
    std::cout << "         Camera model | " << cameraInfo.modelName << std::endl;
    std::cout << "              isColor | " << cameraInfo.isColorCamera << std::endl;
    std::cout << "        Camera vendor | " << cameraInfo.vendorName << std::endl;
    std::cout << "               Sensor | " << cameraInfo.sensorInfo << std::endl;
    std::cout << "           Resolution | " << cameraInfo.sensorResolution << std::endl;
    std::cout << "     Firmware version | " << cameraInfo.firmwareVersion << std::endl;
    std::cout << "  Firmware build time | " << cameraInfo.firmwareBuildTime << std::endl;
    std::cout << std::endl;
}

bool
singleCamera::getCameraConfiguration( FlyCapture2::Error& error )
{
    error = cam.GetConfiguration( &config );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in GetConfiguration " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::setCameraConfiguration( FlyCapture2::Error& error )
{
    error = cam.SetConfiguration( &config );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setCameraConfiguration " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::startCapture( FlyCapture2::Error& error )
{
    // Start capturing images
    error = cam.StartCapture( );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in StartCapture " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::captureOneImage( FlyCapture2::Error& error, cv::Mat& image )
{
    FlyCapture2::Image rawImage;

    // Retrieve an image
    error = cam.RetrieveBuffer( &rawImage );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        error.PrintErrorTrace( );
        std::cout << "[#INFO]Error in RetrieveBuffer, captureOneImage " << std::endl;
        return false;
    }

    // Create a converted image
    FlyCapture2::Image convertedImage;

    // Convert the raw image
    error = rawImage.Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &convertedImage );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in Convert " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }

    // Change to opencv image Mat
    unsigned char* pdata = convertedImage.GetData( );

    cv::Mat cv_image;
    if ( cameraInfo.isColorCamera )
        cv_image = cv::Mat( 1024, 1280, CV_8UC3, pdata );
    else
        cv_image = cv::Mat( 1024, 1280, CV_8UC1, pdata );

    cv_image.copyTo( image );

    return true;
}

bool
singleCamera::StopCapture( FlyCapture2::Error& error )
{
    // Stop capturing images
    error = cam.StopCapture( );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in StopCapture " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
}

bool
singleCamera::disconnectCamera( FlyCapture2::Error& error )
{
    // Disconnect the camera
    error = cam.Disconnect( );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in Disconnect " << std::endl;
        error.PrintErrorTrace( );
        return -1;
    }
}

bool
singleCamera::setMetadata( FlyCapture2::Error& error )
{
    // Enable metadata
    FlyCapture2::EmbeddedImageInfo info;
    info.timestamp.onOff    = true;
    info.gain.onOff         = true;
    info.shutter.onOff      = true;
    info.brightness.onOff   = true;
    info.exposure.onOff     = true;
    info.whiteBalance.onOff = true;
    info.frameCounter.onOff = true;
    info.ROIPosition.onOff  = true;
    error                   = cam.SetEmbeddedImageInfo( &info );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in GetConfiguration " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::setFrameRate( FlyCapture2::Error& error, float rate )
{
    FlyCapture2::PropertyInfo pInfo;
    pInfo.type = FlyCapture2::FRAME_RATE;
    error      = cam.GetPropertyInfo( &pInfo );

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::FRAME_RATE;
    prop.autoManualMode = ( false && pInfo.autoSupported );
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;
    prop.absValue       = rate;
    error               = cam.SetProperty( &prop );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setFrameRate " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::setBrightness( FlyCapture2::Error& error, float brightness )
{
    FlyCapture2::PropertyInfo pInfo;
    pInfo.type = FlyCapture2::BRIGHTNESS;
    error      = cam.GetPropertyInfo( &pInfo );

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::BRIGHTNESS;
    prop.autoManualMode = ( false && pInfo.autoSupported );
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;
    prop.absValue       = brightness;
    error               = cam.SetProperty( &prop );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setBrightness " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}
