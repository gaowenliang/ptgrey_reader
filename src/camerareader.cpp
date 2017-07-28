#include "camerareader.h"

unsigned int
singleCameraReader::getCameraNum( )
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
    error      = camera.GetProperty( &fProp );
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
    error      = camera.GetProperty( &fProp );
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
    error      = camera.GetProperty( &fProp );
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
    error      = camera.GetProperty( &fProp );
    std::cout << " Sharpness " << fProp.absValue << std::endl;
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
    error      = camera.GetProperty( &fProp );
    std::cout << " WhiteBalance " << fProp.absValue << " A:" << fProp.valueA << " B:" << fProp.valueB << std::endl;

    std::cout << " autoManualMode " << fProp.autoManualMode << std::endl;
    std::cout << " onOff " << fProp.onOff << std::endl;
    std::cout << " onePush " << fProp.onePush << std::endl;

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
    error      = camera.GetProperty( &fProp );
    std::cout << " Hue " << fProp.absValue << std::endl;
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
    error      = camera.GetProperty( &fProp );
    std::cout << " Saturation " << fProp.absValue << std::endl;
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
    error      = camera.GetProperty( &fProp );
    std::cout << " Gamma " << fProp.absValue << std::endl;
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
    error      = camera.GetProperty( &fProp );
    std::cout << " Iris " << fProp.absValue << std::endl;
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
    error      = camera.GetProperty( &fProp );
    std::cout << " Shutter " << fProp.absValue << std::endl;
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
    error      = camera.GetProperty( &fProp );
    std::cout << " Gain " << fProp.absValue << std::endl;
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
    error      = camera.GetProperty( &fProp );
    std::cout << " TriggerMode " << fProp.absValue << std::endl;
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
    error      = camera.GetProperty( &fProp );
    std::cout << " TriggerDelay " << fProp.absValue << std::endl;
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
    int maxTryConnectTimes = 5;
    int tryConnectIndex    = 0;

    while ( tryConnectIndex < maxTryConnectTimes )
    {
        // Connect to a camera
        error = camera.Connect( &guid );
        if ( error != FlyCapture2::PGRERROR_OK )
        {
            std::cout << "[#INFO] Error in Connect." << std::endl;
            error.PrintErrorTrace( );
            continue;
        }
        else
        {
            if ( camera.IsConnected( ) )
            {
                std::cout << "[#INFO] Camera Connected." << std::endl;
                return true;
            }
            else
                continue;
        }
        ++tryConnectIndex;
    }
    std::cout << "[#INFO] Cannot connect to the Camera." << std::endl;
    return false;
}

bool
singleCamera::getCameraInfo( FlyCapture2::Error& error )
{
    // Connect to a camera
    error = camera.GetCameraInfo( &cameraInfo );
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
    error = camera.GetConfiguration( &cameraConfig );
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
    error = camera.SetConfiguration( &cameraConfig );
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
    error = camera.StartCapture( );
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
    error = camera.RetrieveBuffer( &rawImage );
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
    error = camera.StopCapture( );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in StopCapture " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::disconnectCamera( FlyCapture2::Error& error )
{
    // Disconnect the camera
    error = camera.Disconnect( );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in Disconnect " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
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
    error                   = camera.SetEmbeddedImageInfo( &info );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setMetadata." << std::endl;
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
    error      = camera.GetPropertyInfo( &pInfo );

    //    std::cout << "in setFrameRate" << std::endl;
    //    std::cout << "    autoSupported   | " << pInfo.autoSupported << std::endl;
    //    std::cout << "    manualSupported | " << pInfo.manualSupported << std::endl;
    //    std::cout << "    absValSupported | " << pInfo.absValSupported << std::endl;
    //    std::cout << "             absMax | " << pInfo.absMax << std::endl;

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::FRAME_RATE;
    prop.autoManualMode = ( false && pInfo.autoSupported );
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;
    if ( rate < pInfo.absMax )
        prop.absValue = rate;
    else
        prop.absValue = pInfo.absMax;

    error = camera.SetProperty( &prop );
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
    error      = camera.GetPropertyInfo( &pInfo );

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::BRIGHTNESS;
    prop.autoManualMode = ( false && pInfo.autoSupported );
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;
    if ( brightness < pInfo.absMax )
        prop.absValue = brightness;
    else
        prop.absValue = pInfo.absMax;

    error = camera.SetProperty( &prop );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setBrightness " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::setAutoExposure( FlyCapture2::Error& error, float exposure )
{
    FlyCapture2::PropertyInfo pInfo;
    pInfo.type = FlyCapture2::AUTO_EXPOSURE;
    error      = camera.GetPropertyInfo( &pInfo );

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::AUTO_EXPOSURE;
    prop.autoManualMode = false;
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;
    if ( exposure < pInfo.absMax )
        prop.absValue = exposure;
    else
        prop.absValue = pInfo.absMax;

    error = camera.SetProperty( &prop );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setAutoExposure " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::setGamma( FlyCapture2::Error& error, float gamma )
{
    FlyCapture2::PropertyInfo pInfo;
    pInfo.type = FlyCapture2::GAMMA;
    error      = camera.GetPropertyInfo( &pInfo );

    //    std::cout << "in setGamma" << std::endl;
    //    std::cout << "    autoSupported   | " << pInfo.autoSupported << std::endl;
    //    std::cout << "    manualSupported | " << pInfo.manualSupported << std::endl;
    //    std::cout << "    absValSupported | " << pInfo.absValSupported << std::endl;
    //    std::cout << "             absMax | " << pInfo.absMax << std::endl;

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::GAMMA;
    prop.autoManualMode = false;
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;
    if ( gamma < pInfo.absMax )
        prop.absValue = gamma;
    else
        prop.absValue = pInfo.absMax;

    error = camera.SetProperty( &prop );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setGamma." << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::setShutterAuto( FlyCapture2::Error& error )
{
    FlyCapture2::PropertyInfo pInfo;
    pInfo.type = FlyCapture2::SHUTTER;
    error      = camera.GetPropertyInfo( &pInfo );

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::SHUTTER;
    prop.autoManualMode = true;
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;

    error = camera.SetProperty( &prop );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setShutterAuto." << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::isColorCamera( )
{
    return cameraInfo.isColorCamera;
}

bool
singleCamera::setShutter( FlyCapture2::Error& error, float shutter )
{
    FlyCapture2::PropertyInfo pInfo;
    pInfo.type = FlyCapture2::SHUTTER;
    error      = camera.GetPropertyInfo( &pInfo );

    // std::cout << "in setShutter" << std::endl;
    // std::cout << "    autoSupported   | " << pInfo.autoSupported << std::endl;
    // std::cout << "    manualSupported | " << pInfo.manualSupported << std::endl;
    // std::cout << "    absValSupported | " << pInfo.absValSupported << std::endl;
    // std::cout << "             absMax | " << pInfo.absMax << std::endl;

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::SHUTTER;
    prop.autoManualMode = false;
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;
    if ( shutter < pInfo.absMax )
        prop.absValue = shutter;
    else
        prop.absValue = pInfo.absMax;

    error = camera.SetProperty( &prop );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setShutter." << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::setGain( FlyCapture2::Error& error, float exposure )
{
    FlyCapture2::PropertyInfo pInfo;
    pInfo.type = FlyCapture2::GAIN;
    error      = camera.GetPropertyInfo( &pInfo );

    // std::cout << "in setGain" << std::endl;
    // std::cout << "    autoSupported   | " << pInfo.autoSupported << std::endl;
    // std::cout << "    manualSupported | " << pInfo.manualSupported << std::endl;
    // std::cout << "    absValSupported | " << pInfo.absValSupported << std::endl;
    // std::cout << "             absMax | " << pInfo.absMax << std::endl;

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::GAIN;
    prop.autoManualMode = false;
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;
    if ( exposure < pInfo.absMax )
        prop.absValue = exposure;
    else
        prop.absValue = pInfo.absMax;

    error = camera.SetProperty( &prop );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setGain " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}
