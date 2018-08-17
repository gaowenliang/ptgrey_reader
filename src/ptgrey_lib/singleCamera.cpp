#include "singleCamera.h"

using namespace ptgrey_reader;

singleCamera::singleCamera( ) { pCamera = new FlyCapture2::Camera( ); }

singleCamera::singleCamera( unsigned int serialNum )
: serialNumber( serialNum )
{
    pCamera = new FlyCapture2::Camera( );
}

singleCamera::~singleCamera( ) { delete pCamera; }

bool
singleCamera::initCamera( FlyCapture2::Error& error, FlyCapture2::BusManager& BusMana )
{
    error = BusMana.GetCameraFromSerialNumber( serialNumber, &guid );

    std::cout << " serialNumber " << serialNumber << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in GetCameraFromSerialNumber." << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
    {
        std::cout << "[#INFO] Connect to Camera " << serialNumber << std::endl;
        return true;
    }
}

bool
singleCamera::reInitCamera( FlyCapture2::Error& error, unsigned int serialNum, FlyCapture2::BusManager& BusMana )
{
    serialNumber = serialNum;

    error = BusMana.GetCameraFromSerialNumber( serialNumber, &guid );

    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in GetCameraFromSerialNumber." << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
    {
        std::cout << "[#INFO] Connect to Camera " << serialNum << std::endl;
        return true;
    }
}

float
singleCamera::getFrameRate( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::FRAME_RATE;
    error      = pCamera->GetProperty( &fProp );
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
    error      = pCamera->GetProperty( &fProp );
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
    error      = pCamera->GetProperty( &fProp );
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
    error      = pCamera->GetProperty( &fProp );
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
    error      = pCamera->GetProperty( &fProp );
    std::cout << " WhiteBalance " << fProp.absValue << " A:" << fProp.valueA
              << " B:" << fProp.valueB << std::endl;

    //    std::cout << " autoManualMode " << fProp.autoManualMode << std::endl;
    //    std::cout << " onOff " << fProp.onOff << std::endl;
    //    std::cout << " onePush " << fProp.onePush << std::endl;

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
    error      = pCamera->GetProperty( &fProp );
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
    error      = pCamera->GetProperty( &fProp );
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
    error      = pCamera->GetProperty( &fProp );
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
    error      = pCamera->GetProperty( &fProp );
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
    error      = pCamera->GetProperty( &fProp );
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
    error      = pCamera->GetProperty( &fProp );
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
    error      = pCamera->GetProperty( &fProp );

    if ( fProp.present == true )
        std::cout << " Trigger present: " << fProp.present
                  << ". Camera support external triggering." << std::endl;
    std::cout << " TriggerMode " << fProp.absValue << std::endl;
    std::cout << " TriggerMode onOff " << fProp.onOff << std::endl;
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
    error      = pCamera->GetProperty( &fProp );
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

float
singleCamera::getCameraTemperature( FlyCapture2::Error& error )
{
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::TEMPERATURE;
    error      = pCamera->GetProperty( &fProp );

    std::cout << " Temperature " << fProp.absValue << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in getCameraTemperature " << std::endl;
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
        ++tryConnectIndex;
        error = pCamera->Connect( &guid );
        if ( error != FlyCapture2::PGRERROR_OK )
        {
            std::cout << "[#INFO] Error in Connect." << std::endl;
            error.PrintErrorTrace( );
            continue;
        }
        else
        {
            if ( pCamera->IsConnected( ) )
            {
                std::cout << "[#INFO] Camera Connected." << std::endl;
                return true;
            }
            else
                continue;
        }
    }
    std::cout << "[#INFO] Cannot connect to the Camera." << std::endl;
    return false;
}

bool
singleCamera::getCameraInfo( FlyCapture2::Error& error )
{
    // Connect to a camera
    error = pCamera->GetCameraInfo( &cameraInfo );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in GetCameraInfo " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
    {
        serialNumber = cameraInfo.serialNumber;
        return true;
    }
}

const unsigned int
singleCamera::getSerialNumber( ) const
{
    return serialNumber;
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
    error = pCamera->GetConfiguration( &cameraConfig );
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
    error = pCamera->SetConfiguration( &cameraConfig );
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
singleCamera::setCameraConfiguration( FlyCapture2::Error& error, FlyCapture2::FC2Config& cfg )
{
    error = pCamera->SetConfiguration( &cfg );
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
    error = pCamera->StartCapture( );

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
singleCamera::captureOneImage( FlyCapture2::Error& error, cv::Mat& image, FlyCapture2::TimeStamp& time )
{
    // Retrieve an image
    error = pCamera->RetrieveBuffer( &rawImage );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        error.PrintErrorTrace( );
        std::cout << "[#INFO]Error in RetrieveBuffer, captureOneImage " << std::endl;
        //        return false;
        // TODO
    }

    time = rawImage.GetTimeStamp( );
    //    std::cout << "time " << time.seconds << " " << time.microSeconds <<
    //    std::endl;

    // Create a converted image
    FlyCapture2::Image convertedImage;

    // Convert the raw image
    if ( cameraInfo.isColorCamera )
        error = rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &convertedImage );
    else
        error = rawImage.Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &convertedImage );

    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in Convert " << std::endl;
        error.PrintErrorTrace( );
        //     return false;
        // TODO
    }

    // Change to opencv image Mat
    unsigned char* pdata = convertedImage.GetData( );

    cv::Mat cv_image;
    if ( cameraInfo.isColorCamera )
        cv_image = cv::Mat( rawImage.GetRows( ), rawImage.GetCols( ), CV_8UC3, pdata );
    else
        cv_image = cv::Mat( rawImage.GetRows( ), rawImage.GetCols( ), CV_8UC1, pdata );

    cv_image.copyTo( image );

    return true;
}

bool
singleCamera::StopCapture( FlyCapture2::Error& error )
{
    // Stop capturing images
    error = pCamera->StopCapture( );
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
    error = pCamera->Disconnect( );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in Disconnect " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

FlyCapture2::PGRGuid&
singleCamera::UniquePGRGuid( )
{
    return guid;
}

FlyCapture2::CameraInfo&
singleCamera::camInfo( )
{
    return cameraInfo;
}

FlyCapture2::FC2Config&
singleCamera::camConfig( )
{
    return cameraConfig;
}

void
singleCamera::setSerialNumber( unsigned int serial )
{
    serialNumber = serial;
}

FlyCapture2::Camera*
singleCamera::getPCamera( ) const
{
    return pCamera;
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
    error                   = pCamera->SetEmbeddedImageInfo( &info );
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
    error      = pCamera->GetPropertyInfo( &pInfo );

    //    std::cout << "in setFrameRate" << std::endl;
    //    std::cout << "    autoSupported   | " << pInfo.autoSupported <<
    //    std::endl;
    //    std::cout << "    manualSupported | " << pInfo.manualSupported <<
    //    std::endl;
    //    std::cout << "    absValSupported | " << pInfo.absValSupported <<
    //    std::endl;
    //    std::cout << "             absMax | " << pInfo.absMax << std::endl;

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::FRAME_RATE;
    prop.autoManualMode = ( false && pInfo.autoSupported );
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;

    if ( rate < pInfo.absMin )
        prop.absValue = pInfo.absMin;
    else if ( rate > pInfo.absMax )
        prop.absValue = pInfo.absMax;
    else
        prop.absValue = rate;

    error = pCamera->SetProperty( &prop );
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
    error      = pCamera->GetPropertyInfo( &pInfo );

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::BRIGHTNESS;
    prop.autoManualMode = ( false && pInfo.autoSupported );
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;

    if ( brightness < pInfo.absMin )
        prop.absValue = pInfo.absMin;
    else if ( brightness > pInfo.absMax )
        prop.absValue = pInfo.absMax;
    else
        prop.absValue = brightness;

    error = pCamera->SetProperty( &prop );
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
    error      = pCamera->GetPropertyInfo( &pInfo );

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::AUTO_EXPOSURE;
    prop.autoManualMode = false;
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;

    if ( exposure < pInfo.absMin )
        prop.absValue = pInfo.absMin;
    else if ( exposure > pInfo.absMax )
        prop.absValue = pInfo.absMax;
    else
        prop.absValue = exposure;

    error = pCamera->SetProperty( &prop );
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
singleCamera::setWhiteBalance( FlyCapture2::Error& error, int WB_red, int WB_Blue )
{
    FlyCapture2::PropertyInfo pInfo;
    pInfo.type = FlyCapture2::WHITE_BALANCE;
    error      = pCamera->GetPropertyInfo( &pInfo );

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::WHITE_BALANCE;
    prop.autoManualMode = true;
    prop.absControl     = false;
    prop.onOff          = pInfo.onOffSupported;

    if ( WB_red < int( pInfo.min ) )
        prop.valueA = pInfo.min;
    else if ( WB_red > int( pInfo.max ) )
        prop.valueA = pInfo.max;
    else
        prop.valueA = uint( WB_red );

    if ( WB_Blue < int( pInfo.min ) )
        prop.valueB = pInfo.min;
    else if ( WB_Blue > int( pInfo.max ) )
        prop.valueB = pInfo.max;
    else
        prop.valueB = uint( WB_Blue );

    error = pCamera->SetProperty( &prop );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setWhiteBalance " << std::endl;
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
    error      = pCamera->GetPropertyInfo( &pInfo );

    // std::cout << "in setGamma" << std::endl;
    // std::cout << "    autoSupported   | " << pInfo.autoSupported << std::endl;
    // std::cout << "    manualSupported | " << pInfo.manualSupported << std::endl;
    // std::cout << "    absValSupported | " << pInfo.absValSupported << std::endl;
    // std::cout << "             absMax | " << pInfo.absMax << std::endl;
    // std::cout << "             absMin | " << pInfo.absMin << std::endl;

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::GAMMA;
    prop.autoManualMode = false;
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;

    if ( gamma < pInfo.absMin )
        prop.absValue = pInfo.absMin;
    else if ( gamma > pInfo.absMax )
        prop.absValue = pInfo.absMax;
    else
        prop.absValue = gamma;

    error = pCamera->SetProperty( &prop );
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
    error      = pCamera->GetPropertyInfo( &pInfo );

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::SHUTTER;
    prop.autoManualMode = true;
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;

    error = pCamera->SetProperty( &prop );
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
singleCamera::setTrigger( FlyCapture2::Error& error )
{
    FlyCapture2::TriggerMode triggerMode;
    error = pCamera->GetTriggerMode( &triggerMode );

    triggerMode.mode     = 0; // 0 means stand trigger
    triggerMode.onOff    = true;
    triggerMode.polarity = true;

    error = pCamera->SetTriggerMode( &triggerMode );

    error = pCamera->GetTriggerMode( &triggerMode );
    if ( triggerMode.onOff == true )
    {
        std::cout << "[#INFO] Trigger setted." << std::endl;
        return true;
    }
    else
        return false;
}

bool
singleCamera::setTriggerOFF( FlyCapture2::Error& error )
{
    FlyCapture2::TriggerMode triggerMode;
    error = pCamera->GetTriggerMode( &triggerMode );

    triggerMode.mode     = 0; // 0 means stand trigger
    triggerMode.onOff    = false;
    triggerMode.polarity = true;

    error = pCamera->SetTriggerMode( &triggerMode );

    error = pCamera->GetTriggerMode( &triggerMode );
    if ( triggerMode.onOff == true )
    {
        std::cout << "[#INFO] Trigger setted." << std::endl;
        return true;
    }
    else
    {
        std::cout << "[#INFO] Trigger OFF." << std::endl;
        return false;
    }
}

bool
singleCamera::setTimeout( FlyCapture2::Error& error, double timeout_ms )
{
    FlyCapture2::FC2Config pConfig;
    error = pCamera->GetConfiguration( &pConfig );

    pConfig.grabTimeout = ( int )( timeout_ms );
    if ( pConfig.grabTimeout < 0.00001 )
    {
        pConfig.grabTimeout = -1; //  no timeout
    }
    error = pCamera->SetConfiguration( &pConfig );
    std::cout << "-set grabTimeout | " << pConfig.grabTimeout << std::endl;
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setTimeout " << std::endl;
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
    error      = pCamera->GetPropertyInfo( &pInfo );

    // std::cout << "in setShutter" << std::endl;
    // std::cout << "    autoSupported   | " << pInfo.autoSupported << std::endl;
    // std::cout << "    manualSupported | " << pInfo.manualSupported << std::endl;
    // std::cout << "    absValSupported | " << pInfo.absValSupported << std::endl;
    // std::cout << "             absMax | " << pInfo.absMax << std::endl;
    // std::cout << "             absMin | " << pInfo.absMin << std::endl;

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::SHUTTER;
    prop.autoManualMode = false;
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;

    if ( shutter < pInfo.absMin )
        prop.absValue = pInfo.absMin;
    else if ( shutter > pInfo.absMax )
        prop.absValue = pInfo.absMax;
    else
        prop.absValue = shutter;

    error = pCamera->SetProperty( &prop );
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
singleCamera::setSharpness( FlyCapture2::Error& error, float sharpness )
{
    FlyCapture2::PropertyInfo pInfo;
    pInfo.type = FlyCapture2::SHARPNESS;
    error      = pCamera->GetPropertyInfo( &pInfo );

    // std::cout << "in setSharpness" << std::endl;
    // std::cout << "    autoSupported   | " << pInfo.autoSupported << std::endl;
    // std::cout << "    manualSupported | " << pInfo.manualSupported << std::endl;
    // std::cout << "    absValSupported | " << pInfo.absValSupported << std::endl;
    // std::cout << "             absMax | " << pInfo.absMax << std::endl;
    // std::cout << "             absMin | " << pInfo.absMin << std::endl;

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::SHARPNESS;
    prop.autoManualMode = false;
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;

    if ( sharpness < pInfo.absMin )
        prop.absValue = pInfo.absMin;
    else if ( sharpness > pInfo.absMax )
        prop.absValue = pInfo.absMax;
    else
        prop.absValue = sharpness;
    std::cout << "    set sharpness as | " << prop.absValue << std::endl;

    error = pCamera->SetProperty( &prop );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setSharpness." << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::setHue( FlyCapture2::Error& error, float hue )
{
    FlyCapture2::PropertyInfo pInfo;
    pInfo.type = FlyCapture2::HUE;
    error      = pCamera->GetPropertyInfo( &pInfo );

    // std::cout << "in setHue" << std::endl;
    // std::cout << "    autoSupported   | " << pInfo.autoSupported << std::endl;
    // std::cout << "    manualSupported | " << pInfo.manualSupported << std::endl;
    // std::cout << "    absValSupported | " << pInfo.absValSupported << std::endl;
    // std::cout << "             absMax | " << pInfo.absMax << std::endl;
    // std::cout << "             absMin | " << pInfo.absMin << std::endl;

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::HUE;
    prop.autoManualMode = false;
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;

    if ( hue < pInfo.absMin )
        prop.absValue = pInfo.absMin;
    else if ( hue > pInfo.absMax )
        prop.absValue = pInfo.absMax;
    else
        prop.absValue = hue;
    std::cout << "    set hue as | " << prop.absValue << std::endl;

    error = pCamera->SetProperty( &prop );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setHue." << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::setSaturation( FlyCapture2::Error& error, float saturation )
{
    FlyCapture2::PropertyInfo pInfo;
    pInfo.type = FlyCapture2::SATURATION;
    error      = pCamera->GetPropertyInfo( &pInfo );

    //  std::cout << "in setSaturation" << std::endl;
    //  std::cout << "    autoSupported   | " << pInfo.autoSupported << std::endl;
    //  std::cout << "    manualSupported | " << pInfo.manualSupported << std::endl;
    //  std::cout << "    absValSupported | " << pInfo.absValSupported << std::endl;
    //  std::cout << "             absMax | " << pInfo.absMax << std::endl;
    //  std::cout << "             absMin | " << pInfo.absMin << std::endl;

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::SATURATION;
    prop.autoManualMode = false;
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;

    if ( saturation < pInfo.absMin )
        prop.absValue = pInfo.absMin;
    else if ( saturation > pInfo.absMax )
        prop.absValue = pInfo.absMax;
    else
        prop.absValue = saturation;
    std::cout << "    set saturation as | " << prop.absValue << std::endl;

    error = pCamera->SetProperty( &prop );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setSaturation." << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::setIris( FlyCapture2::Error& error, float iris )
{
    FlyCapture2::PropertyInfo pInfo;
    pInfo.type = FlyCapture2::IRIS;
    error      = pCamera->GetPropertyInfo( &pInfo );

    // std::cout << "in setIris" << std::endl;
    // std::cout << "    autoSupported   | " << pInfo.autoSupported << std::endl;
    // std::cout << "    manualSupported | " << pInfo.manualSupported << std::endl;
    // std::cout << "    absValSupported | " << pInfo.absValSupported << std::endl;
    // std::cout << "             absMax | " << pInfo.absMax << std::endl;
    // std::cout << "             absMin | " << pInfo.absMin << std::endl;

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::IRIS;
    prop.autoManualMode = false;
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;

    if ( iris < pInfo.absMin )
        prop.absValue = pInfo.absMin;
    else if ( iris > pInfo.absMax )
        prop.absValue = pInfo.absMax;
    else
        prop.absValue = iris;
    std::cout << "    set iris as | " << prop.absValue << std::endl;

    error = pCamera->SetProperty( &prop );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setIris." << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}

bool
singleCamera::setGain( FlyCapture2::Error& error, float gain )
{
    FlyCapture2::PropertyInfo pInfo;
    pInfo.type = FlyCapture2::GAIN;
    error      = pCamera->GetPropertyInfo( &pInfo );

    //  std::cout << "in setGain" << std::endl;
    //  std::cout << "    autoSupported   | " << pInfo.autoSupported << std::endl;
    //  std::cout << "    manualSupported | " << pInfo.manualSupported << std::endl;
    //  std::cout << "    absValSupported | " << pInfo.absValSupported << std::endl;
    //  std::cout << "             absMax | " << pInfo.absMax << std::endl;
    //  std::cout << "             absMin | " << pInfo.absMin << std::endl;

    FlyCapture2::Property prop;
    prop.type           = FlyCapture2::GAIN;
    prop.autoManualMode = false;
    prop.absControl     = pInfo.absValSupported;
    prop.onOff          = pInfo.onOffSupported;

    if ( gain < pInfo.absMin )
        prop.absValue = pInfo.absMin;
    else if ( gain > pInfo.absMax )
        prop.absValue = pInfo.absMax;
    else
        prop.absValue = gain;

    error = pCamera->SetProperty( &prop );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "[#INFO]Error in setGain " << std::endl;
        error.PrintErrorTrace( );
        return false;
    }
    else
        return true;
}
