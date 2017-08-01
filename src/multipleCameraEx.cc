
//#include "FlyCapture2.h"
#include <flycapture/FlyCapture2.h>
#include <iostream>
#include <sstream>

using namespace FlyCapture2;
using namespace std;

void
PrintBuildInfo( )
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion( &fc2Version );

    ostringstream version;
    version << "FlyCapture2 library version: " << fc2Version.major << "."
            << fc2Version.minor << "." << fc2Version.type << "." << fc2Version.build;
    cout << version.str( ) << endl;

    ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
    cout << timeStamp.str( ) << endl << endl;
}

void
PrintCameraInfo( CameraInfo* pCamInfo )
{
    cout << endl;
    cout << "*** CAMERA INFORMATION ***" << endl;
    cout << "Serial number -" << pCamInfo->serialNumber << endl;
    cout << "Camera model - " << pCamInfo->modelName << endl;
    cout << "Camera vendor - " << pCamInfo->vendorName << endl;
    cout << "Sensor - " << pCamInfo->sensorInfo << endl;
    cout << "Resolution - " << pCamInfo->sensorResolution << endl;
    cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
    cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl << endl;
}

void
PrintError( Error error )
{
    error.PrintErrorTrace( );
}

int
main( int /*argc*/, char** /*argv*/ )
{
    PrintBuildInfo( );

    const int k_numImages = 100;
    Error error;

    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras( &numCameras );
    if ( error != PGRERROR_OK )
    {
        PrintError( error );
        return -1;
    }

    cout << "Number of cameras detected: " << numCameras << endl;

    if ( numCameras < 1 )
    {
        cout << "Insufficient number of cameras... press Enter to exit." << endl;
        ;
        cin.ignore( );
        return -1;
    }

    Camera** ppCameras = new Camera*[numCameras];

    // Connect to all detected cameras and attempt to set them to
    // a common video mode and frame rate
    for ( unsigned int i = 0; i < numCameras; i++ )
    {
        ppCameras[i] = new Camera( );

        PGRGuid guid;
        error = busMgr.GetCameraFromIndex( i, &guid );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }

        // Connect to a camera
        error = ppCameras[i]->Connect( &guid );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }

        // Get the camera information
        CameraInfo camInfo;
        error = ppCameras[i]->GetCameraInfo( &camInfo );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }

        PrintCameraInfo( &camInfo );

        // Set all cameras to a specific mode and frame rate so they
        // can be synchronized

        error = ppCameras[i]->SetVideoModeAndFrameRate( VIDEOMODE_640x480Y8, FRAMERATE_15 );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            cout << "Error starting SetVideoModeAndFrameRate cameras. " << endl;
            cout << "This example requires cameras to be able to set to 640x480          "
                    "  Y8 at "
                    "        30fps. "
                 << endl;
            cout << "If your camera does not support this mode, please edit the"
                    "  source "
                    "code and recompile the application. "
                 << endl;
            cout << "Press Enter to exit. " << endl;

            cin.ignore( );
            return -1;
        }
    }

    error = Camera::StartSyncCapture( numCameras, ( const Camera** )ppCameras );
    if ( error != PGRERROR_OK )
    {
        PrintError( error );
        cout << "Error starting cameras. " << endl;
        cout << "This example requires cameras to be able to set to 640x480 Y8 at 30fps. " << endl;
        cout << "If your camera does not support this mode, please edit the source code "
                "and recompile the application. "
             << endl;
        cout << "Press Enter to exit. " << endl;

        cin.ignore( );
        return -1;
    }

    for ( int j = 0; j < k_numImages; j++ )
    {
        // Display the timestamps for all cameras to show that the image
        // capture is synchronized for each image
        for ( unsigned int i = 0; i < numCameras; i++ )
        {
            Image image;
            error = ppCameras[i]->RetrieveBuffer( &image );
            if ( error != PGRERROR_OK )
            {
                PrintError( error );
                return -1;
            }

            TimeStamp timestamp = image.GetTimeStamp( );
            cout << "Cam " << i << " - Frame " << j << " - TimeStamp ["
                 << timestamp.cycleSeconds << " " << timestamp.cycleCount << "]" << endl;
        }
    }

    for ( unsigned int i = 0; i < numCameras; i++ )
    {
        ppCameras[i]->StopCapture( );
        ppCameras[i]->Disconnect( );
        delete ppCameras[i];
    }

    delete[] ppCameras;

    cout << "Done! Press Enter to exit..." << endl;
    cin.ignore( );

    return 0;
}
