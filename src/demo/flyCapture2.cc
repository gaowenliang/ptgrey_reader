#include <flycapture/FlyCapture2.h>
#include <iostream>
#include <sstream>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

using namespace FlyCapture2;
using namespace std;

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
main( int argc, char** argv )
{
    ros::init( argc, argv, "singleReader" );
    ros::NodeHandle nh( "~" );
    ros::Publisher imagePublisher = nh.advertise< sensor_msgs::Image >( "/image_out", 3 );
    ros::Publisher imagePublisher2 = nh.advertise< sensor_msgs::Image >( "/image_out2", 3 );

    Error error;

    // Since this application saves images in the current folder
    // we must ensure that we have permission to write to this folder.
    // If we do not have permission, fail right away.
    FILE* tempFile = fopen( "test.txt", "w+" );
    if ( tempFile == NULL )
    {
        cout << "Failed to create file in current folder.  Please check permissions." << endl;
        return -1;
    }
    fclose( tempFile );
    remove( "test.txt" );

    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras( &numCameras );
    if ( error != PGRERROR_OK )
    {
        PrintError( error );
        return -1;
    }

    cout << "Number of cameras detected: " << numCameras << endl;

    PGRGuid guid1;
    PGRGuid guid2;
    error = busMgr.GetCameraFromSerialNumber( 17221121, &guid1 );
    error = busMgr.GetCameraFromSerialNumber( 17221110, &guid2 );
    if ( error != PGRERROR_OK )
    {
        PrintError( error );
        return -1;
    }
    cv::namedWindow( "cv_image", CV_WINDOW_NORMAL );
    cv::namedWindow( "cv_image2", CV_WINDOW_NORMAL );

    {
        const int k_numImages = 1000;

        Error error;
        Camera cam;
        Camera cam2;

        // Connect to a camera
        error = cam.Connect( &guid1 );
        error = cam2.Connect( &guid2 );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }

        // Get the camera information
        CameraInfo camInfo;
        CameraInfo camInfo2;
        error = cam.GetCameraInfo( &camInfo );
        error = cam2.GetCameraInfo( &camInfo2 );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }
        PrintCameraInfo( &camInfo );
        PrintCameraInfo( &camInfo2 );

        // Get the camera configuration
        FC2Config config;
        FC2Config config2;
        error = cam.GetConfiguration( &config );
        error = cam2.GetConfiguration( &config2 );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }

        // Set the number of driver buffers used to 10.
        config.numBuffers = 10;

        // Set the camera configuration
        error = cam.SetConfiguration( &config );
        error = cam2.SetConfiguration( &config2 );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }

        float rate = 25;
        FlyCapture2::PropertyInfo pInfo;
        pInfo.type = FlyCapture2::FRAME_RATE;
        error      = cam.GetPropertyInfo( &pInfo );
        FlyCapture2::Property prop;
        prop.type           = FlyCapture2::FRAME_RATE;
        prop.autoManualMode = ( false && pInfo.autoSupported );
        prop.absControl     = pInfo.absValSupported;
        prop.onOff          = pInfo.onOffSupported;
        if ( rate < pInfo.absMax )
            prop.absValue = rate;
        else
            prop.absValue = pInfo.absMax;

        error = cam.SetProperty( &prop );
        error = cam2.SetProperty( &prop );
        if ( error != FlyCapture2::PGRERROR_OK )
        {
            std::cout << "[#INFO]Error in setFrameRate " << std::endl;
            error.PrintErrorTrace( );
            return false;
        }

        // Start capturing images
        error = cam.StartCapture( );
        error = cam2.StartCapture( );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }

        Image rawImage;
        Image rawImage2;

        TimeStamp time;
        TimeStamp time2;

        int imageCnt = 0;

        //        unsigned int haha = 5;
        //        int haha2         = 5;

        //        if ( haha == haha2 )
        //            std::cout << "haha == haha2" << std::endl;

        //        while ( 0 )
        while ( ros::ok( ) )
        //                    for ( int imageCnt = 0; imageCnt < k_numImages;
        //                    imageCnt++ )
        {
            ++imageCnt;
            // Retrieve an image
            error = cam.RetrieveBuffer( &rawImage );
            error = cam2.RetrieveBuffer( &rawImage2 );
            if ( error != PGRERROR_OK )
            {
                PrintError( error );
                continue;
            }

            cout << "Grabbed image " << imageCnt << endl;

            // Create a converted image
            Image convertedImage;
            Image convertedImage2;

            // Convert the raw image
            error = rawImage.Convert( PIXEL_FORMAT_MONO8, &convertedImage );
            error = rawImage2.Convert( PIXEL_FORMAT_MONO8, &convertedImage2 );
            if ( error != PGRERROR_OK )
            {
                PrintError( error );
                return -1;
            }

            time              = rawImage.GetTimeStamp( );
            time2             = rawImage2.GetTimeStamp( );
            double timeStamp  = ( double )time.seconds + time.microSeconds / 1000000.0;
            double timeStamp2 = ( double )time2.seconds + time2.microSeconds / 1000000.0;
            // std::cout << "time " << time.seconds << " " << time.microSeconds <<
            // std::endl << "time2 " << time2.seconds << " " << time2.microSeconds <<
            //          std::endl;

            double time_error = timeStamp - timeStamp2;
            std::cout << "time_error " << time_error << std::endl;

            // Create a unique filename

            ostringstream filename;
            ostringstream filename2;
            filename << "FlyCapture2Test-" << camInfo.serialNumber << "-" << imageCnt << ".pgm";
            filename2 << "FlyCapture2Test-" << camInfo2.serialNumber << "-" << imageCnt << ".pgm";

            unsigned char* pdata  = convertedImage.GetData( );
            unsigned char* pdata2 = convertedImage2.GetData( );
            cv::Mat cv_image = cv::Mat( rawImage.GetRows( ), rawImage.GetCols( ), CV_8UC1, pdata );
            cv::Mat cv_image2
            = cv::Mat( rawImage2.GetRows( ), rawImage2.GetCols( ), CV_8UC1, pdata2 );

            cv_bridge::CvImage outImg;
            outImg.header.stamp = ros::Time::now( );
            //            outImg.header.stamp.sec  = cv_image.time.seconds;
            //            outImg.header.stamp.nsec = cv_image.time.microSeconds *
            //            1000;
            outImg.header.frame_id = "frame";

            outImg.encoding = sensor_msgs::image_encodings::MONO8;

            outImg.image = cv_image;
            imagePublisher.publish( outImg );

            //        int step = rawImage.GetStride( ) / 2;
            //        cv::Mat image_test( rawImage.GetRows( ), rawImage.GetCols( ),
            //        CV_8UC1 );
            //        cv::Mat image_test2( rawImage.GetRows( ), rawImage.GetCols( ),
            //        CV_8UC1
            //        );
            //        // Step through the raw data and set each image in turn
            //        for ( size_t i = 0; i < rawImage.GetRows( ); i++ ) // Rows
            //        {
            //            for ( size_t j = 0; j < rawImage.GetCols( );
            //                  j++ ) // Columns that need to have the 16 bits split
            //                  into
            //                  2 8
            //                  bit groups
            //            {
            //                size_t index            = i * step + j;
            //                size_t raw_index        = 2 * index;
            //                image_test.data[index]  = pdata[raw_index];
            //                image_test2.data[index] = pdata[raw_index + 1];
            //            }
            //        }

            cv::imshow( "cv_image", cv_image );
            cv::imshow( "cv_image2", cv_image2 );
            //        cv::imwrite( filename.str( ) + "cv_image.jpg", image_test );
            cv::waitKey( 10 );
        }

        // Stop capturing images
        error = cam.StopCapture( );
        error = cam2.StopCapture( );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }

        // Disconnect the camera
        error = cam.Disconnect( );
        error = cam2.Disconnect( );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }
    }

    cv::destroyAllWindows( );

    cout << "Done! Press Enter to exit..." << endl;
    cin.ignore( );

    return 0;
}
