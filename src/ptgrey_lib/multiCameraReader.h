#ifndef MULTICAMERAREADER_H
#define MULTICAMERAREADER_H

#include "multiCamera.h"

namespace ptgrey_reader
{

class multiCameraReader
{
    public:
    multiCameraReader( ) {}
    multiCameraReader( const std::vector< unsigned int > serialNums )
    //    : cameras( serialNums )
    {
        cameraNumber = int( serialNums.size( ) );
        pcameras     = new multiCamera( serialNums );
        imageWithStamp.clear( );
        imageWithStamp.resize( cameraNumber );
    }
    ~multiCameraReader( ) {}

    public:
    unsigned int getConnectCameraNum( );

    void setCameraProperty( const double frameRate,
                            const double brightness,
                            const double exposure,
                            const double gain,
                            const bool is_auto_shutter,
                            const double shutter,
                            const bool is_sync );

    void printCameraProperty( );
    bool startCamera( const std::vector< unsigned int > serialNum,
                      const double frameRate,
                      const double brightness,
                      const double exposure,
                      const double gain,
                      const bool is_auto_shutter,
                      const double shutter,
                      bool is_print_info   = false,
                      bool is_synchronized = false );
    bool grabImage( std::vector< ptgrey_reader::cvImage >& cv_images );
    void stopCamera( );

    FlyCapture2::BusManager& BusManager( ) { return busMgr; }
    multiCamera* Cameras( ) { return pcameras; }
    const int cameraNum( ) const { return cameraNumber; }

    private:
    std::vector< std::pair< cv::Mat, FlyCapture2::TimeStamp > > imageWithStamp;
    unsigned int cameraNumber;
    multiCamera* pcameras;
    FlyCapture2::Error error;
    FlyCapture2::BusManager busMgr;
};
}
#endif // MULTICAMERAREADER_H
