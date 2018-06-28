#ifndef SINGLECAMERAREADER_H
#define SINGLECAMERAREADER_H

#include "singleCamera.h"
#include <opencv2/opencv.hpp>

namespace ptgrey_reader
{

class singleCameraReader
{
    public:
    singleCameraReader( );
    singleCameraReader( const unsigned int serial_num );
    ~singleCameraReader( );

    public:
    unsigned int getConnectCameraNum( );
    void setCameraProperty( double frameRate,
                            double brightness,
                            double exposure,
                            double gain,
                            bool is_auto_shutter,
                            double shutter,
                            int WB_red,
                            int WB_Blue,
                            double aturation,
                            double hue,
                            double sharpness,
                            bool is_sync = false );
    void printCameraProperty( );

    bool startCamera( unsigned int serialNum,
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
                      bool is_print_info = false,
                      bool is_sync       = false );
    cvImage grabImage( );
    void stopCamera( );

    FlyCapture2::BusManager& BusManager( );
    singleCamera& Camera( );
    const int cameraNum( ) const;

    private:
    unsigned int cameraNumber;
    singleCamera camera;
    FlyCapture2::Error error;
    FlyCapture2::BusManager busMgr;
};
}
#endif // SINGLECAMERAREADER_H
