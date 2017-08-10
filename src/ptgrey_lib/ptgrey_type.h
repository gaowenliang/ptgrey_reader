#ifndef PTGREY_TYPE_H
#define PTGREY_TYPE_H

#include <flycapture/FlyCapture2.h>
#include <opencv2/core.hpp>

namespace ptgrey_reader
{

class cvImage
{
    public:
    cvImage( ) {}
    ~cvImage( ) { image.release( ); }

    cv::Mat image;
    FlyCapture2::TimeStamp time;
};
}

#endif // PTGREY_TYPE_H
