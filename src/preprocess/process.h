#ifndef _FISHEYE_IMAGE_PREPROCESS
#define _FISHEYE_IMAGE_PREPROCESS

#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace preprocess
{
class PreProcess
{
    public:
    PreProcess( );
    PreProcess( cv::Size _raw_image_size, cv::Size _roi_size, cv::Point _center, float _resize_scale );

    cv::Mat do_preprocess( cv::Mat image_input );

    float resize_scale;
    int roi_row_start;
    int roi_col_start;
    int roi_row_end;
    int roi_col_end;
};
}
#endif //_FISHEYE_IMAGE_PREPROCESS
