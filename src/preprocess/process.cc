#include "process.h"

preprocess::PreProcess::PreProcess( cv::Size _raw_image_size, cv::Size _roi_size, cv::Point _center, float _resize_scale )
: resize_scale( _resize_scale )
{
    roi_row_start = _center.y - _roi_size.height / 2;
    roi_row_end   = roi_row_start + _roi_size.height;
    roi_col_start = _center.x - _roi_size.width / 2;
    roi_col_end   = roi_col_start + _roi_size.width;

    std::cout << "roi_row_start " << roi_row_start << std::endl;
    std::cout << "roi_row_end   " << roi_row_end << std::endl;
    std::cout << "roi_col_start " << roi_col_start << std::endl;
    std::cout << "roi_col_end   " << roi_col_end << std::endl;
}

cv::Mat
preprocess::PreProcess::do_preprocess( cv::Mat image_input )
{
    cv::Mat image_input_roi = image_input( cv::Range( roi_row_start, roi_row_end ), cv::Range( roi_col_start, roi_col_end ) );

    cv::Mat image_input_resized;
    cv::resize( image_input_roi, image_input_resized, cv::Size( image_input_roi.cols * resize_scale, image_input_roi.rows * resize_scale ) );
    return image_input_resized;
}
