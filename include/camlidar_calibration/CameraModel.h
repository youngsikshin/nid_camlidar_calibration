#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "Datatypes.h"

using namespace std;

namespace camlidar_calib {

class CameraModel
{
public:
    typedef shared_ptr<CameraModel> Ptr;

    CameraModel() {};
    CameraModel(int width, int height) : width_(width), height_(height)
    {

    };

    virtual ~CameraModel() {};

    virtual inline bool is_in_image(Eigen::Vector2f uv, int boundary, float scale);
    virtual inline bool is_in_image(Eigen::Vector2f uv, int boundary) = 0;
    virtual inline bool is_in_image(const Point& point, int boundary) = 0;

    virtual inline Eigen::Vector2f xyz_to_uv(const Point& xyz) = 0;

    virtual void undistort_image(const cv::Mat& raw, cv::Mat& rectified) = 0;

    inline int width() { return width_; }
    inline int height() { return height_; }

protected:
    int width_;
    int height_;
};

}   // camlidar_calib
