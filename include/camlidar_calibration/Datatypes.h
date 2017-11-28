#pragma once

#include <vector>
#include <memory>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;

namespace camlidar_calib
{

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;

// float/double, determines numeric precision
typedef float NumType;

typedef Eigen::Matrix<NumType, 60, 60> Matrix60x60;
typedef Eigen::Matrix<NumType, 30, 30> Matrix30x30;
typedef Eigen::Matrix<NumType, 10, 10> Matrix10x10;
typedef Eigen::Matrix<NumType, 6, 6> Matrix6x6;

typedef Eigen::Matrix<NumType, 3, 4> Matrix3x4;
typedef Eigen::Matrix<NumType, 3, 6> Matrix3x6;

typedef Eigen::Matrix<NumType, 2, 6> Matrix2x6;
typedef Eigen::Matrix<NumType, 2, 3> Matrix2x3;

typedef Eigen::Matrix<NumType, 1, 12> Matrix1x12;
typedef Eigen::Matrix<NumType, 1, 6> Matrix1x6;
typedef Eigen::Matrix<NumType, 1, 2> Matrix1x2;

typedef Eigen::Matrix<NumType, 60, 1> Vector60;
typedef Eigen::Matrix<NumType, 30, 1> Vector30;
typedef Eigen::Matrix<NumType, 12, 1> Vector12;
typedef Eigen::Matrix<NumType, 6, 1> Vector6;
typedef Eigen::Matrix<NumType, 4, 1> Vector4;

typedef Eigen::Transform<NumType,3, Eigen::Affine> AffineTransform;

typedef vector<cv::Mat> ImgPyramid;

} // namespace camlidar_calib
