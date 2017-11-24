#pragma once
#include <string>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <camlidar_calibration/Datatypes.h>
#include <camlidar_calibration/FileLoader.h>

using namespace std;

class Calibration
{
public:
    Calibration(string param_path);
    ~Calibration();

private:
    string param_path_;
};
