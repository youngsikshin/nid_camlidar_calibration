#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <glog/logging.h>

#include <camlidar_calibration/Calibration.h>

using namespace std;

int main(int argc, char *argv[]) {

    google::InitGoogleLogging(argv[0]);
    google::SetLogDestination( google::GLOG_INFO, "./Test." );

    FLAGS_alsologtostderr = 1;

    string pkg_path = ros::package::getPath("camlidar_calibration");
    string params_path = pkg_path+"/params/";

    camlidar_calib::Calibration calibration(params_path);

    return 0;

}
