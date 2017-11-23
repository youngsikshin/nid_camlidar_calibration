#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/core/core.hpp>

#include <glog/logging.h>
#include <camlidar_calibration/FileLoader.h>

using namespace std;

int main(int argc, char *argv[]) {

    google::InitGoogleLogging(argv[0]);
    google::SetLogDestination( google::GLOG_INFO, "./Test." );

    FLAGS_alsologtostderr = 1;


    string pkg_path = ros::package::getPath("camlidar_calibration");
    string params_path = pkg_path+"/params/";
    cv::FileStorage f_ros_settings(params_path+"KITTI00-02.yaml", cv::FileStorage::READ);

    string path = string(f_ros_settings["Data.path"]);

    FileLoader file_loader(path);
    file_loader.lidar(0);
    LOG(INFO) << "[CamLidarCalibration]\t Parameter Path : " << path << endl;

}
