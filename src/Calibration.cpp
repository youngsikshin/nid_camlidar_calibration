#include <camlidar_calibration/Calibration.h>

Calibration::Calibration(string param_path)
  :param_path_(param_path)
{
    cv::FileStorage f_ros_settings(param_path_+"KITTI00-02.yaml", cv::FileStorage::READ);

    string path = string(f_ros_settings["Data.path"]);

    float width = f_ros_settings["Camera.width"];
    float height = f_ros_settings["Camera.height"];
    float fx = f_ros_settings["Camera.fx"];
    float fy = f_ros_settings["Camera.fy"];
    float cx = f_ros_settings["Camera.cx"];
    float cy = f_ros_settings["Camera.cy"];
    float k1 = f_ros_settings["Camera.k1"];
    float k2 = f_ros_settings["Camera.k2"];
    float p1 = f_ros_settings["Camera.p1"];
    float p2 = f_ros_settings["Camera.p2"];
    float d0 = f_ros_settings["Camera.d0"];
    float d1 = f_ros_settings["Camera.d1"];
    float d2 = f_ros_settings["Camera.d2"];
    float d3 = f_ros_settings["Camera.d3"];
    float d4 = f_ros_settings["Camera.d4"];
    int RGB_sel = f_ros_settings["Camera.RGB"];

    cv::Mat T;
    f_ros_settings["extrinsicMatrix"] >> T;

    camlidar_calib::Matrix3x4 extrinsics_;

    cv::cv2eigen(T, extrinsics_);

    FileLoader file_loader(path);
    file_loader.lidar(0);
    LOG(INFO) << "[CamLidarCalibration]\t Parameter Path : " << path << endl;

}

Calibration::~Calibration()
{

}
