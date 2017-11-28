#include <camlidar_calibration/Calibration.h>

namespace camlidar_calib {

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

    cv::cv2eigen(T, extrinsics_);

    // Data loading
    file_loader_ = FileLoader(path);
    num_data_ = file_loader_.num_data();
    for(int i=0; i<num_data_; ++i) {
        images_.push_back(file_loader_.image(i));
        pc_.push_back(file_loader_.lidar(i));
    }

    Eigen::Matrix4f extrinsic;
    extrinsic.block<3, 4>(0, 0) = extrinsics_;

    Eigen::Matrix3f rot_pert;
    rot_pert = Eigen::AngleAxisf(5.0/180.0*M_PI, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rot = extrinsics_.block<3, 3>(0,0);
    rot.noalias() = rot*rot_pert;

    Eigen::Vector3f twc = extrinsics_.block<3, 1>(0,3); // for kitti

    Sophus::SE3d sophus_twc(rot.cast<double>(), twc.cast<double>());

    camera_.reset(new PinholeModel(width, height, fx, fy, cx, cy, d0, d1, d2, d3, d4));

    camlidar_calib::PointCloud pointcloud;
    pcl::transformPointCloud(pointcloud, pointcloud, extrinsic);

    LOG(INFO) << "[CamLidarCalibration]\t Parameter Path : " << path << endl;

    cerr << "[CamLidarCalibration]\t extrinsic_ " << endl << extrinsic << endl;
    cerr << "[CamLidarCalibration]\t sophus_twc " << endl << sophus_twc.matrix() << endl;

//    for(int i=0; i<7; ++i)
//      cerr << *(sophus_twc.data()+i) << endl;

    Sophus::SE3Group<double> q = Eigen::Map< const Sophus::SE3Group<double> >(sophus_twc.data());

//    for(int i=0; i<7; ++i)
//      cerr << *(q.data()+i) << endl;

    extrinsics_se3_ = q;
    cerr << extrinsics_.matrix() << endl;
    cerr << extrinsics_se3_.matrix() << endl << endl;


//    optimizer_sophusSE3();
    axis_ = new double[6];
    auto extrinsic_iso = util::sophusToIso(extrinsics_se3_);
    util::isoToAngleAxis(extrinsic_iso, &axis_[0]);

//    optimizer_ceresAngleAxis();
    optimizer_sophusSE3();
}

Calibration::~Calibration()
{

}

void Calibration::optimizer_ceresAngleAxis()
{
    ceres::Problem problem;

    for(int i=0; i<num_data_;++i) {
        ceres::CostFunction* cost_function = NIDErrorCeresAxisAngle::Create(camera_, images_[i], pc_[i]);
        problem.AddResidualBlock(cost_function, NULL, axis_);
    }

    ceres::Solver::Options options;
    options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon() * 1e-100;
    options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon() * 1e-100;
    options.parameter_tolerance = 1e-100;

    //    options.inner_iteration_tolerance

    options.linear_solver_type = ceres::DENSE_QR;  // DENSE_SCHUR, DENSE_QR
    options.minimizer_type = ceres::LINE_SEARCH;  // ceres::TRUST_REGION
    options.line_search_direction_type = ceres::STEEPEST_DESCENT; //STEEPEST_DESCENT, BFGS
    //    options.function_tolerance
    options.max_num_iterations = 1000;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
}

void Calibration::optimizer_sophusSE3()
{
//    cerr << file_loader_.num_data() << endl;
    ceres::Problem problem;

//    problem.AddParameterBlock(extrinsics_se3_.data(), Sophus::SE3d::num_parameters,
//                              new Sophus::test::LocalParameterizationSE3);

    for(int i=0; i<num_data_; ++i) {
        ceres::CostFunction* cost_function = NIDError::Create(camera_, images_[i],pc_[i]);
        problem.AddResidualBlock(cost_function, NULL, extrinsics_se3_.data());
    }

    ceres::Solver::Options options;
    options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon() * 1e-100;
    options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon() * 1e-100;
    options.parameter_tolerance = 1e-100;
//    options.inner_iteration_tolerance

    options.linear_solver_type = ceres::DENSE_QR;  // DENSE_SCHUR, DENSE_QR
    options.minimizer_type = ceres::LINE_SEARCH;  // ceres::TRUST_REGION
    options.line_search_direction_type = ceres::STEEPEST_DESCENT; //STEEPEST_DESCENT, BFGS
//    options.function_tolerance
    options.max_num_iterations = 1000;
    options.minimizer_progress_to_stdout = true;
    options.update_state_every_iteration = true;

    ceres::LocalParameterization* param = Sophus::test::getParameterization(true);
    problem.SetParameterization(extrinsics_se3_.data(), param);

    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    cerr << extrinsics_se3_.matrix() << endl << endl;

}

} //  camlidar_calib
