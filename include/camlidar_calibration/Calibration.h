#pragma once

#include <string>
#include <cmath>
#include <typeinfo>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <sophus/se3.hpp>

#include <camlidar_calibration/Datatypes.h>
#include <camlidar_calibration/FileLoader.h>
#include <camlidar_calibration/PinholeModel.h>
#include "local_parameterization_se3.hpp"

#include "ceres/ceres.h"
#include "ceres/types.h"
#include "ceres/rotation.h"
#include "ceres/autodiff_cost_function.h"

namespace Eigen {
namespace internal {

template <class T, int N, typename NewType>
struct cast_impl<ceres::Jet<T, N>, NewType> {
  EIGEN_DEVICE_FUNC
  static inline NewType run(ceres::Jet<T, N> const& x) {
    return static_cast<NewType>(x.a);
  }
};

}  // namespace internal
}  // namespace Eigen

template<typename T>
struct JetOps {
  static bool IsScalar() {
    return true;
  }
  static T GetScalar(const T& t) {
    return t;
  }
  static void SetScalar(const T& scalar, T* t) {
    *t = scalar;
  }
  static void ScaleDerivative(double scale_by, T *value) {
    // For double, there is no derivative to scale.
  }
};

template<typename T, int N>
struct JetOps<ceres::Jet<T, N> > {
  static bool IsScalar() {
    return false;
  }
  static T GetScalar(const ceres::Jet<T, N>& t) {
    return t.a;
  }
  static void SetScalar(const T& scalar, ceres::Jet<T, N>* t) {
    t->a = scalar;
  }
  static void ScaleDerivative(double scale_by, ceres::Jet<T, N> *value) {
    value->v *= scale_by;
  }
};

namespace util
{

template <typename T>
static ceres::MatrixAdapter<T, 1, 4> ColumnMajorAdapter4x3(T* pointer) {
    return ceres::MatrixAdapter<T, 1, 4>(pointer);
}

static void isoToAngleAxis(const Eigen::Isometry3d& pose, double* cam){
//    Matrix<const double,3,3> rot(pose.linear());
//    cout<<"rotation : "<<pose.linear().data()<<endl;
//    auto begin = pose.linear().data();
    ceres::RotationMatrixToAngleAxis(ColumnMajorAdapter4x3(pose.linear().data()), cam);
    Eigen::Vector3d t(pose.translation());
    cam[3]=t.x();
    cam[4]=t.y();
    cam[5]=t.z();
}

static Eigen::Isometry3d axisAngleToIso(const double* cam){
    Eigen::Isometry3d poseFinal = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d rot;
    ceres::AngleAxisToRotationMatrix(cam,rot.data());
    poseFinal.linear() = rot;
    poseFinal.translation() = Eigen::Vector3d(cam[3],cam[4],cam[5]);
    return poseFinal;//.cast<float>();
}

static Sophus::SE3d isoToSophus(const Eigen::Isometry3d& pose){
    return Sophus::SE3d(pose);
}

static Eigen::Isometry3d sophusToIso(Sophus::SE3d soph){
    //    return Isometry3d(soph.matrix());
    Eigen::Isometry3d poseFinal = Eigen::Isometry3d::Identity();
    poseFinal.linear() = soph.rotationMatrix();
    poseFinal.translation() = soph.translation();
    return poseFinal;
}
}

namespace camlidar_calib
{
using namespace std;

class Probability
{
public:
    Probability () {}
    Probability (int n)
    {
        joint_prob_.resize(n, n);
        ref_prob_.resize(n, 1);
        gray_prob_.resize(n, 1);

        joint_prob_.setZero();
        ref_prob_.setZero();
        gray_prob_.setZero();
    }

    ~Probability () {}

    void gray_probability (int n, double p)
    {
        gray_prob_(n, 0) = p;
    }
    void ref_probability (int n, double p)
    {
        ref_prob_(n, 0) = p;
    }
    void joint_probability (int gn, int rn, double p)
    {
        joint_prob_(gn, rn) = p;
    }

    double gray_probability(int n) { return gray_prob_(n,0); }
    double ref_probability(int n) { return ref_prob_(n,0); }
    double joint_probability(int gray, int ref) { return joint_prob_(gray, ref); }

private:
    Eigen::MatrixXd joint_prob_;
    Eigen::MatrixXd ref_prob_;
    Eigen::MatrixXd gray_prob_;
};

class Histogram
{
public:
    Histogram () {}
    Histogram (int n)
    {
        joint_hist_.resize(n, n);
        ref_hist_.resize(n, 1);
        gray_hist_.resize(n, 1);

        joint_hist_.setZero();
        ref_hist_.setZero();
        gray_hist_.setZero();

        num_ = 0;
    }
    ~Histogram () {}

    void add_hist(int gray, int ref) {
//        cerr << gray << ", " << ref << endl;
        gray_hist_(gray, 0) = gray_hist_(gray, 0) + 1.0;
        ref_hist_(ref, 0) = ref_hist_(ref, 0) + 1.0;
        joint_hist_(gray,ref) = joint_hist_(gray,ref) + 1.0;

        ++num_;

    }

    double gray_hist(int gray) {
        return gray_hist_(gray, 0);
    }

    double ref_hist(int ref) {
        return ref_hist_(ref, 0);
    }

    double joint_hist(int gray, int ref) {
        return joint_hist_(gray,ref);
    }

    int num() {
        return num_;
    }

private:
    Eigen::MatrixXd joint_hist_;
    Eigen::MatrixXd ref_hist_;
    Eigen::MatrixXd gray_hist_;

    int num_;

};

struct NumeriNIDErrorCeresAxisAngleFull {
    vector<cv::Mat> images_;
    vector<PointCloud> lidars_;
    CameraModel::Ptr camera_;
    PinholeModel::Ptr pinhole_model_;

    NumeriNIDErrorCeresAxisAngleFull(CameraModel::Ptr camera, const vector<cv::Mat>& image, const vector<PointCloud>& lidar)
    {
        for(int i=0; i<image.size(); ++i) {
            cv::Mat img;
            image[i].copyTo(img);
            images_.push_back((img));

            PointCloud pc(lidar[i]);
            lidars_.push_back(pc);

        }

        camera_ = camera;
        pinhole_model_ = static_pointer_cast<PinholeModel> (camera_);
    }

    static ceres::CostFunction* Create(CameraModel::Ptr camera, const vector<cv::Mat>& image, const vector<PointCloud>& lidar)
    {
        return (new ceres::NumericDiffCostFunction<NumeriNIDErrorCeresAxisAngleFull, ceres::CENTRAL, 1, 6>(new NumeriNIDErrorCeresAxisAngleFull(camera, image, lidar)));
    }

    bool operator() (const double* extrinsic, double* residuals) const {
        Histogram hist(256);
        Probability prob(256);

        double* p_extrinsic = new double[6];
        for(int i=0; i<6; ++i)
            p_extrinsic[i] = extrinsic[i];

        Eigen::Isometry3d Tcl = util::axisAngleToIso(p_extrinsic);

        for(int i=0; i<images_.size(); ++i) {
            cv::Mat test;
            cvtColor(images_[i], test, cv::COLOR_GRAY2BGR);


            PointCloud lidar_c;
            pcl::transformPointCloud(lidars_[i], lidar_c, Tcl.matrix());

            for(auto point:lidar_c) {

                auto uv = camera_->xyz_to_uv(point);

                if(!camera_->is_in_image(uv, 2) || point.z <= 0.0 || point.z > 30.0)
                    continue;

                if (point.intensity < 0.08 || point.intensity > 0.39)
                    continue;

                const float u_f = uv(0);
                const float v_f = uv(1);
                const int u_i = static_cast<int> (u_f);
                const int v_i = static_cast<int> (v_f);

                int gray_val = (int) images_[i].at<uint8_t>(v_i, u_i);
                int ref_val = (int) (point.intensity*255.0);

                hist.add_hist(gray_val, ref_val);

                cv::circle(test, cv::Point(u_i, v_i), 1.5, cv::Scalar( 1.0, 0.0, 0.0 ), -1);
            }

            cv::namedWindow("test",cv::WINDOW_AUTOSIZE);
            cv::imshow("test",test);
            cv::waitKey(1);
        }

        for (int i=0; i<256; ++i) {
            for (int j=0; j<256; ++j) {
                double joint_p = hist.joint_hist(i,j) / hist.num();

                double ref_p = hist.ref_hist(j) / hist.num();

                prob.joint_probability(i, j, joint_p);
                prob.ref_probability(j, ref_p);
//                cerr << joint_p << ", " << ref_p << endl;
            }
            double gray_p = hist.gray_hist(i) / hist.num();

            prob.gray_probability(i, gray_p);
        }

        double H_gray = 0.0;
        double H_ref = 0.0;
        double H_joint = 0.0;

        for (int i=0; i<256; ++i) {
            for (int j=0; j<256; ++j) {
                double p_joint = prob.joint_probability(i, j);

                if (p_joint > 0.0)  H_joint -= p_joint * log(p_joint);
            }

            double p_gray = prob.gray_probability(i);
            double p_ref = prob.ref_probability(i);

            if (p_gray > 0.0)    H_gray -= p_gray * log(p_gray);
            if (p_ref > 0.0)     H_ref -= p_ref * log(p_ref);

        }

        double NID = 2 - (H_gray+H_ref) / H_joint;
        cerr << "[NIDError]\t NID = " << NID << endl;


//        T T_NID(NID*NID);

        residuals[0] = NID * 1e-3;

        return true;
    }
};

struct NIDErrorCeresAxisAngleFull {
    vector<cv::Mat> images_;
    vector<PointCloud> lidars_;
    CameraModel::Ptr camera_;
    PinholeModel::Ptr pinhole_model_;

    NIDErrorCeresAxisAngleFull(CameraModel::Ptr camera, const vector<cv::Mat>& image, const vector<PointCloud>& lidar)
    {
        for(int i=0; i<image.size(); ++i) {
            cv::Mat img;
            image[i].copyTo(img);
            images_.push_back((img));

            PointCloud pc(lidar[i]);
//            pcl::copyPointCloud(lidar[i], pc);
            lidars_.push_back(pc);

            cerr << pc[0].x << ", " << pc[0].y << ", " << pc[0].z << endl << endl;

        }

        camera_ = camera;
        pinhole_model_ = static_pointer_cast<PinholeModel> (camera_);
    }

    static ceres::CostFunction* Create(CameraModel::Ptr camera, const vector<cv::Mat>& image, const vector<PointCloud>& lidar)
    {
        return (new ceres::AutoDiffCostFunction<NIDErrorCeresAxisAngleFull, 1, 6>(new NIDErrorCeresAxisAngleFull(camera, image, lidar)));
    }

    template <typename T>
    bool operator()(const T* const extrinsic, T* residuals) const {
        Histogram hist(256);
        Probability prob(256);

        double* p_extrinsic = new double[6];
        for(int i=0; i<6; ++i)
            p_extrinsic[i] = JetOps<T>::GetScalar(extrinsic[i]);

        Eigen::Isometry3d Tcl = util::axisAngleToIso(p_extrinsic);

        for(int i=0; i<images_.size(); ++i) {
            cv::Mat test;
            cvtColor(images_[i], test, cv::COLOR_GRAY2BGR);


            PointCloud lidar_c;
            pcl::transformPointCloud(lidars_[i], lidar_c, Tcl.matrix());

            for(auto point:lidar_c) {

                auto uv = camera_->xyz_to_uv(point);

                if(!camera_->is_in_image(uv, 2) || point.z <= 0.0 || point.z > 30.0)
                    continue;

                const float u_f = uv(0);
                const float v_f = uv(1);
                const int u_i = static_cast<int> (u_f);
                const int v_i = static_cast<int> (v_f);

                int gray_val = (int) images_[i].at<uint8_t>(v_i, u_i);
                int ref_val = (int) (point.intensity*255.0);

                hist.add_hist(gray_val, ref_val);

                cv::circle(test, cv::Point(u_i, v_i), 1.5, cv::Scalar( 1.0, 0.0, 0.0 ), -1);
            }

            cv::namedWindow("test",cv::WINDOW_AUTOSIZE);
            cv::imshow("test",test);
            cv::waitKey(1);
        }

        for (int i=0; i<256; ++i) {
            for (int j=0; j<256; ++j) {
                double joint_p = hist.joint_hist(i,j) / hist.num();

                double ref_p = hist.ref_hist(j) / hist.num();

                prob.joint_probability(i, j, joint_p);
                prob.ref_probability(j, ref_p);
//                cerr << joint_p << ", " << ref_p << endl;
            }
            double gray_p = hist.gray_hist(i) / hist.num();

            prob.gray_probability(i, gray_p);
        }

        double H_gray = 0.0;
        double H_ref = 0.0;
        double H_joint = 0.0;

        for (int i=0; i<256; ++i) {
            for (int j=0; j<256; ++j) {
                double p_joint = prob.joint_probability(i, j);

                if (p_joint > 0.0)  H_joint -= p_joint * log(p_joint);
            }

            double p_gray = prob.gray_probability(i);
            double p_ref = prob.ref_probability(i);

            if (p_gray > 0.0)    H_gray -= p_gray * log(p_gray);
            if (p_ref > 0.0)     H_ref -= p_ref * log(p_ref);

        }

        double NID = 2 - (H_gray+H_ref) / H_joint;
        cerr << "[NIDError]\t NID = " << NID << endl;


        T T_NID(NID);

        residuals[0] = T_NID;

        return true;
    }
};

struct NIDErrorCeresAxisAngle {
    cv::Mat image_;
    PointCloud lidar_;
    CameraModel::Ptr camera_;
    PinholeModel::Ptr pinhole_model_;

    NIDErrorCeresAxisAngle(CameraModel::Ptr camera, const cv::Mat& image, const PointCloud& lidar)
        : lidar_(lidar)
    {
        image.copyTo(image_);
        camera_ = camera;
        pinhole_model_ = static_pointer_cast<PinholeModel> (camera_);
    }

    static ceres::CostFunction* Create(CameraModel::Ptr camera, const cv::Mat& image, const PointCloud& lidar)
    {
        return (new ceres::AutoDiffCostFunction<NIDErrorCeresAxisAngle, 1, 6>(new NIDErrorCeresAxisAngle(camera, image, lidar)));
    }

    template <typename T>
    bool operator()(const T* const extrinsic, T* residuals) const {
        cv::Mat test;
        cvtColor(image_, test, cv::COLOR_GRAY2BGR);

        Histogram hist(256);
        Probability prob(256);

        double* p_extrinsic = new double[6];
        for(int i=0; i<6; ++i)
            p_extrinsic[i] = JetOps<T>::GetScalar(extrinsic[i]);

        Eigen::Isometry3d Tcl = util::axisAngleToIso(p_extrinsic);

        PointCloud lidar_c;
        pcl::transformPointCloud(lidar_, lidar_c, Tcl.matrix());

        for(auto point:lidar_c) {

            auto uv = camera_->xyz_to_uv(point);

            if(!camera_->is_in_image(uv, 2) || point.z <= 0.0 || point.z > 30.0)
                continue;

            const float u_f = uv(0);
            const float v_f = uv(1);
            const int u_i = static_cast<int> (u_f);
            const int v_i = static_cast<int> (v_f);

//            cerr << (int) image_.at<uint8_t>(v_i, u_i) << endl;
//            cerr << (int) (point.intensity*255.0) << endl;

            int gray_val = (int) image_.at<uint8_t>(v_i, u_i);
            int ref_val = (int) (point.intensity*255.0);

            hist.add_hist(gray_val, ref_val);

            cv::circle(test, cv::Point(u_i, v_i), 1.5, cv::Scalar( 1.0, 0.0, 0.0 ), -1);
        }

        for (int i=0; i<256; ++i) {
            for (int j=0; j<256; ++j) {
                double joint_p = hist.joint_hist(i,j) / hist.num();

                double ref_p = hist.ref_hist(j) / hist.num();

                prob.joint_probability(i, j, joint_p);
                prob.ref_probability(j, ref_p);
//                cerr << joint_p << ", " << ref_p << endl;
            }
            double gray_p = hist.gray_hist(i) / hist.num();

            prob.gray_probability(i, gray_p);
        }

        double H_gray = 0.0;
        double H_ref = 0.0;
        double H_joint = 0.0;

        for (int i=0; i<256; ++i) {
            for (int j=0; j<256; ++j) {
                double p_joint = prob.joint_probability(i, j);

                if (p_joint > 0.0)  H_joint -= p_joint * log(p_joint);
            }

            double p_gray = prob.gray_probability(i);
            double p_ref = prob.ref_probability(i);

            if (p_gray > 0.0)    H_gray -= p_gray * log(p_gray);
            if (p_ref > 0.0)     H_ref -= p_ref * log(p_ref);

        }

        double NID = 2 - (H_gray+H_ref) / H_joint;
//        double NID_squre = NID*NID;
//        double MI = H_gray + H_ref - H_joint;

//        cerr << "[NIDError]\t Img size: " << image_.cols << ", " << image_.rows << endl;
        cv::namedWindow("test",cv::WINDOW_AUTOSIZE);
        cv::imshow("test",test);
        cv::waitKey(1);

        cerr << "[NIDError]\t NID = " << NID << endl;

        T T_NID(NID);

        residuals[0] = T_NID;

        return true;
    }
};

struct NIDError {
    cv::Mat image_;
    PointCloud lidar_;
    CameraModel::Ptr camera_;
    PinholeModel::Ptr pinhole_model_;

    NIDError(CameraModel::Ptr camera, const cv::Mat& image, const PointCloud& lidar)
        : lidar_(lidar)
    {
        image.copyTo(image_);
        camera_ = camera;
        pinhole_model_ = static_pointer_cast<PinholeModel> (camera_);
    }

    static ceres::CostFunction* Create(CameraModel::Ptr camera, const cv::Mat& image, const PointCloud& lidar)
    {
        return (new ceres::AutoDiffCostFunction<NIDError, 1, Sophus::SE3d::num_parameters>(new NIDError(camera, image, lidar)));
    }

    template <typename T>
    bool operator()(const T* const extrinsic, T* residuals) const {

        cv::Mat test;
        cvtColor(image_, test, cv::COLOR_GRAY2BGR);
        // init hist
        Histogram hist(256);
        Probability prob(256);

        // lidar-to-camera projection
        double* p_extrinsic = new double[7];
        for(int i=0; i<7; ++i)
            p_extrinsic[i] = JetOps<T>::GetScalar(extrinsic[i]);

        Sophus::SE3Group<double> Tcl = Eigen::Map< const Sophus::SE3Group<double> >(p_extrinsic);


        PointCloud lidar_c;
        pcl::transformPointCloud(lidar_, lidar_c, Tcl.matrix());

        for(auto point:lidar_c) {

            auto uv = camera_->xyz_to_uv(point);

            if(!camera_->is_in_image(uv, 2) || point.z <= 0.0 || point.z > 30.0)
                continue;

            const float u_f = uv(0);
            const float v_f = uv(1);
            const int u_i = static_cast<int> (u_f);
            const int v_i = static_cast<int> (v_f);

//            cerr << (int) image_.at<uint8_t>(v_i, u_i) << endl;
//            cerr << (int) (point.intensity*255.0) << endl;

            int gray_val = (int) image_.at<uint8_t>(v_i, u_i);
            int ref_val = (int) (point.intensity*255.0);

            hist.add_hist(gray_val, ref_val);

            cv::circle(test, cv::Point(u_i, v_i), 1.5, cv::Scalar( 1.0, 0.0, 0.0 ), -1);
        }

        for (int i=0; i<256; ++i) {
            for (int j=0; j<256; ++j) {
                double joint_p = hist.joint_hist(i,j) / hist.num();

                double ref_p = hist.ref_hist(j) / hist.num();

                prob.joint_probability(i, j, joint_p);
                prob.ref_probability(j, ref_p);
//                cerr << joint_p << ", " << ref_p << endl;
            }
            double gray_p = hist.gray_hist(i) / hist.num();

            prob.gray_probability(i, gray_p);
        }

        double H_gray = 0.0;
        double H_ref = 0.0;
        double H_joint = 0.0;

        for (int i=0; i<256; ++i) {
            for (int j=0; j<256; ++j) {
                double p_joint = prob.joint_probability(i, j);

                if (p_joint > 0.0)  H_joint -= p_joint * log(p_joint);
            }

            double p_gray = prob.gray_probability(i);
            double p_ref = prob.ref_probability(i);

            if (p_gray > 0.0)    H_gray -= p_gray * log(p_gray);
            if (p_ref > 0.0)     H_ref -= p_ref * log(p_ref);

        }

        double NID = 2 - (H_gray+H_ref) / H_joint;
//        double NID_squre = NID*NID;
//        double MI = H_gray + H_ref - H_joint;

//        cerr << "[NIDError]\t Img size: " << image_.cols << ", " << image_.rows << endl;
        cv::namedWindow("test",cv::WINDOW_AUTOSIZE);
        cv::imshow("test",test);
        cv::waitKey(1);

        cerr << "[NIDError]\t NID = " << NID << endl;

        T T_NID(NID);

        residuals[0] = T_NID;

        return true;
    }
};  // struct NIDError

class Calibration
{
public:
    Calibration(string param_path);
    ~Calibration();
    void optimizer_sophusSE3();
    void optimizer_ceresAngleAxis();
    void optimizer_ceresAngleAxisFull();

private:
    FileLoader file_loader_;
    string param_path_;

    CameraModel::Ptr camera_;

    Matrix3x4 extrinsics_;
    Sophus::SE3d extrinsics_se3_;

    int num_data_;
    vector<cv::Mat> images_;
    vector<PointCloud> pc_;

    double* axis_;
};

} // camlidar_calib
