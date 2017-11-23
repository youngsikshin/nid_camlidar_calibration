#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <dirent.h>

#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class FileLoader
{
public:
    FileLoader(string path);
    ~FileLoader();

    cv::Mat image(int idx);
    void lidar(int idx);
    int get_dir_list(string dir, vector<string> &files);

private:
    int idx_;

    string path_;
    string camera_path_;
    string lidar_path_;

    vector<string> camera_flist_;
    vector<string> lidar_flist_;
};
