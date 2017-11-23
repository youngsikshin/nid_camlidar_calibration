#include <camlidar_calibration/FileLoader.h>

FileLoader::FileLoader(string path)
    : idx_(0), path_(path)
{
    LOG(INFO) << "[FileLoader]\t Set path : " << path_ << endl;

    camera_path_ = path_ + "camera/";
    lidar_path_ = path_ + "lidar/";
    get_dir_list(camera_path_, camera_flist_);
    get_dir_list(lidar_path_, lidar_flist_);

    for(auto fname : camera_flist_)
        cerr << fname << endl;

    for(auto fname : lidar_flist_)
        cerr << fname << endl;
}

FileLoader::~FileLoader()
{

}

cv::Mat FileLoader::image(int idx)
{
    string camera_fname = camera_path_ + "camera/" + camera_flist_[idx];
    LOG(INFO) << "[FileLoader]\t Camera file name : " << camera_fname << endl;

    return cv::imread(camera_fname,CV_LOAD_IMAGE_GRAYSCALE);
}

void FileLoader::lidar(int idx)
{
    string lidar_fname = lidar_path_ + lidar_flist_[idx];
    LOG(INFO) << "[FileLoader]\t Lidar file name : " << lidar_fname << endl;

    ifstream f_lidar;
    f_lidar.open(lidar_fname, ios::in | ios::binary);

//    f_lidar.seekg(0, ios::end);
//    int f_size = f_lidar.tellg();

    while(!f_lidar.eof()){
        float x, y, z, r;

        f_lidar.read(reinterpret_cast <char *>(&x), sizeof(x));
        f_lidar.read(reinterpret_cast <char *>(&y), sizeof(y));
        f_lidar.read(reinterpret_cast <char *>(&z), sizeof(z));
        f_lidar.read(reinterpret_cast <char *>(&r), sizeof(r));

        LOG(INFO) << "[FileLoader]\t x y z r : " << x << ", " << y << ", " << z << ", " << r << endl;

    }

    f_lidar.close();
//    LOG(INFO) << "[FileLoader]\t Lidar file size : " << f_size << endl;
}

int FileLoader::get_dir_list(string dir, vector<string> &files)
{
    vector<string> tmp_files;
    struct dirent **namelist;
    int n;
    n = scandir(dir.c_str(),&namelist, 0 , alphasort);
    if (n < 0)
        LOG(ERROR) << "scandir" << endl;
    else {
        while (n--) {
            if(string(namelist[n]->d_name) != "." && string(namelist[n]->d_name) != ".."){
                tmp_files.push_back(string(namelist[n]->d_name));
            }
            free(namelist[n]);
        }

        free(namelist);
    }

    for(auto iter = tmp_files.rbegin() ; iter!= tmp_files.rend() ; iter++){
        files.push_back(*iter);
    }

    return n;
}
