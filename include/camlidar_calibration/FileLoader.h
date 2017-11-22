#pragma once
#include <string>

using namespace std;

class FileLoader
{
public:
    FileLoader(string path);
    ~FileLoader();

private:
    string path_;

};
