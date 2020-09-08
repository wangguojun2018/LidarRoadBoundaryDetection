#ifndef READ_POSE_H
#define READ_POSE_H

#include<string>
#include<vector>
#include<eigen3/Eigen/Dense>
#include<dirent.h>
#include<fstream>
#include<istream>
#include<iostream>

using namespace Eigen;
using namespace std;
class read_pose
{
public:
    read_pose(string txt_path,string pose_path);
    void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type);
   // bool computePairNum(std::string pair1,std::string pair2);
    void sort_filelists(std::vector<std::string>& filists,std::string type);
//    int getFileRows(const char *fileName);
//    int getFileColumns(const char * fileName);
    int getPoses(const char *fileName, vector<Matrix4d,aligned_allocator<Matrix4d>>& poses);
    Vector3d getAngles(const char *fileName);
    int get_calibration_imu_to_velodyne(const char * fileName, Matrix4d& Tr_velo_cam);
    void process(vector<Matrix<double,2,3>,aligned_allocator<Matrix<double,2,3>> >& transforms);
private:
    vector<Matrix4d,aligned_allocator<Matrix4d>> _poses;
    vector<Vector3d,aligned_allocator<Matrix4d>> _eulerangles;
    vector<string> _file_lists;
    string _txt_path;
    string _pose_path;
};

#endif // READ_POSE_H
