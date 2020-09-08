#ifndef READ_FRAMETRANSFORM_H
#define READ_FRAMETRANSFORM_H
#include<string>
#include<vector>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/StdVector>
#include<dirent.h>
#include<fstream>
#include<iostream>

//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)
#define Eigen_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ASSERT


using namespace std;
using namespace Eigen;

//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix4d)


class read_frameTransform
{
public:
    read_frameTransform(string filename);
    void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type);
   // bool computePairNum(std::string pair1,std::string pair2);
    void sort_filelists(std::vector<std::string>& filists,std::string type);
//    int getFileRows(const char *fileName);
//    int getFileColumns(const char * fileName);
    int getInputData(const char *fileName);
//    Vector3d getAngles(const char *fileName);
    void process(vector<Matrix4d,aligned_allocator<Matrix4d>>& transforms);
private:
    vector<Matrix4d,aligned_allocator<Matrix4d>> _frameTransforms;

//    vector<Vector3d> _eulerangles;
//    vector<string> _file_lists;
//    string _txt_path;
    string _TrOneFrame_path;
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // READ_FRAMETRANSFORM_H
