#ifndef RECTIFY_H
#define RECTIFY_H


#include<eigen3/Eigen/Dense>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <Eigen/Geometry>
#include"cloud_mapper.h"


typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud;

using namespace Eigen;
using namespace std;
typedef const Matrix<double,2,3> tran_rota;




class rectify
{
public:
    rectify();

    void process(PointCloud incloud, tran_rota& tran, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudScans);
    void Transform_point(pcl::PointXYZI& point,tran_rota& tran,float realtime);
    Matrix3d euler2RotationMatrix(const Vector3d& angleRota);

private:
    //Matrix4d _trans;
    //PointCloud _incloud;
    //cloud_mapper _mapper;
};

#endif // RECTIFY_H
