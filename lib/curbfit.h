#ifndef CURBFIT_H
#define CURBFIT_H

#include<opencv2/opencv.hpp>
#include<vector>
#include<eigen3/Eigen/Core>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/ModelCoefficients.h>
// #include <pcl/surface/on_nurbs/fitting_curve_2d.h>
// #include <pcl/surface/on_nurbs/triangulation.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/PointIndices.h>
#include<boost/make_shared.hpp>

using namespace cv;
using namespace std;
using namespace Eigen;

typedef pcl::PointXYZI PointT;

class curbFit
{
public:
    curbFit(pcl::PointCloud<PointT>::Ptr incloud);
    void process(vector<pcl::PointXYZ>& line3d);
    void lineFitRansac(pcl::ModelCoefficients::Ptr model);
    // void PointCloud2Vector2d (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::on_nurbs::vector_vec2d& data);
    // void nurbFit(pcl::PointCloud<pcl::PointXYZRGB>::Ptr curveCloud);
    void ransac_curve(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud, double residualThreshold, vector<pcl::PointXYZ>& outPoints, bool isleft);
private:
    vector<Point> _inputData;
    vector<Point3f> _inputData3d;
    Vec4f _line;
    Vec6f _line3d;
    pcl::PointCloud<PointT> _lineCloud;
};

#endif // CURBFIT_H
