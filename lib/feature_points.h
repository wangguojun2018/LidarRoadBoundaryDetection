/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-09-07 11:13:04
 */

#ifndef CURB_POINT_H
#define CURB_POINT_H


#include<pcl/ModelCoefficients.h>
#include"math_utils.h"
#include<pcl/features/normal_3d.h>
#include<pcl/features/normal_3d_omp.h>
#include<pcl/filters/extract_indices.h>
#include<boost/make_shared.hpp>
#include<Eigen/Dense>
#include<Eigen/Core>
#include<algorithm>
#include<pcl/search/kdtree.h>
#include<iterator>
#include <pcl/features/don.h>
#include <pcl/filters/conditional_removal.h>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/project_inliers.h>
#include<lib/types.h>
#include <proto/road_boundary.pb.h>

using namespace std;
using namespace Eigen;
using namespace RoadBoundary;

class FeaturePoints
{
    public:
    // -sr 2 -hr 5 -st 1 -nrmin 0.25 -nt 0.25
    FeaturePoints(PointCloudType::Ptr incloud, vector<IndexRange> scanIndices,FeaturePointsMsg msg);
    float RegionMaxZ(int j);
    float RegionMinZ(int j);
    float slope_angle(int i);
    void compute_normal_omp(double searchRadius, pcl::PointCloud<pcl::Normal>& normal);
    vector<int> vectors_intersection(vector<int> v1, vector<int> v2);
    void extractPoints(PointCloudType::Ptr incloud,
                       PointCloudType::Ptr outCloud,
                       boost::shared_ptr<vector<int> > indices,bool setNeg=false);
    void extractFeatures(PointCloudType::Ptr feature_points);
    float calcPointDistance(const PointType& p);
    float computeHorizonDiff(int index,int region);
    float computeHeightSigma(int index,int region);
    void normal_diff_filter(PointCloudType::Ptr incloud, pcl::IndicesConstPtr &outIndex);

private:

    PointCloudType::Ptr _cloud;
    scanIndices _scanindices;

    int _HeightRegion;
    float _HeightSigmaThres;
    float _HeightMaxThres;
    float _HeightMinThres;

    float _CurvatureThres;
    int _CurvatureRegion;

    float _DistanceHorizonThres;
    float _DistanceVerticalThres;

    float _AngularRes;

    vector<int> _HeightPointsIndex;

    vector<int> _CurvaturePointsIndex;

    vector<int> _DistanceVerticlePointsIndex;

    vector<int> _DistanceHorizonPointsIndex;

    vector<int> _Index;

    bool _use_verticle;
    bool _use_horizon;
};

#endif // CURB_POINT_H
