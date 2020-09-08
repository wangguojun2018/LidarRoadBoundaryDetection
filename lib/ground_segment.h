/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-09-07 11:13:49
 */
#ifndef GROUND_SEGMENT_H
#define GROUND_SEGMENT_H


#include<pcl/PointIndices.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/filters/extract_indices.h>
#include<lib/types.h>
#include<proto/road_boundary.pb.h>


using namespace std;
using namespace RoadBoundary;

class GroundSegmentation
{
public:
    GroundSegmentation(PointCloudType::Ptr incloud,GroundSegmentationMsg msg);
    void extractGround(PointCloudType::Ptr outCloud,
            PointCloudType::Ptr inputCloud,
            pcl::PointIndices::Ptr Indices,
            bool setNeg=false);
    void planeSeg(PointCloudType::Ptr cloud,
            pcl::ModelCoefficients::Ptr coefficients,
            pcl::PointIndices::Ptr planeIndices);
    void groundfilter(PointCloudType::Ptr groundpoints,
                      PointCloudType::Ptr non_groundpoints);
    void process(PointCloudType::Ptr outcloud);

private:
    float _threshold;
    CloudPtrList _cloudptrlist;

};

#endif // GROUND_SEGMENT_H
