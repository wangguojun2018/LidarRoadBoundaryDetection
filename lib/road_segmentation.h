/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-09-07 11:14:03
 */
#ifndef ROAD_SEGMENTATION_H
#define ROAD_SEGMENTATION_H

#include<lib/types.h>

using namespace std;

class RoadSegmentation
{
public:
    RoadSegmentation(PointCloudType::Ptr incloud);
    void generatePolarGrid();
    void computeDistanceVec();
    void computeSegmentAngle();
    void process(PointCloudType::Ptr incloud,CloudPtrList outcloud);
    
    vector<PointType> _nearest_points;
    vector<float> _distance_vec_filtered;
    vector<float> _distance_vec_;

private:
    PointCloudType::Ptr _completeCloud;

    vector< vector<PointType> > _grid_map_vec;

    
    vector<pair<float,int> > _distance_vec;
    vector<pair<float,int> > _distance_vec_front;
    vector<pair<float,int> > _distance_vec_rear;

    vector<int> _segmentAngle;

};

#endif // ROAD_SEGMENTATION_H
