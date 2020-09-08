/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-09-07 11:13:28
 */
#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <lib/types.h>

using namespace std;
using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXY> PointCloudXY;

class point2D
{
    public:
    float r;
    float z;

    point2D() {}

    point2D(PointType point)
    {
        r = sqrt(pow(point.x, 2) + pow(point.y, 2));
        z = point.z;
    }
};


class GridMap
{
public:
    GridMap(PointCloudType::Ptr incloud, float gridRes, int gridNum);
    void generateCartesianGrid(vector<vector<PointType>> &grid_map_vec_carte);
   
    void distanceFilterByCartesianGrid(PointCloudType::Ptr outcloud, bool left);
private:
    PointCloudType::Ptr _origin_cloud_ptr;
    
    
    int _grid_num;
    float _grid_res;
};
#endif // GRID_MAP_H
