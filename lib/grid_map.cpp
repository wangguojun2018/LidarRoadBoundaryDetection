/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-09-04 16:28:24
 */
#include "grid_map.h"

GridMap::GridMap(PointCloudType::Ptr incloud, float gridRes, int gridNum)
    :_grid_res(gridRes),_grid_num(gridNum)
{
    _origin_cloud_ptr.reset(new PointCloudType);
    *_origin_cloud_ptr=*incloud;
}


void GridMap::generateCartesianGrid( vector<vector<pcl::PointXYZI> >& grid_map_vec_carte)
{
    grid_map_vec_carte.resize(_grid_num);
    for(int i=0;i<_origin_cloud_ptr->points.size();++i)
    {

        int row=int(_origin_cloud_ptr->points[i].x/_grid_res+_grid_num/2);
        if(row>=0&&row<grid_map_vec_carte.size())
        {
            grid_map_vec_carte[row].push_back(_origin_cloud_ptr->points[i]);
        }
    }
}

void GridMap::distanceFilterByCartesianGrid(PointCloudType::Ptr outcloud,bool left)
{
    vector<vector<pcl::PointXYZI> > gridLeft;
    this->generateCartesianGrid(gridLeft);

    PointCloudType::Ptr Leftcloud(new PointCloudType);

    for(int i=0;i<gridLeft.size();++i)
    {
        if (gridLeft[i].size() > 0)
        {
            int min_y_Left = 0;

            for (size_t k = 0; k < gridLeft[i].size(); ++k)
            {
                if(left)
                {
                    if(gridLeft[i][min_y_Left].y> gridLeft[i][k].y)
                    {
                        min_y_Left = k;
                    }
                }
                else
                {
                    if(gridLeft[i][min_y_Left].y< gridLeft[i][k].y)
                    {
                        min_y_Left = k;
                    }
                }

            }
            outcloud->points.push_back(gridLeft[i][min_y_Left]);
        }
    }
}
