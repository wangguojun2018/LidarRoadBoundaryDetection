/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-09-07 20:42:23
 */

#include "road_segmentation.h"
#include<lib/log.h>


RoadSegmentation::RoadSegmentation(PointCloudType::Ptr incloud)

{
    _completeCloud.reset(new PointCloudType);
    *_completeCloud = *incloud;
    _grid_map_vec.resize(360);
    _segmentAngle.resize(2);
    _distance_vec.resize(360);
    _distance_vec_filtered.resize(360);
    _distance_vec_.resize(360);
    _nearest_points.resize(360);
}

void RoadSegmentation::generatePolarGrid()
{
    for (int i = 0; i < _completeCloud->points.size(); ++i)
    {

        float ori = std::atan2(_completeCloud->points[i].y,
                               _completeCloud->points[i].x) *
                    180 / M_PI;
        ori = ori < 0 ? ori + 360 : ori;
        float resolution = 1;
        int segment_index = (int)(ori / resolution);

        if (segment_index < 360)
        {
            _grid_map_vec[segment_index].push_back(_completeCloud->points[i]);
        }
    }
}

void RoadSegmentation::computeDistanceVec()
{
#pragma omp parallel for schedule(runtime)
    for (int i = 0; i < _grid_map_vec.size(); ++i)
    {
        sort(_grid_map_vec[i].begin(), _grid_map_vec[i].end(), [](PointType &left, PointType &right) {
            float d1 = pow(left.x, 2) + pow(left.y, 2);
            float d2 = pow(right.x, 2) + pow(right.y, 2);
            return (d1 < d2);
        });

        if (_grid_map_vec[i].size() > 0)
        {
            double distance_min = pow(pow(_grid_map_vec[i][0].x, 2) + pow(_grid_map_vec[i][0].y, 2), 0.5);
            double distance_max = pow(pow(_grid_map_vec[i][_grid_map_vec[i].size() - 1].x, 2) +
                                          pow(_grid_map_vec[i][_grid_map_vec[i].size() - 1].y, 2),
                                      0.5);
            _distance_vec[i] = pair<float, int>(distance_min / distance_max, i);
            // _distance_vec_[i]=distance_min/distance_max;
            // PointType point;
            // point.x = distance_min * cos(i * M_PI / 180);
            // point.y = distance_min * sin(i * M_PI / 180);
            // point.z = 0;
            // _nearest_points[i] = point;
        }
        else
        {
            // PointType point;
            // point.x = 33 * cos(i * M_PI / 180);
            // point.y = 33 * sin(i * M_PI / 180);
            // point.z = 0;
            _distance_vec[i] = pair<float, int>(1.0, i);
            // _nearest_points[i] = point;
        }
    }
    //    vector<pair<double,int> > distance_vec_temp(360);
    //    distance_vec_temp[0]=_distance_vec[0];
    //    distance_vec_temp[1]=_distance_vec[1];
    //    distance_vec_temp[359]=_distance_vec[359];
    //    distance_vec_temp[358]=_distance_vec[358];
    //对距离vector进行中值滤波
    _distance_vec_front.push_back(_distance_vec[0]);
    _distance_vec_front.push_back(_distance_vec[1]);
    _distance_vec_front.push_back(_distance_vec[2]);
    _distance_vec_front.push_back(_distance_vec[359]);
    _distance_vec_front.push_back(_distance_vec[358]);
    _distance_vec_front.push_back(_distance_vec[357]);
    for (int i = 3; i < _distance_vec.size() - 3; ++i)
    {
        vector<float> temp;
        if (i >= 0 && i < 3)
        {
            //            vector<double> temp;
            temp.push_back(_distance_vec[i - 3 + 360].first);
            temp.push_back(_distance_vec[i - 2 + 360].first);
            temp.push_back(_distance_vec[i - 1 + 360].first);
            temp.push_back(_distance_vec[i].first);
            temp.push_back(_distance_vec[i + 1].first);
            temp.push_back(_distance_vec[i + 2].first);
            temp.push_back(_distance_vec[i + 3].first);
            sort(temp.begin(), temp.end());
        }
        else if (i >= 357 && i <= 359)
        {
            //            vector<double> temp;
            temp.push_back(_distance_vec[i - 3].first);
            temp.push_back(_distance_vec[i - 2].first);
            temp.push_back(_distance_vec[i - 1].first);
            temp.push_back(_distance_vec[i].first);
            temp.push_back(_distance_vec[i + 1 - 360].first);
            temp.push_back(_distance_vec[i + 2 - 360].first);
            temp.push_back(_distance_vec[i + 3 - 360].first);
            sort(temp.begin(), temp.end());
        }
        else
        {
            //            vector<double> temp;
            temp.push_back(_distance_vec[i - 3].first);
            temp.push_back(_distance_vec[i - 2].first);
            temp.push_back(_distance_vec[i - 1].first);
            temp.push_back(_distance_vec[i].first);
            temp.push_back(_distance_vec[i + 1].first);
            temp.push_back(_distance_vec[i + 2].first);
            temp.push_back(_distance_vec[i + 3].first);
            sort(temp.begin(), temp.end());
        }

        if (i > 90 && i < 270)
        {
            _distance_vec_rear.push_back(pair<float, int>(temp[3], i));
        }
        else
        {
            _distance_vec_front.push_back(pair<float, int>(temp[3], i));
        }
        // _distance_vec_filtered[i] = temp[2];
    }
}

void RoadSegmentation::computeSegmentAngle()
{
    sort(_distance_vec_front.begin(), _distance_vec_front.end(), [](pair<float, int> &left, pair<float, int> &right) {
        return left.first > right.first;
    });

    sort(_distance_vec_rear.begin(), _distance_vec_rear.end(), [](pair<float, int> &left, pair<float, int> &right) {
        return left.first > right.first;
    });

    //计算前方segmentation angle
    if (_distance_vec_front[0].first == 1.0)
    {
        vector<int> max_distance_angle_left;
        vector<int> max_distance_angle_right;

        for (int i = 0; i < _distance_vec_front.size(); ++i)
        {
            if (_distance_vec_front[i].first == 1.0 && _distance_vec_front[i].second < 90)
            {
                max_distance_angle_left.push_back(_distance_vec_front[i].second);
            }
            if (_distance_vec_front[i].first == 1.0 && _distance_vec_front[i].second > 270)
            {
                max_distance_angle_right.push_back(_distance_vec_front[i].second);
            }
        }

        sort(max_distance_angle_left.begin(), max_distance_angle_left.end(), [](int &left, int &right) {
            return left > right;
        });

        sort(max_distance_angle_right.begin(), max_distance_angle_right.end(), [](int &left, int &right) {
            return left > right;
        });

        max_distance_angle_left.insert(max_distance_angle_left.end(),
                                       max_distance_angle_right.begin(),
                                       max_distance_angle_right.end());

        _segmentAngle[0] = max_distance_angle_left[int(max_distance_angle_left.size() / 2)];
    }
    else
    {
        _segmentAngle[0] = _distance_vec_front[0].second;
    }
    //计算后方segmentation angle
    if (_distance_vec_rear[0].first == 1.0)
    {
        vector<int> max_distance_angle;
        for (int i = 0; i < _distance_vec_rear.size(); ++i)
        {
            if (_distance_vec_rear[i].first == 1)
            {
                max_distance_angle.push_back(_distance_vec_rear[i].second);
            }
        }
        sort(max_distance_angle.begin(), max_distance_angle.end());
        _segmentAngle[1] = max_distance_angle[int(max_distance_angle.size() / 2)];
    }
    else
    {
        _segmentAngle[1] = _distance_vec_rear[0].second;
    }
    AINFO << "The front segmentation angle is " << _segmentAngle[0] << endl;
    AINFO << "The back segmentation angle is  " << _segmentAngle[1] << endl;
}

void RoadSegmentation::process(PointCloudType::Ptr incloud, CloudPtrList outcloud)
{
    this->generatePolarGrid();
    this->computeDistanceVec();
    this->computeSegmentAngle();

    for (int i = 0; i < incloud->points.size(); ++i)
    {
        PointType point(incloud->points[i]);
        //        point.intensity=i;
        if (point.x > 0)
        {

            if (point.y > point.x * tan(_segmentAngle[0] * M_PI / 180))
            {

                outcloud[0]->points.push_back(point);
            }
            else
            {
                outcloud[1]->points.push_back(point);
            }
        }
        else
        {
            if (point.y > point.x * tan(_segmentAngle[1] * M_PI / 180))
            {
                outcloud[0]->points.push_back(point);
            }
            else
            {
                outcloud[1]->points.push_back(point);
            }
        }
    }
}
