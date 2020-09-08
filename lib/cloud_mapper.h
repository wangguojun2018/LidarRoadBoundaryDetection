/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-09-07 11:12:35
 */
#ifndef CLOUD_MAPPER_H
#define CLOUD_MAPPER_H

#include<vector>
#include<cmath>
#include<lib/types.h>
#include<proto/road_boundary.pb.h>


using namespace std;
using namespace RoadBoundary;


class CloudMapper
{
    public:
        CloudMapper(CloudMapperMsg msg);
        CloudMapper(){ };

        const float& getLowerBound() { return _lowerBound; }
        const float& getUpperBound() { return _upperBound; }
        const int& getNumberOfScanRings() { return _nScanRings; }
        int getRingForAngle(const float& angle);
        void processByIntensity(PointCloudType::Ptr incloud, PointCloudType::Ptr outcloud,
                        scanIndices& scanindices);
        void processByOri(PointCloudType::Ptr incloud,PointCloudType::Ptr outcloud);
        void processByVer(PointCloudType::Ptr incloud,PointCloudType::Ptr outcloud,
                                scanIndices& scanindices);
        float _lowerBound;      ///< the vertical angle of the first scan ring
        float _upperBound;      ///< the vertical angle of the last scan ring
        int _nScanRings;   ///< number of scan rings
        float _factor;          ///< linear interpolation factor

};

#endif // CLOUD_MAPPER_H
