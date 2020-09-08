#include "cloud_mapper.h"

CloudMapper::CloudMapper(CloudMapperMsg msg)
   
{
    _lowerBound=msg.lowerbound();
    _upperBound=msg.upperbound();
    _nScanRings=msg.nscanrings();
    _factor=(_nScanRings - 1) / (_upperBound - _lowerBound);

}

int CloudMapper::getRingForAngle(const float& angle)
{
    return int(((angle * 180 / M_PI) - _lowerBound) * _factor+0.5);
}

void CloudMapper::processByVer(PointCloudType::Ptr incloud,PointCloudType::Ptr outcloud,
                                scanIndices& scanindices)
{
    size_t cloudSize = incloud->points.size();

    vector<PointCloudType > laserCloudScans(_nScanRings);

    // extract valid points from input cloud
    for (int i = 1; i < cloudSize; i++)
    {
        // skip NaN and INF valued points
        if (!pcl_isfinite(incloud->points[i].x) ||
                !pcl_isfinite(incloud->points[i].y) ||
                !pcl_isfinite(incloud->points[i].z))
        {
            continue;
        }

        // skip zero valued points
        if (incloud->points[i].x * incloud->points[i].x +incloud->points[i].y *incloud->points[i].y +
                incloud->points[i].z * incloud->points[i].z < 0.0001)
        {
            continue;
        }

        // calculate vertical point angle and scan ID
        float angle = std::atan(incloud->points[i].z / std::sqrt(incloud->points[i].x *incloud->points[i].x +
                                                                 incloud->points[i].y * incloud->points[i].y));

        int scanID = getRingForAngle(angle);
        if (scanID >= getNumberOfScanRings() || scanID < 0)
        {
            continue;
        }
        laserCloudScans[scanID].push_back(incloud->points[i]);
    }
    
    cloudSize = 0;
    for (int i = 0; i <getNumberOfScanRings(); i++)
    {
        if(laserCloudScans[i].size()>0)
        {
            (*outcloud) += laserCloudScans[i];
            IndexRange range(cloudSize, 0);
            cloudSize += laserCloudScans[i].size();
            range.second = cloudSize > 0 ? cloudSize - 1 : 0;
            scanindices.push_back(range);
        }

    }

}
void CloudMapper::processByIntensity(PointCloudType::Ptr incloud,PointCloudType::Ptr outcloud,
                                scanIndices& scanindices)
{
    size_t cloudSize = incloud->points.size();

    vector<PointCloudType > laserCloudScans(_nScanRings);
    int scanID=0;
    //    float intensity=(rand()%(256+1));
    // extract valid points from input cloud
    for (int i = 0; i < cloudSize; i++)
    {
        scanID=incloud->points[i].intensity;

        laserCloudScans[scanID].push_back(incloud->points[i]);
    }
    //laserCloudOut=laserCloudScans;
    cloudSize = 0;
    for (int i = 0; i <getNumberOfScanRings(); i++)
    {
        if(laserCloudScans[i].size()>0)
        {
            (*outcloud) += laserCloudScans[i];
            IndexRange range(cloudSize, 0);
            cloudSize += laserCloudScans[i].size();
            range.second = cloudSize > 0 ? cloudSize - 1 : 0;
            scanindices.push_back(range);
        }

    }

}
void CloudMapper::processByOri(PointCloudType::Ptr incloud,PointCloudType::Ptr outcloud)
{
    size_t cloudSize = incloud->points.size();
    int scanID=0;
    PointType point;
    for (int i = 1; i < cloudSize; i++)
    {
        // skip NaN and INF valued points
        if (!pcl_isfinite(incloud->points[i].x) ||
                !pcl_isfinite(incloud->points[i].y) ||
                !pcl_isfinite(incloud->points[i].z))
        {
            continue;
        }

        // skip zero valued points
        if (incloud->points[i].x * incloud->points[i].x +incloud->points[i].y *incloud->points[i].y +
                incloud->points[i].z * incloud->points[i].z < 0.0001)
        {
            continue;
        }

        float ori = std::atan2(incloud->points[i].y, incloud->points[i].x)*180/M_PI;
        float ori_pre = std::atan2(incloud->points[i-1].y, incloud->points[i-1].x)*180/M_PI;
        
        if(ori<0)
        {
            ori+=360;
        }
        if(ori_pre<0)
        {
            ori_pre+=360;
        }

        if(abs(ori-ori_pre)>250)
        {
            scanID+=1;
        }

        if(scanID<getNumberOfScanRings())
        {
            point=incloud->points[i];
            point.intensity=scanID;
            outcloud->points.push_back(point);
        }
    }


}


