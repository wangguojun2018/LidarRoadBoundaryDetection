#include "rectify.h"


rectify::rectify()
{

}


void rectify::process(PointCloud incloud, tran_rota &tran, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudScans)
{
    size_t cloudSize = incloud->size();



    //计算点云扫描水平起始角度和终止角度

    float startOri = std::atan2(incloud->points[0].y,incloud->points[0].x);  //返回-pi到pi
    float endOri =std::atan2(incloud->points[cloudSize-1].y,incloud->points[cloudSize-1].x)+2*M_PI;

//    float endOri=startOri+2*M_PI;
    //    float endOri=2*M_PI;
    //    float startOri=0;
    //            incloud->points[cloudSize - 1].x);//lidar从x轴开始逆时针扫描的
    cout<<" incloud last point is "<<incloud->points[cloudSize-1].x<<incloud->points[cloudSize - 1].y<<endl;
    cout<<" incloud first point is "<<incloud->points[0].x<<" "<<incloud->points[0].y<<endl;
    cout<<"startOri is "<<startOri*180/M_PI<<endl;
    cout<<"endOri is "<<endOri*180/M_PI<<endl;
    //    if (endOri - startOri > 3 * M_PI)
    //    {
    //        endOri -= 2 * M_PI;
    //    }
    //    else if (endOri - startOri < M_PI)
    //    {
    //        endOri += 2 * M_PI;
    //    }

    //    if (startOri - endOri <0)
    //    {
    //        endOri -= 2 * M_PI;
    //    }

    //    else if (endOri - startOri < M_PI)
    //    {
    //        endOri += 2 * M_PI;
    //    }
    //std::cout<<"startori is "<<startOri<<" endori is "<<endOri<<std::endl;
    bool halfPassed =false;
    pcl::PointXYZI point;
    //vector<pcl::PointCloud<pcl::PointXYZI> > laserCloudScans(_mapper.getNumberOfScanRings());

    for (int i = 0; i < cloudSize; i++)
    {
        point.x = incloud->points[i].x;
        point.y = incloud->points[i].y;
        point.z = incloud->points[i].z;

        // skip NaN and INF valued points
        if (!pcl_isfinite(point.x) ||
                !pcl_isfinite(point.y) ||
                !pcl_isfinite(point.z))
        {
            continue;
        }

        // skip zero valued points
        if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001)
        {
            continue;
        }

        // calculate horizontal point angle，计算点的水平角

        float ori = std::atan2(point.y, point.x);
        //        if (!halfPassed)
        //        {
        //            if (ori < startOri - M_PI / 2)
        //            {
        //                ori += 2 * M_PI;
        //            }
        //            else if (ori > startOri + M_PI * 3 / 2)
        //            {
        //                ori -= 2 * M_PI;
        //            }

        //            if (ori - startOri > M_PI)
        //            {
        //                halfPassed = true;
        //            }
        //        }
        if(ori<startOri)
        {
            ori+=2*M_PI;
        }
        //        else
        //        {
        //            ori += 2 * M_PI;

        //            if (ori < endOri - M_PI * 3 / 2)
        //            {
        //                ori += 2 * M_PI;
        //            }
        //            else if (ori > endOri + M_PI / 2)
        //            {
        //                ori -= 2 * M_PI;
        //            }
        //        }

        // calculate relative scan time based on point orientation，根据点的水平角计算点在当前帧的时间戳
        float relTime = 0.1*(ori - startOri) / (endOri - startOri);
        point.intensity =relTime;
        // project point to the start of the sweep using corresponding IMU data，将该点映射到扫描起始时间
        
        Transform_point(point,tran,relTime);

        laserCloudScans->points.push_back(point);
    }

    // construct sorted full resolution cloud
    //    cloudSize = 0;
    //    for (int i = 0; i < _mapper.getNumberOfScanRings(); i++)
    //    {
    //        (*outcloud) += laserCloudScans[i];
    //        IndexRange range(cloudSize, 0);
    //        cloudSize += laserCloudScans[i].size();
    //        range.second = cloudSize > 0 ? cloudSize - 1 : 0;
    //        scanindices.push_back(range);
    //    }
}

void rectify::Transform_point(pcl::PointXYZI& point, tran_rota &trans, float realtime)
{
    //std::cout<<"point i is "<<point.x<<" "<<point.y<<" "<<point.z<<std::endl;
    Vector3d _point(point.x, point.y,point.z);
    Vector3d Tran=(realtime/0.1)*trans.row(0);
    Vector3d Rota=(realtime/0.1)*trans.row(1);
    //    Matrix3d Rmatrix=euler2RotationMatrix(Rota);
    Matrix3d Rx=Matrix3d::Identity();
    Matrix3d Ry=Matrix3d::Identity();
    Matrix3d Rz;
//    Rx<<1,0,0,
//            0,cos(Rota(0)),-sin(Rota(0)),
//            0,sin(Rota(0)),cos(Rota(0));
//    Ry<<cos(Rota(1)),0,sin(Rota(1)),
//            0,1,0,
//            -sin(Rota(1)),0,cos(Rota(1));
    Rz<<cos(Rota(2)),-sin(Rota(2)),0,
            sin(Rota(2)),cos(Rota(2)),0,
            0,0,1;
    Matrix3d MatrixR=(Rz*Ry*Rx);
    _point=MatrixR.transpose()*(_point-Tran);
    point.x=_point(0);
    point.y=_point(1);
    point.z=_point(2);
    // std::cout<<"after rectify point i is "<float endOri=2*M_PI;<point.x<<" "<<point.y<<" "<<point.z<<std::endl;
}

Matrix3d rectify::euler2RotationMatrix(const Vector3d &angleRota)
{
    Eigen::AngleAxisd rollAngle(angleRota(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(angleRota(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(angleRota(2), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = rollAngle*yawAngle*pitchAngle;
    Eigen::Matrix3d R = q.matrix();
    return R;
}


