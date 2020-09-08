#include "boundary_points.h"
#include<lib/log.h>
BoundaryPoints::BoundaryPoints(PointCloudType &incloud, BoundaryPointsMsg msg)
{
    _cloud.reset(new PointCloudType);
    *_cloud = incloud;
    _gridNum = msg.gridnum();
    _gridRes = msg.gridres();
    _varThres = msg.varthres();
    _meanThres = msg.meanthres();
    _curveFitThres = msg.curvefitthres();
    _use_curve_fit=msg.usecurveransac();
}
void BoundaryPoints::distanceFilterByLaserLeft(PointCloudType::Ptr incloud, PointCloudType::Ptr outcloud)
{
    PointCloudType::Ptr completeCloudLeftMapped(new PointCloudType);
    scanIndices scanIDindicesLeft;
    CloudMapper mapperLeft;
    mapperLeft.processByIntensity(incloud, completeCloudLeftMapped, scanIDindicesLeft);
    PointCloudType::Ptr Leftcloud(new PointCloudType);
    // 从每个scan中提取curb点
    size_t nScansLeft = scanIDindicesLeft.size();
    //    *clusterCloud[0]=_completeCloudFiltered;
    for (size_t i = 0; i < nScansLeft; i++)
    {
        size_t scanStartIdx = scanIDindicesLeft[i].first;
        size_t scanEndIdx = scanIDindicesLeft[i].second;

        vector<pcl::PointXYZI> LeftFront;
        vector<pcl::PointXYZI> LeftRear;

        for (size_t j = scanStartIdx; j <= scanEndIdx; ++j)
        {
            if (completeCloudLeftMapped->points[j].x >= 0)
            {
                LeftFront.push_back(completeCloudLeftMapped->points[j]);
            }
            else
            {
                LeftRear.push_back(completeCloudLeftMapped->points[j]);
            }
        }
        //针对每个scan每个象限分别提取y值绝对值最小的点
        //左前方
        if (LeftFront.size() > 0)
        {
            vector<pcl::PointXYZI> yNeg;
            //            vector<pcl::PointXYZI> yPos;
            for (size_t k = 0; k < LeftFront.size(); ++k)
            {

                if (LeftFront[k].y < 0)
                {
                    yNeg.push_back(LeftFront[k]);
                }
            }

            if (yNeg.size() > 0)
            {
                int max_y_LeftFront = 0;
                for (size_t k = 0; k < yNeg.size(); ++k)
                {

                    if (abs(yNeg[max_y_LeftFront].y) < abs(yNeg[k].y))
                    {
                        max_y_LeftFront = k;
                    }
                }
                Leftcloud->points.push_back(yNeg[max_y_LeftFront]);
            }
            else
            {

                int min_y_LeftFront = 0;
                for (size_t k = 0; k < LeftFront.size(); ++k)
                {

                    if (abs(LeftFront[min_y_LeftFront].y) > abs(LeftFront[k].y))
                    {
                        min_y_LeftFront = k;
                    }
                }
                Leftcloud->points.push_back(LeftFront[min_y_LeftFront]);
            }
        }
        //左后方
        if (LeftRear.size() > 0)
        {
            vector<pcl::PointXYZI> yNeg;
            for (size_t k = 0; k < LeftRear.size(); ++k)
            {

                if (LeftRear[k].y < 0)
                {
                    yNeg.push_back(LeftRear[k]);
                }
            }
            if (yNeg.size() > 0)
            {
                int min_y_LeftRear = 0;
                for (size_t k = 0; k < yNeg.size(); ++k)
                {

                    if (abs(yNeg[min_y_LeftRear].y) < abs(yNeg[k].y))
                    {
                        min_y_LeftRear = k;
                    }
                }
                Leftcloud->points.push_back(yNeg[min_y_LeftRear]);
            }
            else
            {
                int min_y_LeftRear = 0;
                for (size_t k = 0; k < LeftRear.size(); ++k)
                {

                    if (abs(LeftRear[min_y_LeftRear].y) > abs(LeftRear[k].y))
                    {
                        min_y_LeftRear = k;
                    }
                }
                Leftcloud->points.push_back(LeftRear[min_y_LeftRear]);
            }
        }
    }
    *outcloud = *Leftcloud;
}
void BoundaryPoints::distanceFilterByLaserRight(PointCloudType::Ptr incloud, PointCloudType::Ptr outcloud)
{
    PointCloudType::Ptr completeCloudLeftMapped(new PointCloudType);
    scanIndices scanIDindicesLeft;
    CloudMapper mapperLeft;
    mapperLeft.processByIntensity(incloud, completeCloudLeftMapped, scanIDindicesLeft);
    PointCloudType::Ptr Leftcloud(new PointCloudType);
    // 从每个scan中提取curb点
    size_t nScansLeft = scanIDindicesLeft.size();
    //    *clusterCloud[0]=_completeCloudFiltered;
    for (size_t i = 0; i < nScansLeft; i++)
    {
        size_t scanStartIdx = scanIDindicesLeft[i].first;
        size_t scanEndIdx = scanIDindicesLeft[i].second;

        vector<pcl::PointXYZI> LeftFront;
        vector<pcl::PointXYZI> LeftRear;

        for (size_t j = scanStartIdx; j <= scanEndIdx; ++j)
        {
            if (completeCloudLeftMapped->points[j].x >= 0)
            {
                LeftFront.push_back(completeCloudLeftMapped->points[j]);
            }
            else
            {
                LeftRear.push_back(completeCloudLeftMapped->points[j]);
            }
        }
        //针对每个scan每个象限分别提取y值绝对值最小的点
        //左前方
        if (LeftFront.size() > 0)
        {
            vector<PointType> yPos;
            for (size_t k = 0; k < LeftFront.size(); ++k)
            {

                if (LeftFront[k].y > 0)
                {
                    yPos.push_back(LeftFront[k]);
                }
            }

            if (yPos.size() > 0)
            {
                int max_y_LeftFront = 0;
                for (size_t k = 0; k < yPos.size(); ++k)
                {

                    if (abs(yPos[max_y_LeftFront].y) < abs(yPos[k].y))
                    {
                        max_y_LeftFront = k;
                    }
                }
                Leftcloud->points.push_back(yPos[max_y_LeftFront]);
            }
            else
            {

                int min_y_LeftFront = 0;
                for (size_t k = 0; k < LeftFront.size(); ++k)
                {

                    if (abs(LeftFront[min_y_LeftFront].y) > abs(LeftFront[k].y))
                    {
                        min_y_LeftFront = k;
                    }
                }
                Leftcloud->points.push_back(LeftFront[min_y_LeftFront]);
            }
        }
        //左后方
        if (LeftRear.size() > 0)
        {
            vector<PointType> yPos;
            for (size_t k = 0; k < LeftRear.size(); ++k)
            {

                if (LeftRear[k].y > 0)
                {
                    yPos.push_back(LeftRear[k]);
                }
            }
            if (yPos.size() > 0)
            {
                int min_y_LeftRear = 0;
                for (size_t k = 0; k < yPos.size(); ++k)
                {

                    if (abs(yPos[min_y_LeftRear].y) < abs(yPos[k].y))
                    {
                        min_y_LeftRear = k;
                    }
                }
                Leftcloud->points.push_back(yPos[min_y_LeftRear]);
            }
            else
            {
                int min_y_LeftRear = 0;
                for (size_t k = 0; k < LeftRear.size(); ++k)
                {

                    if (abs(LeftRear[min_y_LeftRear].y) > abs(LeftRear[k].y))
                    {
                        min_y_LeftRear = k;
                    }
                }
                Leftcloud->points.push_back(LeftRear[min_y_LeftRear]);
            }
        }
    }
    *outcloud = *Leftcloud;
}

void BoundaryPoints::extractPointCloud(PointCloudType &incloud, pcl::PointIndicesPtr indices, PointCloudType &outcloud)
{

    //    boost::shared_ptr<vector<int> > indice_=boost::make_shared<vector<int> >(indices);
    PointCloudType::Ptr cloudOut(new PointCloudType);
    //    pcl::PointCloud<PointT>::Ptr cloudIn(new pcl::PointCloud<PointT>);
    //    *cloudIn=_completeCloud;
    pcl::ExtractIndices<PointType> extract;
    extract.setNegative(false);
    extract.setInputCloud(boost::make_shared<PointCloudType>(incloud));
    extract.setIndices(indices);
    extract.filter(*cloudOut);
    outcloud = *cloudOut;
}

void BoundaryPoints::lineFitRansac(PointCloudType &incloud, pcl::PointIndices &indices)
{
    pcl::ModelCoefficients _model;
    pcl::PointIndices _indices;
    Vector3f axis(1, 0, 0);
    //    pcl::ModelCoefficients model;
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    // seg.setAxis(axis);
    // seg.setEpsAngle(0.35);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(_curveFitThres);
    seg.setInputCloud(boost::make_shared<PointCloudType>(incloud));
    seg.segment(_indices, _model);
    indices = _indices;
}

void BoundaryPoints::statisticalFilter_indces(PointCloudType incloud, int meanK, pcl::PointIndicesPtr pointindices, double stdThreshold)
{
    vector<int> indices;
    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud(boost::make_shared<pcl::PointCloud<PointType>>(incloud));
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stdThreshold);
    sor.setKeepOrganized(false);
    //boost::make_shared<pcl::PointIndices>(_pointIndices)= sor.getRemovedIndices();
    sor.filter(indices);
    //sor.filter(_indices);
    for (int i = 0; i < indices.size(); ++i)
    {
        pointindices->indices.push_back(indices[i]);
    }
}

void BoundaryPoints::pointcloud_projection(PointCloudType::Ptr incloud, PointCloudType &outcloud)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = 1;
    coefficients->values[1] = 0;
    coefficients->values[2] = 0;
    coefficients->values[3] = 0;
    pcl::ProjectInliers<pcl::PointXYZI> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(incloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(outcloud);
}
void BoundaryPoints::ransac_curve(PointCloudType::Ptr incloud, float residualThreshold,
                                  PointCloudType::Ptr outcloud)
{
    srand(time(NULL)); //time(NULL)获取当前时间,返回一个长整形数值
    //-------------------------------------------------------------- make sample data

    vector<double> x(incloud->points.size());
    vector<double> y(incloud->points.size());
    int nData = x.size();
    for (int i = 0; i < x.size(); i++)
    {
        x[i] = incloud->points[i].x;
        y[i] = incloud->points[i].y;
    }
    //-------------------------------------------------------------- build matrix
    cv::Mat A(nData, 3, CV_64FC1);
    cv::Mat B(nData, 1, CV_64FC1);

    for (int i = 0; i < nData; i++)
    {
        A.at<double>(i, 0) = x[i] * x[i];
    }
    for (int i = 0; i < nData; i++)
    {
        A.at<double>(i, 1) = x[i];
    }
    for (int i = 0; i < nData; i++)
    {
        A.at<double>(i, 2) = 1.0;
    }

    for (int i = 0; i < nData; i++)
    {
        B.at<double>(i, 0) = y[i];
    }

    //-------------------------------------------------------------- RANSAC fitting
    //    int n_data = 100 ;
    int N = 300;                  //iterations
    double T = residualThreshold; // residual threshold，拟合残差阈值

    int n_sample = 3;
    int max_cnt = 0;
    cv::Mat best_model(3, 1, CV_64FC1);

    for (int i = 0; i < N; i++)
    {
        //random sampling - 3 point
        int k[3] = {
            -1,
        };
        k[0] = floor((rand() % nData + 1)) + 1;

        do
        {
            k[1] = floor((rand() % nData + 1)) + 1;
        } while (k[1] == k[0] || k[1] < 0);

        do
        {
            k[2] = floor((rand() % nData + 1)) + 1;
        } while (k[2] == k[0] || k[2] == k[1] || k[2] < 0);

        //        printf("random sample : %d %d %d\n", k[0], k[1], k[2]) ;

        //model estimation
        cv::Mat AA(3, 3, CV_64FC1);
        cv::Mat BB(3, 1, CV_64FC1);
        for (int j = 0; j < 3; j++)
        {
            AA.at<double>(j, 0) = x[k[j]] * x[k[j]];
            AA.at<double>(j, 1) = x[k[j]];
            AA.at<double>(j, 2) = 1.0;

            BB.at<double>(j, 0) = y[k[j]];
        }

        cv::Mat AA_pinv(3, 3, CV_64FC1);
        invert(AA, AA_pinv, cv::DECOMP_SVD); //求AA逆矩阵

        cv::Mat X = AA_pinv * BB;

        //evaluation
        if (X.at<double>(0) < 0.1)
        {
            cv::Mat residual(nData, 1, CV_64FC1);
            residual = cv::abs(B - A * X);
            int cnt = 0; //内点计数
            for (int j = 0; j < nData; j++)
            {
                double data = residual.at<double>(j, 0);

                if (data < T)
                {
                    cnt++;
                }
            }

            if (cnt > max_cnt) //如果内点数量大于0,则作为best model
            {
                best_model = X;
                max_cnt = cnt;
            }
        }
    }

    //------------------------------------------------------------------- optional LS fitting
    cv::Mat residual = cv::abs(A * best_model - B);
    std::vector<int> vec_index; //存储模型内点索引
    for (int i = 0; i < nData; i++)
    {
        double data = residual.at<double>(i, 0);
        if (data < T) //如果残差小于阈值则作为内点
        {
            vec_index.push_back(i);
        }
    }

    cv::Mat A2(vec_index.size(), 3, CV_64FC1); //存储所有内点
    cv::Mat B2(vec_index.size(), 1, CV_64FC1);

    for (int i = 0; i < vec_index.size(); i++)
    {
        A2.at<double>(i, 0) = x[vec_index[i]] * x[vec_index[i]];
        A2.at<double>(i, 1) = x[vec_index[i]];
        A2.at<double>(i, 2) = 1.0;

        B2.at<double>(i, 0) = y[vec_index[i]];
    }

    cv::Mat A2_pinv(3, vec_index.size(), CV_64FC1);
    invert(A2, A2_pinv, cv::DECOMP_SVD);

    cv::Mat X = A2_pinv * B2; //利用所有内点再次进行拟合得到优化后的模型系数

    cv::Mat residual_opt = cv::abs(A * X - B);
    std::vector<int> vec_index_opt; //存储模型内点索引
    for (int i = 0; i < nData; i++)
    {
        double data = residual_opt.at<double>(i, 0);
        if (data < T) //如果残差小于阈值则作为内点
        {
            outcloud->points.push_back(incloud->points[i]);
        }
    }
    AINFO << "curb line model coefficient is " << X.at<double>(0) << " " << X.at<double>(1) << " " << X.at<double>(2) << endl;
}
void BoundaryPoints::process(PointCloudType::Ptr obstacleCloud, CloudPtrList clusterCloud)
{

    PointCloudType::Ptr cloud2D(new PointCloudType);
    PointCloudType::Ptr cloudFilted(new PointCloudType);
    PointCloudType::Ptr completeCloudLeft2D(new PointCloudType);
    PointCloudType::Ptr completeCloudRight2D(new PointCloudType);
    PointCloudType::Ptr completeCloudLeftFiltered(new PointCloudType);
    PointCloudType::Ptr completeCloudRightFiltered(new PointCloudType);
    pcl::PointIndices::Ptr leftIndices(new pcl::PointIndices);
    pcl::PointIndices::Ptr rightIndices(new pcl::PointIndices);
    pcl::PointIndices::Ptr cloudIndices(new pcl::PointIndices);

    CloudPtrList clusterPtrLR(2);
    for (int i = 0; i < clusterPtrLR.size(); ++i)
    {
        clusterPtrLR[i] = boost::make_shared<PointCloudType>();
    }

    //移除车顶干扰点
    PointCloudType::Ptr obstacleCloudFiltered(new PointCloudType);
    for (int i = 0; i < obstacleCloud->points.size(); ++i)
    {
        if ((pow(obstacleCloud->points[i].x, 2) + pow(obstacleCloud->points[i].y, 2)) >= 7)
        {
            obstacleCloudFiltered->points.push_back(obstacleCloud->points[i]);
        }
    }

    RoadSegmentation mycluster(obstacleCloudFiltered);
    mycluster.process(_cloud, clusterPtrLR);
    AINFO << "left candidate points is" << clusterPtrLR[0]->points.size() << endl;
    AINFO << "right candidate points is " << clusterPtrLR[1]->points.size() << endl;

  
    PointCloudType::Ptr pointcloud_distancefiltered(new PointCloudType);
    PointCloudType::Ptr pointcloud_distanceleftfiltered(new PointCloudType);
    PointCloudType::Ptr pointcloud_distancerightfiltered(new PointCloudType);

    GridMap my_mapL(clusterPtrLR[0], _gridRes, _gridNum);
    my_mapL.distanceFilterByCartesianGrid(pointcloud_distanceleftfiltered, true);
    for (int i = 0; i < pointcloud_distanceleftfiltered->points.size(); ++i)
    {
        pointcloud_distanceleftfiltered->points[i].intensity = i;
    }

    GridMap my_mapR(clusterPtrLR[1], _gridRes, _gridNum);
    my_mapR.distanceFilterByCartesianGrid(pointcloud_distancerightfiltered, false);
    for (int i = 0; i < pointcloud_distancerightfiltered->points.size(); ++i)
    {
        pointcloud_distancerightfiltered->points[i].intensity = i;
    }

    *pointcloud_distancefiltered += *pointcloud_distanceleftfiltered;
    *pointcloud_distancefiltered += *pointcloud_distancerightfiltered;

    AINFO << "distance filter left is " << pointcloud_distanceleftfiltered->points.size() << endl;
    AINFO << "distance filter right is " << pointcloud_distancerightfiltered->points.size() << endl;

    //将道路最近边界点分成前后两段用于拟合

    // PointCloudType::Ptr LeftFrontcloud(new PointCloudType);
    // PointCloudType::Ptr LeftRearcloud(new PointCloudType);
    // PointCloudType::Ptr RightFrontcloud(new PointCloudType);
    // PointCloudType::Ptr RightRearcloud(new PointCloudType);

    // for (int i = 0; i < pointcloud_distanceleftfiltered->points.size(); ++i)
    // {
    //     if (pointcloud_distanceleftfiltered->points[i].x >= 0)
    //     {
    //         LeftFrontcloud->points.push_back(pointcloud_distanceleftfiltered->points[i]);
    //     }
    //     else
    //     {
    //         LeftRearcloud->points.push_back(pointcloud_distanceleftfiltered->points[i]);
    //     }
    // }

    // for (int i = 0; i < pointcloud_distancerightfiltered->points.size(); ++i)
    // {
    //     if (pointcloud_distancerightfiltered->points[i].x >= 0)
    //     {
    //         RightFrontcloud->points.push_back(pointcloud_distancerightfiltered->points[i]);
    //     }
    //     else
    //     {
    //         RightRearcloud->points.push_back(pointcloud_distancerightfiltered->points[i]);
    //     }
    // }

    // PointCloudType::Ptr LeftFront_filtered(new PointCloudType);
    // PointCloudType::Ptr LeftRear_filtered(new PointCloudType);
    // PointCloudType::Ptr RightFront_filtered(new PointCloudType);
    // PointCloudType::Ptr RightRear_filtered(new PointCloudType);
   
    // pcl::PointIndicesPtr LeftFront_indices(new pcl::PointIndices);
    // pcl::PointIndicesPtr LeftRear_indices(new pcl::PointIndices);
    // pcl::PointIndicesPtr RightFront_indices(new pcl::PointIndices);
    // pcl::PointIndicesPtr RightRear_indices(new pcl::PointIndices);
     PointCloudType::Ptr Leftcloud_initial(new PointCloudType);
    PointCloudType::Ptr Rightcloud_initial(new PointCloudType);


    // if (_use_curve_fit)
    // {
    //     ransac_curve(LeftFrontcloud,_curveFitThres,LeftFront_filtered);
    //     ransac_curve(LeftRearcloud,_curveFitThres,LeftRear_filtered);
    //     ransac_curve(RightFrontcloud,_curveFitThres,RightFront_filtered);
    //     ransac_curve(RightRearcloud,_curveFitThres,RightRear_filtered);
    // }
    // else
    // {
    //     lineFitRansac(*LeftFrontcloud,*LeftFront_indices);
    //     extractPointCloud(*LeftFrontcloud,LeftFront_indices,*LeftFront_filtered);

    //     lineFitRansac(*LeftRearcloud,*LeftRear_indices);
    //     extractPointCloud(*LeftRearcloud,LeftRear_indices,*LeftRear_filtered);

    //     lineFitRansac(*RightFrontcloud,*RightFront_indices);
    //     extractPointCloud(*RightFrontcloud,RightFront_indices,*RightFront_filtered);

    //     lineFitRansac(*RightRearcloud,*RightRear_indices);
    //     extractPointCloud(*RightRearcloud,RightRear_indices,*RightRear_filtered);

    // }

    // *Leftcloud_initial=*LeftFront_filtered+*LeftRear_filtered;
    // *Rightcloud_initial=*RightFront_filtered+*RightRear_filtered;
    
    
      
    ransac_curve(pointcloud_distanceleftfiltered, _curveFitThres, Leftcloud_initial);
    ransac_curve(pointcloud_distancerightfiltered, _curveFitThres, Rightcloud_initial);

    GaussianProcess my_gauLeft(pointcloud_distanceleftfiltered, Leftcloud_initial,
                                _varThres, _meanThres);
    my_gauLeft.process(clusterCloud[0]);
    GaussianProcess my_gauRight(pointcloud_distancerightfiltered, Rightcloud_initial,
                                 _varThres, _meanThres);
    my_gauRight.process(clusterCloud[1]);

  
    AINFO << "before gaussian left point is " << Leftcloud_initial->points.size() << endl;
    AINFO << "after gaussian left point is " << clusterCloud[0]->points.size() << endl;
    AINFO << "before gaussian right point is " << Rightcloud_initial->points.size() << endl;
    AINFO << "after gaussian right point is " << clusterCloud[1]->points.size() << endl;

}
