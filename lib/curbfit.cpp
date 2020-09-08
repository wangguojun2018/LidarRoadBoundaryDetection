#include "curbfit.h"


curbFit::curbFit(pcl::PointCloud<PointT>::Ptr incloud):_lineCloud(*incloud)
{

    //    for(int i=0;i<incloud->points.size();++i)
    //    {
    //        _inputData3d.push_back(Point3f(incloud->points[i].x,incloud->points[i].y,incloud->points[i].z));
    //    }

}

void curbFit::process(vector<pcl::PointXYZ>& line3d)
{
    //    fitLine(_inputData,_line,CV_DIST_FAIR,0,0.01,0.01);
    //    line2d[1]=_line[1]/_line[0];
    //    line2d[0]=_line[3]-_line[1]/_line[0]*_line[2];
    //    fitLine(_inputData3d,_line3d,CV_DIST_HUBER,0,1e-2,1e-2);
    //    line3d->values.resize(6);
    //    for(int i=0;i<3;++i)
    //    {
    //        line3d->values[i]=_line3d[3+i];
    //    }
    //    for(int i=3;i<6;++i)
    //    {
    //        line3d->values[i]=_line3d[i-3];
    //    }
    //        line3d[0].x=-2;
    //        line3d[0].y=(line3d[0].x-_line3d[3])/_line3d[0]*_line3d[1]+_line3d[4];
    //        line3d[0].z=(line3d[0].x-_line3d[3])/_line3d[0]*_line3d[2]+_line3d[5];
    //        line3d[1].x=25;
    //        line3d[1].y=(line3d[1].x-_line3d[3])/_line3d[0]*_line3d[1]+_line3d[4];
    //        line3d[1].z=(line3d[1].x-_line3d[3])/_line3d[0]*_line3d[2]+_line3d[5];
    pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
    lineFitRansac(model);
    //计算本车与路沿的距离

    //    distanceCurb=(0-model->values[0])/model->values[3]*model->values[4]+model->values[1];
    //根据模型系数计算边界线端点
    line3d[0].x=-10;
    line3d[0].y=(line3d[0].x-model->values[0])/model->values[3]*model->values[4]+model->values[1];
    line3d[0].z=(line3d[0].x-model->values[0])/model->values[3]*model->values[5]+model->values[2];
    line3d[1].x=25;
    line3d[1].y=(line3d[1].x-model->values[0])/model->values[3]*model->values[4]+model->values[1];
    line3d[1].z=(line3d[1].x-model->values[0])/model->values[3]*model->values[5]+model->values[2];
}
void curbFit::lineFitRansac(pcl::ModelCoefficients::Ptr model)
{
    pcl::PointIndices indices;
    Vector3f axis(1,0,0);
    //    pcl::ModelCoefficients model;
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_LINE);
    seg.setAxis(axis);
    seg.setEpsAngle(0.05);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.08);
    seg.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> >(_lineCloud));
    seg.segment(indices,*model);
}
// void curbFit::PointCloud2Vector2d (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::on_nurbs::vector_vec2d &data)
// {
//     for (unsigned i = 0; i < cloud->size (); i++)
//     {
//         pcl::PointXYZI &p = cloud->at (i);
//         if (!pcl_isnan (p.x) && !pcl_isnan (p.y))
//             data.push_back (Eigen::Vector2d (p.x, p.y));
//     }
// }
// void curbFit::nurbFit(pcl::PointCloud<pcl::PointXYZRGB>::Ptr curveCloud)
// {
//     pcl::on_nurbs::NurbsDataCurve2d data;
//     PointCloud2Vector2d (boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >(_lineCloud), data.interior);
//     unsigned order (2);
//     unsigned n_control_points (10);

//     pcl::on_nurbs::FittingCurve2d::Parameter curve_params;
//     curve_params.smoothness = 0.1;
//     curve_params.rScale = 1.0;
//     // #################### CURVE FITTING #########################
//     ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2d::initNurbsPCA (order, &data, n_control_points);

//     pcl::on_nurbs::FittingCurve2d fit (&data, curve);
//     fit.assemble (curve_params);

//     //    Eigen::Vector2d fix1 (0.1, 0.1);
//     //    Eigen::Vector2d fix2 (1.0, 0.0);
//     fit.solve ();
//     pcl::on_nurbs::Triangulation::convertCurve2PointCloud (fit.m_nurbs, curveCloud, 8);
// }
void curbFit::ransac_curve(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,double residualThreshold, vector<pcl::PointXYZ>& outPoints,bool isleft)
{

    //-------------------------------------------------------------- make sample data

    vector<double> x(incloud->points.size());
    vector<double> y(incloud->points.size()) ;
    int nData=incloud->points.size();
    for( int i=0 ; i<x.size() ; i++ )
    {
        x[i] = incloud->points[i].x;
        y[i] = incloud->points[i].y;
    }
    cv::Mat A2(nData,3, CV_64FC1) ;//存储所有内点
    cv::Mat B2(nData,1, CV_64FC1) ;

    for( int i=0 ; i<nData ; i++ )
    {
        A2.at<double>(i,0) = pow(x[i],2);
        A2.at<double>(i,1) = pow(x[i],1);
        A2.at<double>(i,2) = pow(x[i],0);
        //        A2.at<double>(i,3) = pow(x[i],1);
        //        A2.at<double>(i,4) = 1.0 ;

        B2.at<double>(i,0) = y[i] ;
    }

    cv::Mat A2_pinv(3,nData,CV_64FC1) ;
    invert(A2, A2_pinv, cv::DECOMP_SVD);

    cv::Mat X = A2_pinv * B2 ;//利用所有内点再次进行拟合得到优化后的模型系数
    float zPoints=incloud->points[nData/2].z;
    int j=0;
    for(float i=-50;i<50;i=i+2)
    {

        pcl::PointXYZ point;
        float yPoint=X.at<double>(0)*pow(i,2)+X.at<double>(1)*pow(i,1)+
                X.at<double>(2)*pow(i,0);
        point.x=i;
        point.y=yPoint;
        point.y=yPoint;
        point.z=zPoints;
        outPoints[j]=point;
        ++j;
    }
    cout<<"curb line model coefficient is "<<X.at<double>(0)<<" "<<X.at<double>(1)<<" "<<X.at<double>(2)<<endl;
}
