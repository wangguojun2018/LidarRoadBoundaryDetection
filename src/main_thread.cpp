#include <iostream>
#include<lib/bin_reader.h>
#include<lib/read_pose.h>
#include<lib/cloud_mapper.h>
#include<boost/make_shared.hpp>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/point_cloud_color_handlers.h>
#include<lib/ground_segment.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/PointIndices.h>
#include<pcl/console/parse.h>
#include<lib/feature_points.h>
#include<Eigen/Core>
#include<pcl/visualization/pcl_plotter.h>
#include<pcl/features/normal_3d_omp.h>
#include<pcl/features/don.h>
#include <pcl/filters/conditional_removal.h>
#include <lib/types.h>
#include<lib/boundary_points.h>
#include<sstream>
#include<google/protobuf/text_format.h>
#include<lib/log.h>
#include<proto/road_boundary.pb.h>

#define Eigen_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ASSERT

using namespace std;
using namespace Eigen;
using namespace RoadBoundary;


boost::mutex mtex;
bool update=false;

BinReader bin_reader;
RoadBoundaryMsg msg;

void curb_extract(PointCloudType::Ptr complete_points, CloudPtrList boundary_points)
{
    int i=0;

    while(i<bin_reader.getNumberofFile())
    {
        boost::mutex::scoped_lock lock(mtex);
        if(update==false)
        {
            update = true;
            complete_points->clear();
            for (size_t i = 0; i < boundary_points.size(); i++)
            {
                boundary_points[i]->clear();
            }
            
            AINFO << " now frame " << i << " begin processing" << endl;

            //读取bin点云文件
            PointCloudType::Ptr completeCloud(new PointCloudType);
            bin_reader.process(completeCloud, i);

            //根据kitti点云数据存储特点计算点云laserID,并将ID存为intensity
            CloudMapper mapper1(msg.cloudmappermsg());
            PointCloudType::Ptr completeCloudMapper(new PointCloudType);
            mapper1.processByOri(completeCloud, completeCloudMapper);
            AINFO << "raw points number is " << completeCloudMapper->points.size() << endl;

            //地面提取
            PointCloudType::Ptr ground_points(new PointCloudType);
            PointCloudType::Ptr ground_points_no(new PointCloudType); //非地面点
            GroundSegmentation ground(completeCloudMapper,msg.groundsegmentationmsg());
            ground.groundfilter(ground_points, ground_points_no);
            AINFO << "ground points is " << ground_points->points.size() << endl;
            AINFO << "no ground points is " << ground_points_no->points.size() << endl;

            //根据之前计算的Intensity对地面点云进行mapper
            CloudMapper mapper2(msg.cloudmappermsg());
            scanIndices scanIDindices;
            PointCloudType::Ptr ground_points_mapper(new PointCloudType);
            mapper2.processByIntensity(ground_points, ground_points_mapper, scanIDindices);
            AINFO << "ground points mapper is " << ground_points_mapper->points.size() << endl;

            //特征点提取
            pcl::PointCloud<pcl::PointXYZI>::Ptr featurePoints(new pcl::PointCloud<pcl::PointXYZI>);
            FeaturePoints curbExtract(ground_points_mapper, scanIDindices,msg.featurepointsmsg());
            curbExtract.extractFeatures(featurePoints);
            AINFO << "feature points is " << featurePoints->points.size() << endl;

            //高斯过程提取
            BoundaryPoints refinePoints(*featurePoints,msg.boundarypointsmsg());
    
            refinePoints.process(ground_points_no,boundary_points);
            *complete_points=*completeCloud;

            ++i;
        }

    }
}

int main(int argc,char** argv)
{


    GOOGLE_PROTOBUF_VERIFY_VERSION;

    if (argc != 2) 
    {
    cerr << "Usage:  " << argv[0] << " BOUNDARY_PROTO_FILE" << endl;
    return -1;
    }

    string msg_string;

    // Read the existing address book.
    fstream input(argv[1], ios::in);

    if (!input)
    {
        return false;
    }

    std::stringstream str_stream;
    str_stream << input.rdbuf();
    msg_string = str_stream.str();
    google::protobuf::TextFormat::ParseFromString(msg_string,&msg);

    bin_reader.Init(msg.binreadermsg());
    
    PointCloudType::Ptr complete_points(new PointCloudType);
    CloudPtrList boundary_points(2);
   
    for (int i = 0; i < boundary_points.size(); ++i)
    {
        boundary_points[i] = boost::make_shared<PointCloudType>();
    }

  
    //新建线程处理点云输出拟合直线
    boost::thread t1(boost::bind(curb_extract,complete_points,boundary_points));
    

    //可视化
    pcl::visualization::PCLVisualizer viewer("curb_viewer");
    int v1(0);
    int v2(0);
    viewer.setBackgroundColor(0,0,0);
    viewer.addCoordinateSystem(3);

    pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler_cloud_left(new pcl::visualization::PointCloudColorHandlerCustom<PointType>(255, 0, 0));
    pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler_cloud_right(new pcl::visualization::PointCloudColorHandlerCustom<PointType>(0, 0, 255));
    pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler_cloud(new pcl::visualization::PointCloudColorHandlerCustom<PointType>(255, 255, 255));

    int i=0;

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        boost::mutex::scoped_lock lock(mtex);

        if (update)

        {
            viewer.removeAllShapes(); //删掉形状缓存
            viewer.removeAllPointClouds();//删除点云缓存

           handler_cloud->setInputCloud(complete_points);
            viewer.addPointCloud(complete_points, *handler_cloud, "cloud", v1);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud", v1);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "cloud",v1);

            handler_cloud_left->setInputCloud(boundary_points[0]);
            viewer.addPointCloud(boundary_points[0], *handler_cloud_left, "left",v1);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "left", v1);


            handler_cloud_right->setInputCloud(boundary_points[1]);
            viewer.addPointCloud(boundary_points[1], *handler_cloud_right, "right",v1);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "right", v1);
            update=false;
        }
        ++i;
    }

    t1.join();
    return 0;

}