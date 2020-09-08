#include <iostream>
#include <boost/make_shared.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/console/parse.h>
#include <eigen3/Eigen/Core>
#include <pcl/visualization/pcl_plotter.h>

#include <lib/bin_reader.h>
#include <lib/read_pose.h>
#include <lib/cloud_mapper.h>
#include <lib/rectify.h>
#include <lib/ground_segment.h>
#include <lib/feature_points.h>
#include <lib/curbfit.h>
#include <lib/tracking.h>
#include <lib/boundary_points.h>
#include <omp.h>
#include <proto/road_boundary.pb.h>
#include <lib/types.h>
#include<lib/log.h>
#include <sstream>
#include<google/protobuf/text_format.h>


using namespace std;
using namespace Eigen;
using namespace RoadBoundary;


int main(int argc, char *argv[])
{

    GOOGLE_PROTOBUF_VERIFY_VERSION;

    if (argc != 2) 
    {
        AERROR << "Usage:  " << argv[0] << " BOUNDARY_PROTO_FILE" << endl;
        return -1;
    }

    RoadBoundaryMsg msg;
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


    
   
    //读取bin文件。
    PointCloudType::Ptr complete_cloud(new PointCloudType);
    BinReader file_reader(msg.binreadermsg());
    file_reader.process(complete_cloud, msg.binreadermsg().framenum());

    //根据kitti点云数据存储特点计算点云laserID,并将ID存为intensity
    CloudMapper mapper1(msg.cloudmappermsg());
    PointCloudType::Ptr completeCloudMapper(new PointCloudType);
    mapper1.processByOri(complete_cloud, completeCloudMapper);
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
//    curb_point curbExtract(ground_points_mapper, scanIDindices);
    curbExtract.extractFeatures(featurePoints);
    AINFO << "feature points is " << featurePoints->points.size() << endl;

    //  创建curb_refine对象
    BoundaryPoints refinePoints(*featurePoints,msg.boundarypointsmsg());
    CloudPtrList clusterPtr(2);
   
    for (int i = 0; i < clusterPtr.size(); ++i)
    {
        clusterPtr[i] = boost::make_shared<PointCloudType>();
    }

    refinePoints.process(ground_points_no, clusterPtr);
   

    //可视化
    pcl::visualization::PCLVisualizer viewer("curb_viewer");
    int v1(0);
    int v2(0);

    // viewer.createViewPort(0,0,0.5,1,v1);
    // viewer.createViewPort(0.5,0,1,1,v2);

    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(2.5);
    pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler_cloud_left(new pcl::visualization::PointCloudColorHandlerCustom<PointType>(255, 0, 0));
    pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler_cloud_right(new pcl::visualization::PointCloudColorHandlerCustom<PointType>(0, 0, 255));
    pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler_cloud(new pcl::visualization::PointCloudColorHandlerCustom<PointType>(255, 255, 255));

    //    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> ::Ptr handler_cloud(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>("intensity"));
    handler_cloud->setInputCloud(completeCloudMapper);
    viewer.addPointCloud(completeCloudMapper, *handler_cloud, "cloud", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "cloud",v1);

    handler_cloud_left->setInputCloud(clusterPtr[0]);
    viewer.addPointCloud(clusterPtr[0], *handler_cloud_left, "left",v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "left", v1);


    handler_cloud_right->setInputCloud(clusterPtr[1]);
    viewer.addPointCloud(clusterPtr[1], *handler_cloud_right, "right",v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "right", v1);
    

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }


    return 0;


    google::protobuf::ShutdownProtobufLibrary();

}
