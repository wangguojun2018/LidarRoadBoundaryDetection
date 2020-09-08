/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-09-04 12:03:21
 */


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>



typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef std::vector<PointCloudType::Ptr > CloudPtrList;
typedef std::vector<PointCloudType> CloudList;


typedef std::pair<size_t, size_t> IndexRange;
typedef std::vector<IndexRange> scanIndices;
