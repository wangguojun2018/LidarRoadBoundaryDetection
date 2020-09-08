/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-09-07 19:40:34
 */
#ifndef BIN_READER_H
#define BIN_READER_H

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<vector>
#include<string>
#include<dirent.h>
#include<fstream>
#include <lib/types.h>
#include <proto/road_boundary.pb.h>


using namespace std;
using namespace RoadBoundary;

class BinReader
{
public:
    BinReader(BinReaderMsg msg);
    BinReader(){};
    void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type);
    void sort_filelists(std::vector<std::string>& filists);
    void Init(BinReaderMsg msg);
    void readKittiPclBinData(std::string &in_file, PointCloudType::Ptr cloud);
    void my_function(CloudPtrList cloud_list);
    void process(PointCloudType::Ptr cloud,int i);
    int getNumberofFile();

private:
    vector<string> _file_lists;
    string _bin_dir;
};
#endif // BIN_READER_H
