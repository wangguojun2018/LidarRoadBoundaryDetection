/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-09-07 19:41:51
 */
#include "bin_reader.h"
#include<lib/log.h>

bool computePairNum_bin(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}


BinReader::BinReader(BinReaderMsg msg)
{
   _bin_dir=msg.bindir();
   read_filelists( _bin_dir, _file_lists, "bin" );
   sort_filelists( _file_lists);
}

void BinReader::Init(BinReaderMsg msg)
{
   _bin_dir=msg.bindir();
   read_filelists( _bin_dir, _file_lists, "bin" );
   sort_filelists( _file_lists);
}
 int BinReader::getNumberofFile()
 {
     return _file_lists.size();
 }
void BinReader::read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL)
    {
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')
            continue;

        if (type.size() <= 0)
        {
            out_filelsits.push_back(ptr->d_name);
        }
        else
        {
            if (tmp_file.size() < type.size())
                continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type)
            {
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
    closedir(dir);
}



void BinReader::sort_filelists(std::vector<std::string>& filists)
{
    if (filists.empty())return;
    std::sort(filists.begin(),filists.end(),computePairNum_bin);
}

void BinReader::readKittiPclBinData(std::string &in_file, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{

    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    int i;
    for (i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        cloud->push_back(point);
    }
    input.close();

}

// void BinReader::my_function(CloudPtrList cloud_list)
// {

//     for (int i = 0; i < _file_lists.size(); ++i)
//     {
//         std::string bin_file = _bin_dir + _file_lists[i];
        
//         readKittiPclBinData( bin_file,cloud_list[i]);
//     }
// }
void BinReader::process(PointCloudType::Ptr cloud,int i)
{
        if (_bin_dir.substr(_bin_dir.size()-1,1)!="/")
        {
            _bin_dir=_bin_dir+"/";
        }
        
        std::string bin_file = _bin_dir + _file_lists[i];
        // std::string tmp_str = _file_lists[i].substr(0, _file_lists[i].length() - 4) + ".pcd";
        // std::string pcd_file = _pcd_path + tmp_str;
        readKittiPclBinData( bin_file,cloud);
       AINFO<<" 当前帧ID为： "<<_file_lists[i]<<endl;

}
