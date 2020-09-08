#include "read_frametransform.h"

read_frameTransform::read_frameTransform(string filename):_TrOneFrame_path(filename)
{
  _frameTransforms.resize(294);
}

bool computePairNum_pose(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}

void read_frameTransform::read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
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


void read_frameTransform::sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;
    std::sort(filists.begin(),filists.end(),computePairNum_pose);
}

//获取数据
int read_frameTransform::getInputData(const char *fileName)
{
    ifstream fileStream;
//    string oneLine = "";	//输入文件的某一行
    // 打开文件
    fileStream.open(fileName,ios::in);	//ios::in 表示以只读的方式读取文件
    if(fileStream.fail())//文件打开失败:返回0
    {
        cout << "pose文件不存在." << endl;
        fileStream.close();
        //system("pause");
        return 0;
    }
    else//文件存在
    {
        // 读入数据
        //double tmp = 0;		//当前位置上的数值
        int Count=0;
//        int rowCount=0;	// 行数计数器
//        int colCount =0;	// 列数计数器

        int pose_index=0;
        int row_index=0;
        int col_index=0;
        //当前读入的数值
        while (!fileStream.eof())
        {
            double tmp = 0;
            fileStream >> tmp;

            //rowCount=Count/4+
            pose_index=Count/16;
            row_index=(Count/4)%4;
            col_index=(Count)%4;
            _frameTransforms[pose_index](row_index,col_index)=tmp;
            ++Count;

        }
        //std::cout<<"pose index is "<<pose_index<<std::endl;
        // 关闭文件
        fileStream.close();
        return 1 ;
    }
}

//读取旋转角

void read_frameTransform::process(vector<Matrix4d,aligned_allocator<Matrix4d>>& transforms)
{

    getInputData(_TrOneFrame_path.c_str());
    transforms=_frameTransforms;
}
