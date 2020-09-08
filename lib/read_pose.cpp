#include "read_pose.h"


bool computePairNum_pose(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}


read_pose::read_pose(string txt_path,string pose_path):_txt_path(txt_path),_pose_path(pose_path)
{
   _file_lists.resize(294);
   _poses.resize(294);
   _eulerangles.resize(294);
}

void read_pose::read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
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


void read_pose::sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;
    std::sort(filists.begin(),filists.end(),computePairNum_pose);
}

//获取数据
int read_pose::getPoses(const char *fileName, vector<Matrix4d,aligned_allocator<Matrix4d>>& poses)
{
    ifstream fileStream;
    string oneLine = "";	//输入文件的某一行
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
        int rowCount=0;	// 行数计数器
        int colCount =0;	// 列数计数器

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
            poses[pose_index](row_index,col_index)=tmp;
            ++Count;

        }
        //std::cout<<"pose index is "<<pose_index<<std::endl;
        // 关闭文件
        fileStream.close();
        return 1 ;
    }
}
int read_pose::get_calibration_imu_to_velodyne(const char * fileName, Matrix4d& Tr_velo_cam)
{
    ifstream fileStream;
    // 打开文件
    fileStream.open(fileName,ios::in);	//ios::in 表示以只读的方式读取文件
    if(!fileStream.is_open())//文件打开失败:返回0
    {
        cout << "calib文件不存在." << endl;
        fileStream.close();
        //system("pause");
        return 0;
    }
    else//文件存在
    {
        // 读入数据
        string s;
        double tmp;
        bool is_velo_cam=false;
        //当前读入的数值
        int i=0;
        while (!fileStream.eof())
        {

            fileStream>>tmp;
            //            cout<<"read line is "<<s<<endl;
            Tr_velo_cam(i/4,i%4)=tmp;
            ++i;
//            if(is_velo_cam)
//            {
//                stringstream ss;
//                ss<<s;
//                ss>>tmp;
//                Tr_velo_cam(i/4,i%4)=tmp;
//                ++i;
//                cout<<"double tmp is "<<tmp<<endl;
//            }
//            if(s=="Tr:")
//            {
//                is_velo_cam=true;
//            }
        }
        //std::cout<<"pose index is "<<pose_index<<std::endl;
        // 关闭文件

        fileStream.close();
    }
    return 0;
}
//读取旋转角
Vector3d read_pose::getAngles(const char *fileName)
{

    Vector3d eulerangle;
    ifstream fileStream;
    //string oneLine = "";	//输入文件的某一行
    // 打开文件
    fileStream.open(fileName,ios::in);	//ios::in 表示以只读的方式读取文件
    if(!fileStream.is_open())//文件打开失败:返回0
    {
       cout << "angle文件不存在." <<endl;
        fileStream.close();
        //system("pause");
        return eulerangle;
    }
    else//文件存在
    {

        int Count=0;
        double tmp = 0;
        //当前读入的数值
        while (!fileStream.eof()&&Count<=5)
        {

            fileStream >> tmp;

            //cout<<"tmp is "<<tmp<<"Count is "<<Count<<endl;
            if(Count==3)
            {
                eulerangle(0)=tmp;
            }
            else if (Count==4)
            {
                eulerangle(1)=tmp;
            }
            else if (Count==5)
            {
                eulerangle(2)=tmp;
            }
            else
            {
                //continue;
            }
            ++Count;
        }

        // 关闭文件
        fileStream.close();
        return eulerangle ;
    }
}

void read_pose::process(vector<Matrix<double,2,3>,aligned_allocator<Matrix<double,2,3>> >& transforms)
{
    read_filelists( _txt_path, _file_lists, "txt" );
    sort_filelists( _file_lists, "txt" );
    getPoses(_pose_path.c_str(),_poses);
    for(int i=0;i<_file_lists.size();++i)
    {
        string file_path=_txt_path+_file_lists[i];
        _eulerangles[i]=getAngles(file_path.c_str());
        //cout<<"eulerangle i is \n"<<_eulerangles[i]<<endl;
       transforms[i].row(0)=_poses[i].block<3,1>(0,3);
       transforms[i].row(1)=_eulerangles[i];

    }
    for(int i=0;i<transforms.size()-1;++i)
    {
        transforms[i].row(0)=transforms[i+1].row(0)-transforms[i].row(0);
        transforms[i].row(1)=transforms[i+1].row(1)-transforms[i].row(1);
    }
    transforms[transforms.size()-1].row(0)=transforms[transforms.size()-2].row(0);
    transforms[transforms.size()-1].row(1)=transforms[transforms.size()-2].row(1);

}
