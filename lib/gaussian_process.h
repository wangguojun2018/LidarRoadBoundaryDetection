/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-09-07 11:13:17
 */
#ifndef GAUSSIAN_PROCESS_H
#define GAUSSIAN_PROCESS_H

// #include<pcl/point_cloud.h>
// #include<pcl/point_types.h>
// #include<vector>
#include <boost/filesystem.hpp>
#include <limbo/kernel/exp.hpp>
#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/data.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/tools.hpp>
#include <limbo/tools/macros.hpp>
#include<Eigen/Eigen>
#include <limbo/serialize/text_archive.hpp>
#include <lib/types.h>


using namespace std;
using namespace limbo;
using namespace Eigen;


class GaussianProcess
{
public:
    GaussianProcess(PointCloudType::Ptr candidatePoints,
                     PointCloudType::Ptr leftInitialPoints,
                     float varThres, float meanThres);
    vector<int> vectors_difference(vector<int> v1, vector<int> v2);
    void process(PointCloudType::Ptr clusterCloud);
    void initialTrainData();

private:
    PointCloudType _candidatePoints;
    PointCloudType _leftInitialPoints;
//    pcl::PointCloud<pcl::PointXYZI> _rightInitialPoints;
    vector<VectorXd> _xLeft;//存储内点数据
    vector<VectorXd> _yLeft;


    vector<VectorXd> _remainX;//存储未处理点数据
    vector<VectorXd>_remainY;
    vector<int> _leftIndex;
//    vector<int> _rightIndex;
    vector<int> _remainIndex;
    vector<int> _candidateIndex;

    float _meanThres;
    float _varThres;

    struct Params {
        struct kernel_exp
        {
            BO_PARAM(float, sigma_sq, 38.1416);
            BO_PARAM(float, l, 16.1003);
//            BO_PARAM(double, noise,0.0039);
        };
        struct kernel : public defaults::kernel
        {
        };
        struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard
        {
        };
        struct opt_rprop : public defaults::opt_rprop
        {
        };
    };

};

#endif // GAUSSIAN_PROCESS_H
