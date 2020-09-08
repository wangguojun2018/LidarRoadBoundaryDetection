
#ifndef FUSION_KF_H_
#define FUSION_KF_H_

#include <vector>
#include <string>
#include <fstream>
#include"myfilter.h"
#include<pcl/point_types.h>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;
class Tracking
{
public:
    Tracking();
	virtual ~Tracking();
    void ProcessMeasurement(vector<pcl::PointXYZ> &curbPoint, Matrix4d frameTransform,bool ismeasurement);
    myFilter kf_;

private:
	bool is_initialized_;
//	long previous_timestamp_;

	//acceleration noise components
//	float noise_ax;
//	float noise_ay;

};

#endif /* FUSION_KF_H_ */

