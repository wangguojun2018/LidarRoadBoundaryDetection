
#include "tracking.h"


Tracking::Tracking()
{
    is_initialized_ = false;
    //  previous_timestamp_ = 0;

    //create a 4D state vector, we don't know yet the values of the x state
    kf_.x_ = VectorXd(8);
    kf_.x_temp=VectorXd(8);
    //state covariance matrix P
    kf_.P_ = MatrixXd::Identity(8, 8);
    //    kf_.P_ << 0.5, 0, 0, 0,
    //            0, 0.5, 0, 0,
    //            0, 0, 0.5, 0,
    //            0, 0, 0, 0.5;


    //measurement covariance
    kf_.R_ = MatrixXd::Identity(8, 8);
    //    kf_.R_ << 1, 0, 0, 0,
    //            0, 1, 0, 0,
    //            0, 0, 1, 0,
    //            0, 0, 0, 0.2;
    kf_.Q_ = MatrixXd::Identity(8, 8);
    kf_.Q_=5*kf_.Q_;
    //    kf_.Q_ <<  1, 0, 0, 0,
    //            0, 1, 0, 0,
    //            0, 0, 1, 0,
    //            0, 0, 0, 0.2;
    //measurement matrix
    kf_.H_ = 1;
    //    kf_.H_ << 1;

    //the initial transition matrix F_
    kf_.F_ = MatrixXd::Zero(8, 8);
    //  kf_.F_ << 1, 0, 0, 0,
    //            0, 1, 0, 0,
    //            0, 0, 1, 0,
    //            0, 0, 0, 1;

    //set the acceleration noise components
    //  noise_ax = 5;
    //  noise_ay = 5;

}

Tracking::~Tracking() {

}

// Process a single measurement
void Tracking::ProcessMeasurement(vector<pcl::PointXYZ>&curbPoint,Matrix4d frameTransform,bool ismeasurement)
{
    //    pcl::PointXYZ pointFiltered;
    if (!is_initialized_)
    {
        //cout << "Kalman Filter Initialization " << endl;

        //set the state with the initial location and zero velocity

        kf_.x_ << curbPoint[0].x,curbPoint[0].y,curbPoint[0].z,1,curbPoint[1].x,curbPoint[1].y,curbPoint[1].z,1;

        //    previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    Matrix4d frameTransformInverse=frameTransform.inverse();
    for(int i=0;i<curbPoint.size();++i)
    {
        kf_.F_.block<4,4>(4*i,4*i)=frameTransformInverse;
    }

    //    kf_.F_.block<4,4>(0,0)=frameTransformInverse;
    //    kf_.F_.block<4,4>(4,4)=frameTransformInverse;


    //set the process covariance matrix Q
    //predict
    kf_.Predict();
    if(ismeasurement)
    {
        //measurement update
        VectorXd tempPoint(8);
        tempPoint<<curbPoint[0].x,curbPoint[0].y,curbPoint[0].z,1,curbPoint[1].x,curbPoint[1].y,curbPoint[1].z,1;
        kf_.Update(tempPoint);
    }

    std::cout << "x_= " << kf_.x_ << std::endl;
    std::cout << "P_= " << kf_.P_ << std::endl;

    curbPoint[0].x=kf_.x_(0);
    curbPoint[0].y=kf_.x_(1);
    curbPoint[0].z=kf_.x_(2);
    curbPoint[1].x=kf_.x_(4);
    curbPoint[1].y=kf_.x_(5);
    curbPoint[1].z=kf_.x_(6);
    return;


}

