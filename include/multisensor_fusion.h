//
// Created by ccg on 23-2-14.
//

#ifndef ESTIMATOR_MULTISENSOR_FUSION_H
#define ESTIMATOR_MULTISENSOR_FUSION_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <map>
#include <vector>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <mutex>

class MultiSensorOpt {
public:

    void GpsCallBack(const sensor_msgs::NavSatFixConstPtr &gps_msg);

    void OdomCallBack(const nav_msgs::OdometryConstPtr &odom_msg);

    void InputGps(double dt, double latidute, double longtitude, double height, double latcov, double longcov,
                  double hgtcov);

    void InputOdom(double t, Eigen::Quaterniond odom_q, Eigen::Vector3d odom_p);

    void GetGlobalOdom();

    void
    Update(const Eigen::Vector3d &last_odom_p, const Eigen::Quaterniond &last_odom_q, const Eigen::Vector3d &gps_p);

private:
    void GpsToXYZ();

    void AddDeltaXToState(const Eigen::Matrix<double, 15, 1> &delta_x);

    //void updatePath();
    void ComputeJacabianAndResidual(Eigen::Matrix<double, 3, 15> &H, Eigen::Vector3d &residual);

    std::map<double, std::vector<double>> local_pose;
    std::map<double, std::vector<double>> global_pose;
    std::map<double, std::vector<double>> gps_pose;
    std::queue<sensor_msgs::NavSatFixConstPtr> gps_msg_queue;
    std::mutex gps_lock;
    Eigen::Vector3d Tog;  //gps->odom
};


#endif //ESTIMATOR_MULTISENSOR_FUSION_H
