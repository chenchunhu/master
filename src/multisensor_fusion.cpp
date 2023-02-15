//
// Created by ccg on 23-2-14.
//
#include "multisensor_fusion.h"

void MultiSensorOpt::GpsCallBack(const sensor_msgs::NavSatFixConstPtr &gps_msg) {
    gps_lock.lock();
    gps_msg_queue.push(gps_msg);
    gps_lock.unlock();
}

void MultiSensorOpt::OdomCallBack(const nav_msgs::OdometryConstPtr &odom_msg) {
    double odom_t = odom_msg->header.stamp.toSec();
    Eigen::Quaterniond last_odom_q;
    Eigen::Vector3d last_odom_p;
    last_odom_p.x() = odom_msg->pose.pose.position.x;
    last_odom_p.y() = odom_msg->pose.pose.position.y;
    last_odom_p.z() = odom_msg->pose.pose.position.z;

    last_odom_q.w() = odom_msg->pose.pose.orientation.w;
    last_odom_q.x() = odom_msg->pose.pose.orientation.x;
    last_odom_q.y() = odom_msg->pose.pose.orientation.y;
    last_odom_q.z() = odom_msg->pose.pose.orientation.z;
    InputOdom(odom_t, last_odom_q, last_odom_p);

    gps_lock.lock();
    while (!gps_msg_queue.empty()) {
        sensor_msgs::NavSatFixConstPtr gps_msg = gps_msg_queue.front();
        double gps_t = gps_msg->header.stamp.toSec();
        //gps_msg_queue.pop();
        if (gps_t >= odom_t - 0.1 && gps_t <= odom_t + 0.1) {
            double latitude = gps_msg->latitude;
            double longtude = gps_msg->longitude;
            double alttude = gps_msg->altitude;
            double latcov = gps_msg->position_covariance[0];
            double longcov = gps_msg->position_covariance[4];
            double altcov = gps_msg->position_covariance[8];
            InputGps(odom_t, latitude, longtude, alttude, latcov, longcov, altcov);
            gps_msg_queue.pop();
        } else if (gps_t < odom_t - 0.1) {
            gps_msg_queue.pop();
        } else
            break;
    }
    gps_lock.unlock();
}

void MultiSensorOpt::InputGps(double dt, double latidute, double longtitude, double height, double latcov,
                              double longcov, double hgtcov) {

}

void MultiSensorOpt::Update(const Eigen::Vector3d &last_odom_p, const Eigen::Quaterniond &last_odom_q,
                            const Eigen::Vector3d &gps_p) {
    Eigen::Vector3d residual = gps_p - (last_odom_p + last_odom_q.toRotationMatrix() * Tog);
    Eigen::Matrix<double, 3, 15> H;
    ComputeJacabianAndResidual(H, residual);
}

void MultiSensorOpt::ComputeJacabianAndResidual(Eigen::Matrix<double, 3, 15> &H, Eigen::Vector3d &residual) {

    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(0, 6) =;
}